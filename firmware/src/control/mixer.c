/* mixer.c — see mixer.h.
 *
 * Allocation:  motor = Bpinv · wrench, with Bpinv = pinv(B) = B^T (B B^T)^-1
 * (B is 4 x n, full row rank for any real multirotor). Computed once at
 * geometry-set time. Per-cycle cost is the matrix-vector product + the
 * desaturation passes — a few dozen FLOPs, trivial on the F4 FPU.
 *
 * Desaturation shifts the allocation along a "desaturation direction" (a column
 * of Bpinv — how the motors move per unit of one wrench axis) by the gain that
 * best removes the saturation, sacrificing thrust then yaw so roll/pitch torque
 * is preserved.
 */
#include "control/mixer.h"
#include "maths/linalg.h"   /* m_mat4_inv, m_fabsf, m_isnan — the math backend */
#include <stddef.h>

/* ---- geometry / pseudo-inverse ------------------------------------------- */

bool mixer_set_geometry(mixer_t *mx, const float *pos_x, const float *pos_y,
                        const int *spin, uint8_t n) {
  if (mx == NULL || n == 0 || n > MIXER_MAX_MOTORS) return false;
  mx->n = n;
  mx->idle_floor = 0.0f;
  mx->airmode = MIXER_AIRMODE_DISABLED;

  /* Effectiveness rows use the geometry sign convention:
   *   roll  = -sign(y)   pitch = +sign(x)   yaw = +spin   thrust = 1 */
  for (uint8_t i = 0; i < n; i++) {
    mx->B[MIX_ROLL][i]   = (pos_y[i] >= 0.0f) ? -1.0f : +1.0f;
    mx->B[MIX_PITCH][i]  = (pos_x[i] >= 0.0f) ? +1.0f : -1.0f;
    mx->B[MIX_YAW][i]    = (spin[i] >= 0)     ? +1.0f : -1.0f;
    mx->B[MIX_THRUST][i] = 1.0f;
  }

  /* G = B B^T  (4x4) */
  float G[4][4];
  for (int a = 0; a < MIX_NW; a++)
    for (int b = 0; b < MIX_NW; b++) {
      float s = 0.0f;
      for (uint8_t i = 0; i < n; i++) s += mx->B[a][i] * mx->B[b][i];
      G[a][b] = s;
    }

  float Ginv[4][4];
  if (!m_mat4_inv(&G[0][0], &Ginv[0][0])) return false;

  /* Bpinv = B^T Ginv   (n x 4) */
  for (uint8_t i = 0; i < n; i++)
    for (int a = 0; a < MIX_NW; a++) {
      float s = 0.0f;
      for (int b = 0; b < MIX_NW; b++) s += mx->B[b][i] * Ginv[b][a];
      mx->Bpinv[i][a] = s;
    }

  /* Normalise so the collective passes through ~1:1 per motor: scale the whole
   * pseudo-inverse so the THRUST column sums to n. For a symmetric quad-X this
   * makes Bpinv the sign matrix (thrust col all 1, torque cols the ±1 signs),
   * i.e. unit per-axis gain. */
  float thr_sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) thr_sum += mx->Bpinv[i][MIX_THRUST];
  if (m_fabsf(thr_sum) < 1e-9f) return false;
  float g = (float)n / thr_sum;
  for (uint8_t i = 0; i < n; i++)
    for (int a = 0; a < MIX_NW; a++) mx->Bpinv[i][a] *= g;

  /* After scaling, B·Bpinv = g·I, so the delivered wrench is (B·motor)/g. */
  mx->fwd_scale = 1.0f / g;
  return true;
}

void mixer_set_airmode(mixer_t *mx, mixer_airmode_t mode) {
  if (mx) mx->airmode = mode;
}
void mixer_set_idle_floor(mixer_t *mx, float idle) {
  if (mx) mx->idle_floor = idle;
}

/* ---- desaturation -------------------------------------------------------- */

/* The gain k such that motor += k*dir best removes the [min,max] saturation
 * along desaturation direction dir. The symmetric form (k_min+k_max) re-centres
 * the saturation, allowing the axis to move either way (used for thrust). */
static float desat_gain(const float *motor, const float *dir, uint8_t n,
                        float mn, float mx_) {
  float k_min = 0.0f, k_max = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    if (m_fabsf(dir[i]) < 1e-6f) continue;
    float k = 0.0f;
    if (motor[i] < mn)      k = (mn - motor[i]) / dir[i];
    else if (motor[i] > mx_) k = (mx_ - motor[i]) / dir[i];
    else continue;
    if (k < k_min) k_min = k;
    if (k > k_max) k_max = k;
  }
  return k_min + k_max;
}

void mixer_allocate(const mixer_t *mx, const float w[MIX_NW],
                    float *motor, float realized[MIX_NW]) {
  const uint8_t n = mx->n;
  const float lo = mx->idle_floor, hi = 1.0f;

  /* nominal allocation: motor = Bpinv · w */
  for (uint8_t i = 0; i < n; i++) {
    float s = 0.0f;
    for (int a = 0; a < MIX_NW; a++) s += mx->Bpinv[i][a] * w[a];
    motor[i] = s;
  }

  if (mx->airmode == MIXER_AIRMODE_DISABLED) {
    /* Scale the deviation from collective so the worst motor just fits, holding
     * the commanded collective. */
    float thr = w[MIX_THRUST];
    float mn = motor[0], mxv = motor[0];
    for (uint8_t i = 1; i < n; i++) {
      if (motor[i] < mn) mn = motor[i];
      if (motor[i] > mxv) mxv = motor[i];
    }
    float scale = 1.0f;
    if (mn < 0.0f)        { float k = thr / (thr - mn);        if (k < scale) scale = k; }
    if (mxv > 1.0f)       { float k = (1.0f - thr) / (mxv - thr); if (k < scale) scale = k; }
    if (scale < 1.0f)
      for (uint8_t i = 0; i < n; i++) motor[i] = thr + scale * (motor[i] - thr);
  } else {
    /* Sequential desaturation: shift collective thrust to make room for
     * roll/pitch, then desaturate yaw. Roll/pitch are never scaled. */
    float dthr[MIXER_MAX_MOTORS], dyaw[MIXER_MAX_MOTORS];
    for (uint8_t i = 0; i < n; i++) {
      dthr[i] = mx->Bpinv[i][MIX_THRUST];
      dyaw[i] = mx->Bpinv[i][MIX_YAW];
    }
    /* thrust: move either direction to centre the saturation */
    float kt = desat_gain(motor, dthr, n, lo, hi);
    for (uint8_t i = 0; i < n; i++) motor[i] += kt * dthr[i];

    if (mx->airmode == MIXER_AIRMODE_RP) {
      /* sacrifice yaw next: pull yaw toward 0 until motors fit */
      float ky = desat_gain(motor, dyaw, n, lo, hi);
      for (uint8_t i = 0; i < n; i++) motor[i] += ky * dyaw[i];
    }
    /* MIXER_AIRMODE_RPY keeps yaw and lets the final clamp take whatever is
     * left — roll/pitch already preserved by the thrust shift. */
  }

  /* clamp + idle floor + NaN guard */
  for (uint8_t i = 0; i < n; i++) {
    float v = motor[i];
    if (m_isnan(v)) v = lo;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    motor[i] = v;
  }

  /* realized wrench (same units as w): (B · motor) · fwd_scale */
  if (realized) {
    for (int a = 0; a < MIX_NW; a++) {
      float s = 0.0f;
      for (uint8_t i = 0; i < n; i++) s += mx->B[a][i] * motor[i];
      realized[a] = s * mx->fwd_scale;
    }
  }
}
