/**
 * @file ekf.c
 * @brief Attitude error-state (multiplicative) EKF — MEKF.
 *
 * A covariance-based alternative to the Mahony/complementary filters,
 * selectable via SF_FILTER_USED (SF_EKF / SF_EKF_ACCEL_BIAS). It estimates
 * attitude together with the gyro bias (and, in the 9-state variant, the
 * accelerometer bias) instead of approximating the bias with a fixed integral
 * gain. Same I/O contract as m_mahony_filter(): gyro/accel/mag + dt in, an
 * attitude_t (quaternion + Euler) out, so it is a drop-in for attitude_task.c
 * and feeds the existing control/telemetry queues unchanged.
 *
 * State (error-state, injected into a nominal quaternion each update):
 *   x = [ dtheta(3) | gyro_bias(3) | accel_bias(3, 9-state only) ]
 * Nominal: unit quaternion q (body->world), gyro bias bg, accel bias ba.
 *
 * Measurements (decoupled, matching the Mahony philosophy):
 *   - accel: gravity DIRECTION pins roll/pitch (3-vector update; gated on
 *     |a| ~ g so dynamic acceleration is rejected). 9-state observes accel
 *     bias through the unnormalised specific-force model.
 *   - mag:   tilt-compensated heading pins yaw only (scalar update about the
 *     world vertical), so mag noise/bias can never disturb the tilt estimate.
 *
 * The matrix and quaternion math is the shared header-only kernel in
 * maths/linalg.h (reused by other estimators/controllers), not local statics.
 *
 * @implements EST-EKF-001, EST-EKF-002
 */
#include "est/ekf.h"
#include "est/est.h"
#include "maths/linalg.h"
#include "maths/maths_interface.h"
#include <stdbool.h>

#define EKF_NMAX 9 /* max error-state dimension (9-state variant) */
#define EKF_MMAX 3 /* max measurement dimension (accel vector) */

/* Single estimator instance (single-writer: the attitude task), mirroring the
 * module-level state of the Mahony filter. */
typedef struct {
  bool initialized;
  bool est_accel_bias; /* true => 9-state */
  int n;               /* 6 or 9 */
  quaternion_t q;      /* nominal attitude (body->world) */
  float bg[3];         /* nominal gyro bias (rad/s) */
  float ba[3];         /* nominal accel bias (m/s^2), 9-state only */
  float P[EKF_NMAX * EKF_NMAX];
} ekf_t;

static ekf_t E;

/* --------------------------------------------------------------------------
 * Init / reset
 *
 * @implements EST-EKF-101
 * ------------------------------------------------------------------------*/

void ekf_init(bool estimate_accel_bias) {
  E.est_accel_bias = estimate_accel_bias;
  E.n = estimate_accel_bias ? 9 : 6;
  E.q.w = 1.0f;
  E.q.x = 0.0f;
  E.q.y = 0.0f;
  E.q.z = 0.0f;
  for (int i = 0; i < 3; i++) {
    E.bg[i] = 0.0f;
    E.ba[i] = 0.0f;
  }
  for (int i = 0; i < EKF_NMAX * EKF_NMAX; i++)
    E.P[i] = 0.0f;
  int n = E.n;
  for (int i = 0; i < 3; i++)
    E.P[i * n + i] = EKF_P0_ATT;
  for (int i = 3; i < 6; i++)
    E.P[i * n + i] = EKF_P0_BG;
  if (E.est_accel_bias)
    for (int i = 6; i < 9; i++)
      E.P[i * n + i] = EKF_P0_BA;
  E.initialized = true;
}

/* ekf_reset(): zero bias + covariance, preserving the configured 6/9 dimension.
 * @implements EST-EKF-101 */
void ekf_reset(void) { ekf_init(E.initialized ? E.est_accel_bias : false); }

/* @noreq trivial accessor (estimated gyro bias). */
void ekf_get_gyro_bias(float out[3]) {
  out[0] = E.bg[0];
  out[1] = E.bg[1];
  out[2] = E.bg[2];
}

/* @noreq trivial accessor (estimated accel bias). */
void ekf_get_accel_bias(float out[3]) {
  out[0] = E.ba[0];
  out[1] = E.ba[1];
  out[2] = E.ba[2];
}

/* --------------------------------------------------------------------------
 * Error-state injection: fold dx into the nominal state, reset error to 0.
 *
 * @noreq internal MEKF error-state injection helper.
 * ------------------------------------------------------------------------*/
static void ekf_inject(const float *dx) {
  float dtheta[3] = {dx[0], dx[1], dx[2]};
  quaternion_t dq;
  m_quat_exp(dtheta, &dq);
  quaternion_t qn;
  m_quat_mul(&E.q, &dq, &qn);
  E.q = qn;
  m_quat_normalize(&E.q);
  E.bg[0] += dx[3];
  E.bg[1] += dx[4];
  E.bg[2] += dx[5];
  if (E.est_accel_bias) {
    E.ba[0] += dx[6];
    E.ba[1] += dx[7];
    E.ba[2] += dx[8];
  }
}

/* --------------------------------------------------------------------------
 * Generic Kalman correction with Joseph-form covariance update.
 *   H: m x n, y: m (innovation), Rdiag: m (diagonal measurement noise).
 * Supports m == 1 (scalar) and m == 3 (vector).
 *
 * The matrix scratch is `static`, not stack-allocated: a 9x9 Joseph update is
 * ~1.8 KB of temporaries, which overflows the attitude task's stack. The EKF
 * is a singleton driven only by the attitude task (like the static state `E`),
 * so static scratch is safe and keeps the call frame tiny. NOT reentrant.
 *
 * @implements EST-EKF-106
 * ------------------------------------------------------------------------*/
static void ekf_correct(const float *H, int m, const float *y,
                        const float *Rdiag) {
  int n = E.n;
  static float Ht[EKF_NMAX * EKF_MMAX];
  m_mat_transpose(H, m, n, Ht); /* n x m */
  static float PHt[EKF_NMAX * EKF_MMAX];
  m_mat_mul(E.P, n, n, Ht, m, PHt); /* n x m */
  static float S[EKF_MMAX * EKF_MMAX];
  m_mat_mul(H, m, n, PHt, m, S); /* m x m */
  for (int i = 0; i < m; i++)
    S[i * m + i] += Rdiag[i];

  static float Sinv[EKF_MMAX * EKF_MMAX];
  if (m == 1) {
    /* S = H P H^T + R, and H P H^T >= 0 for a positive-definite P, so a healthy
     * innovation variance is always >= R. If it has collapsed below a fraction
     * of R (or gone negative) P is no longer trustworthy: skip rather than
     * divide by it. Relative to R, not an absolute 1e-20 that never trips. */
    if (S[0] < 0.5f * Rdiag[0])
      return;
    Sinv[0] = 1.0f / S[0];
  } else {
    if (!m_mat3_inv(S, Sinv))
      return;
  }

  static float K[EKF_NMAX * EKF_MMAX];
  m_mat_mul(PHt, n, m, Sinv, m, K); /* n x m */

  static float dx[EKF_NMAX];
  for (int i = 0; i < n; i++) {
    float s = 0.0f;
    for (int j = 0; j < m; j++)
      s += K[i * m + j] * y[j];
    dx[i] = s;
  }
  ekf_inject(dx);

  /* Joseph form: P = (I - K H) P (I - K H)^T + K R K^T. */
  static float KH[EKF_NMAX * EKF_NMAX];
  m_mat_mul(K, n, m, H, n, KH); /* n x n */
  static float IKH[EKF_NMAX * EKF_NMAX];
  m_mat_identity(IKH, n);
  for (int i = 0; i < n * n; i++)
    IKH[i] -= KH[i];
  static float tmp[EKF_NMAX * EKF_NMAX];
  m_mat_mul(IKH, n, n, E.P, n, tmp); /* (I-KH) P */
  static float Pnew[EKF_NMAX * EKF_NMAX];
  m_mat_mul_abt(tmp, n, n, IKH, n, Pnew); /* ... (I-KH)^T */
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      float s = 0.0f;
      for (int k = 0; k < m; k++)
        s += K[i * m + k] * Rdiag[k] * K[j * m + k];
      Pnew[i * n + j] += s;
    }
  }
  m_mat_copy(E.P, Pnew, n * n);
  /* Joseph form is symmetric in exact arithmetic; repair float32 asymmetry so
   * it cannot compound through the bias-feedback loop into the predict step. */
  m_mat_symmetrize(E.P, n);
}

/* --------------------------------------------------------------------------
 * Predict: propagate nominal quaternion and covariance over dt.
 *   gx,gy,gz in deg/s (matching the converted IMU reading).
 *
 * @implements EST-EKF-103
 * ------------------------------------------------------------------------*/
static void ekf_predict(float gx, float gy, float gz, float dt) {
  int n = E.n;
  float w[3] = {to_radians(gx) - E.bg[0], to_radians(gy) - E.bg[1],
                to_radians(gz) - E.bg[2]};

  /* Nominal quaternion via exponential map of the bias-corrected rate. */
  float dtheta[3] = {w[0] * dt, w[1] * dt, w[2] * dt};
  quaternion_t dq;
  m_quat_exp(dtheta, &dq);
  quaternion_t qn;
  m_quat_mul(&E.q, &dq, &qn);
  E.q = qn;
  m_quat_normalize(&E.q);

  /* Discrete error-state transition Phi (n x n):
   *   dtheta_dot = -[w]x dtheta - dbg   => Phi_tt = I - [w]x dt, Phi_tb = -I dt
   *   biases are random walks            => identity blocks.
   * Scratch is static (see ekf_correct): keeps this off the task stack. */
  static float Phi[EKF_NMAX * EKF_NMAX];
  m_mat_identity(Phi, n);
  float Sw[9];
  m_skew3(w, Sw);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Phi[i * n + j] -= Sw[i * 3 + j] * dt;
  for (int i = 0; i < 3; i++)
    Phi[i * n + (3 + i)] -= dt; /* attitude <- gyro-bias coupling */

  /* P = Phi P Phi^T + Q. */
  static float PhiP[EKF_NMAX * EKF_NMAX];
  m_mat_mul(Phi, n, n, E.P, n, PhiP);
  static float Pnew[EKF_NMAX * EKF_NMAX];
  m_mat_mul_abt(PhiP, n, n, Phi, n, Pnew);

  float qatt = EKF_Q_ATT * dt;
  float qbg = EKF_Q_BG * dt;
  float qba = EKF_Q_BA * dt;
  for (int i = 0; i < 3; i++)
    Pnew[i * n + i] += qatt;
  for (int i = 3; i < 6; i++)
    Pnew[i * n + i] += qbg;
  if (E.est_accel_bias)
    for (int i = 6; i < 9; i++)
      Pnew[i * n + i] += qba;

  m_mat_copy(E.P, Pnew, n * n);
  /* Phi P Phi^T is symmetric in exact arithmetic; repair float32 asymmetry. */
  m_mat_symmetrize(E.P, n);
}

/* --------------------------------------------------------------------------
 * Accel update: gravity direction pins roll/pitch (and accel bias, 9-state).
 *
 * @implements EST-EKF-102, EST-EKF-104, EST-EKF-105
 * ------------------------------------------------------------------------*/
static void ekf_update_accel(float ax, float ay, float az) {
  float amag = m_sqrt(ax * ax + ay * ay + az * az);
  if (amag < 1e-3f)
    return;
  /* Accel-trust gate: only fuse when |a| ~ g (else the body is accelerating
   * and the accel is not a clean gravity reference). */
  if (m_fabsf(amag - EKF_GRAVITY) > EKF_ACC_GATE)
    return;

  int n = E.n;
  /* This IMU/driver convention reports the GRAVITY vector (points down): a
   * level, upright board reads ~ -g on its vertical axis — matching the Mahony
   * filter's gravity estimate. So the model predicts the world-DOWN direction
   * in body, gb = R^T (0,0,-1); a level board then has zero accel innovation
   * (using world-up here flips the estimate 180°). */
  const float g_w[3] = {0.0f, 0.0f, -1.0f};
  float gb[3];
  m_quat_rotate_inv(&E.q, g_w, gb); /* gravity (down) expressed in body */
  float Sgb[9];
  m_skew3(gb, Sgb);

  static float H[EKF_MMAX * EKF_NMAX];
  for (int i = 0; i < 3 * n; i++)
    H[i] = 0.0f;
  float y[3];

  if (!E.est_accel_bias) {
    /* 6-state: measure the unit gravity direction; h = gb, H_theta = [gb]x. */
    float z[3] = {ax / amag, ay / amag, az / amag};
    y[0] = z[0] - gb[0];
    y[1] = z[1] - gb[1];
    y[2] = z[2] - gb[2];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        H[i * n + j] = Sgb[i * 3 + j];
    float R[3] = {EKF_R_ACC_DIR, EKF_R_ACC_DIR, EKF_R_ACC_DIR};
    ekf_correct(H, 3, y, R);
  } else {
    /* 9-state: specific-force model h = g*gb + ba; observes accel bias.
     * H_theta = g*[gb]x, H_ba = I. */
    y[0] = ax - (EKF_GRAVITY * gb[0] + E.ba[0]);
    y[1] = ay - (EKF_GRAVITY * gb[1] + E.ba[1]);
    y[2] = az - (EKF_GRAVITY * gb[2] + E.ba[2]);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        H[i * n + j] = EKF_GRAVITY * Sgb[i * 3 + j];
    for (int i = 0; i < 3; i++)
      H[i * n + (6 + i)] = 1.0f;
    float R[3] = {EKF_R_ACC, EKF_R_ACC, EKF_R_ACC};
    ekf_correct(H, 3, y, R);
  }
}

/* --------------------------------------------------------------------------
 * Mag update: tilt-compensated heading pins yaw only (scalar update about the
 * world vertical). Declination assumed 0 (field points magnetic north).
 *
 * @implements EST-EKF-105
 * ------------------------------------------------------------------------*/
static void ekf_update_mag(float mx, float my, float mz) {
  float mn = m_sqrt(mx * mx + my * my + mz * mz);
  if (mn < 1e-6f)
    return;
  int n = E.n;
  float mb[3] = {mx / mn, my / mn, mz / mn};
  float mw[3];
  m_quat_rotate(&E.q, mb, mw); /* mag in world frame */
  float heading_err = m_atan2(mw[1], mw[0]);

  /* H maps the body-frame attitude error onto heading: a heading error lives
   * about the world vertical, i.e. the world-down axis expressed in body
   * (v_down_b = -ub). Innovation is the heading error itself. */
  const float up_w[3] = {0.0f, 0.0f, 1.0f};
  float ub[3];
  m_quat_rotate_inv(&E.q, up_w, ub);
  static float H[EKF_NMAX];
  for (int i = 0; i < n; i++)
    H[i] = 0.0f;
  H[0] = -ub[0];
  H[1] = -ub[1];
  H[2] = -ub[2];
  float y[1] = {heading_err};
  float R[1] = {EKF_R_MAG_YAW};
  ekf_correct(H, 1, y, R);
}

/* --------------------------------------------------------------------------
 * Public filter entry — same signature as m_mahony_filter().
 *
 * @implements EST-EKF-001, EST-EKF-002
 * ------------------------------------------------------------------------*/
void m_ekf_filter(const float ax, const float ay, const float az, const float gx,
                  const float gy, const float gz, const float mx, const float my,
                  const float mz, float dt, attitude_t *ori) {
  if (!E.initialized)
    ekf_init(false);

  /* Start from identity (like Mahony) and let the accel/mag updates level and
   * align it; with P0_att the tilt converges in well under a second. */
  ekf_predict(gx, gy, gz, dt);
  ekf_update_accel(ax, ay, az);
  ekf_update_mag(mx, my, mz);

  ori->q = E.q;
  m_quat_to_euler_deg(&E.q, &ori->roll, &ori->pitch, &ori->yaw);
}
