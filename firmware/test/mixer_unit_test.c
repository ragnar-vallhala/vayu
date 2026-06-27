/* mixer_unit_test.c — standalone host test for the control-allocation mixer.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude test/mixer_unit_test.c src/control/mixer.c -lm -o /tmp/mt && /tmp/mt
 *
 * Proves: (1) the symmetric quad-X allocation has unit per-axis gain
 * (out_i = thr + roll*sign + pitch*sign + yaw*spin); (2) under saturation,
 * airmode PRESERVES roll/pitch torque where the uniform scaler DISCARDS it;
 * (3) the idle floor holds.
 */
#include "control/mixer.h"
#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}
static int near(float a, float b) { return fabsf(a - b) < 1e-4f; }

/* Default quad-X geometry (angle_rate_controller.c):
 *   s_mix_roll  = -sign(y) = {-1,-1,+1,+1}  -> pos_y = {+1,+1,-1,-1}
 *   s_mix_pitch = +sign(x) = {+1,-1,-1,+1}  -> pos_x = {+1,-1,-1,+1}
 *   s_mix_yaw   =  spin    = {-1,+1,-1,+1}
 */
static const float PX[4] = { +1, -1, -1, +1 };
static const float PY[4] = { +1, +1, -1, -1 };
static const int   SP[4] = { -1, +1, -1, +1 };
static const float SR[4] = { -1, -1, +1, +1 };
static const float SPi[4]= { +1, -1, -1, +1 };
static const float SY[4] = { -1, +1, -1, +1 };

/* reference mix: out_i = thr + roll*sr + pitch*sp + yaw*sy */
static void reference_mix(float thr, float roll, float pitch, float yaw, float out[4]) {
  for (int i = 0; i < 4; i++)
    out[i] = thr + roll * SR[i] + pitch * SPi[i] + yaw * SY[i];
}

int main(void) {
  mixer_t mx;
  if (!mixer_set_geometry(&mx, PX, PY, SP, 4)) { printf("geometry FAILED\n"); return 1; }

  /* (1) Bpinv is the sign matrix exactly (thrust col = 1, torque cols = the ±1
   * signs) -> unit per-axis gain. */
  printf("Test 1: pseudo-inverse == ±1 mix matrix (unit gain)\n");
  int ok = 1;
  for (int i = 0; i < 4; i++) {
    ok &= near(mx.Bpinv[i][MIX_ROLL],   SR[i]);
    ok &= near(mx.Bpinv[i][MIX_PITCH],  SPi[i]);
    ok &= near(mx.Bpinv[i][MIX_YAW],    SY[i]);
    ok &= near(mx.Bpinv[i][MIX_THRUST], 1.0f);
  }
  check("Bpinv equals the ±1 mix matrix", ok);

  /* (2) unsaturated allocation matches the reference mix value-for-value */
  printf("Test 2: unsaturated allocation == reference mix\n");
  mixer_set_airmode(&mx, MIXER_AIRMODE_RP);
  mixer_set_idle_floor(&mx, 0.0f);
  {
    float w[MIX_NW] = { 0.1f, -0.05f, 0.02f, 0.5f }; /* roll,pitch,yaw,thr */
    float m[4], ref[4];
    mixer_allocate(&mx, w, m, NULL);
    reference_mix(w[MIX_THRUST], w[MIX_ROLL], w[MIX_PITCH], w[MIX_YAW], ref);
    int eq = 1;
    for (int i = 0; i < 4; i++) eq &= near(m[i], ref[i]);
    check("matches out_i = thr + roll*sr + pitch*sp + yaw*sy", eq);
  }

  /* (3) under saturation, airmode preserves roll/pitch; the uniform scaler does
   * not. Demand a large roll at low thrust so two motors would go negative. */
  printf("Test 3: saturation — airmode preserves roll, uniform scaler discards it\n");
  {
    float w[MIX_NW] = { 0.30f, 0.0f, 0.0f, 0.20f };
    float m_rp[4], r_rp[MIX_NW], m_off[4], r_off[MIX_NW];

    mixer_set_airmode(&mx, MIXER_AIRMODE_RP);
    mixer_allocate(&mx, w, m_rp, r_rp);

    mixer_set_airmode(&mx, MIXER_AIRMODE_DISABLED);
    mixer_allocate(&mx, w, m_off, r_off);

    printf("    demanded roll = %.3f\n", w[MIX_ROLL]);
    printf("    airmode RP  : realized roll = %.3f  thrust = %.3f  motors {%.2f %.2f %.2f %.2f}\n",
           r_rp[MIX_ROLL], r_rp[MIX_THRUST], m_rp[0], m_rp[1], m_rp[2], m_rp[3]);
    printf("    airmode off : realized roll = %.3f  thrust = %.3f  motors {%.2f %.2f %.2f %.2f}\n",
           r_off[MIX_ROLL], r_off[MIX_THRUST], m_off[0], m_off[1], m_off[2], m_off[3]);

    check("airmode RP delivers the full demanded roll", near(r_rp[MIX_ROLL], w[MIX_ROLL]));
    check("uniform scaler LOSES roll authority (< demanded)", r_off[MIX_ROLL] < w[MIX_ROLL] - 1e-3f);
    check("airmode raised collective to make room",     r_rp[MIX_THRUST] > w[MIX_THRUST] + 1e-3f);
    check("all motors in [0,1]",
          m_rp[0] >= -1e-6f && m_rp[3] <= 1.0f + 1e-6f);
  }

  /* (4) idle floor holds */
  printf("Test 4: idle floor\n");
  {
    mixer_set_airmode(&mx, MIXER_AIRMODE_RP);
    mixer_set_idle_floor(&mx, 0.06f);
    float w[MIX_NW] = { 0.0f, 0.0f, 0.0f, 0.0f }; /* zero thrust */
    float m[4];
    mixer_allocate(&mx, w, m, NULL);
    int floored = 1;
    for (int i = 0; i < 4; i++) floored &= near(m[i], 0.06f);
    check("every motor held at idle floor", floored);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
