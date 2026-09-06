/**
 * @file sim/host/tests/test_vertical_est.c
 * @brief Host verification suite for the 2-state vertical estimator (VERT).
 *
 * Exercises the pure core in src/est/vertical_estimator.c (linked via
 * vayu_sitl_core) with synthetic accel + baro so each property is checked
 * against a known ground truth:
 *
 *   VERT-001  gravity removal: a stationary craft reads a_up ~ 0 at any attitude
 *   VERT-002  seed: not initialised until first baro; then alt=baro, climb=0
 *   VERT-003  static hold: stays at altitude, climb ~ 0
 *   VERT-004  step response: converges to a new baro level
 *   VERT-005  climb tracking: tracks a ramp; climb_rate clean under baro noise
 *   VERT-006  accel-bias: the third state learns a constant bias, so BOTH
 *             altitude and climb_rate converge to truth (this is the
 *             vibration fix — the 2-state filter used to leave a
 *             (k_alt/k_vel)*b velocity offset here, which on hardware was a
 *             phantom -1.5 m/s descent)
 *   VERT-007  reset: returns to the un-seeded state
 *   VERT-008  bias clamp + accel_unhealthy latch/probation
 *   VERT-009  rangefinder bias aiding: converges the bias far faster than baro
 *             alone (the takeoff window), touches nothing but the bias, and
 *             rejects terrain steps / real motion
 *
 * Companion to test_phase3_est_ekf.c (attitude EKF). Run via ctest or directly.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "est/vertical_estimator.h"
#include "maths/linalg.h"

static int g_checks = 0;
static int g_fails = 0;

static void check(bool pass, const char *name) {
  g_checks++;
  if (pass) {
    printf("    ok   %s\n", name);
  } else {
    g_fails++;
    printf("    FAIL %s\n", name);
  }
}

/* Deterministic pseudo-noise in [-1, 1] (LCG) so the suite is reproducible. */
static uint32_t g_rng = 0x12345678u;
static float noise(void) {
  g_rng = g_rng * 1664525u + 1013904223u;
  return ((float)(g_rng >> 8) / (float)(1u << 24)) * 2.0f - 1.0f;
}

/* Body specific force a *stationary* craft at attitude q reports: the world
 * gravity reading (0,0,-g) expressed in the body frame. */
static void stationary_body_accel(const quaternion_t *q, float a_body[3]) {
  const float world_static[3] = {0.0f, 0.0f, -VERT_GRAVITY};
  m_quat_rotate_inv(q, world_static, a_body);
}

/* --- VERT-001: gravity removal at several attitudes --------------------- */
static void test_gravity_removal(void) {
  const float att[][3] = {
      {0, 0, 0}, {30, 0, 0}, {0, -25, 0}, {15, 40, 90}, {-60, 20, -120},
  };
  bool all_small = true;
  float worst = 0.0f;
  for (unsigned i = 0; i < sizeof(att) / sizeof(att[0]); i++) {
    quaternion_t q;
    m_quat_from_euler_deg(att[i][0], att[i][1], att[i][2], &q);
    float a_body[3];
    stationary_body_accel(&q, a_body);
    float a_up = vert_world_up_accel(&q, a_body);
    if (fabsf(a_up) > worst)
      worst = fabsf(a_up);
    if (fabsf(a_up) > 1e-3f)
      all_small = false;
  }
  check(all_small, "VERT-001 stationary a_up ~ 0 at any attitude");
  printf("         (worst |a_up| = %.2e m/s^2)\n", (double)worst);

  /* A genuine 1 m/s^2 upward push (level) must read +1, not be cancelled. */
  quaternion_t lvl;
  m_quat_from_euler_deg(0, 0, 0, &lvl);
  float push[3] = {0.0f, 0.0f, -(VERT_GRAVITY + 1.0f)}; /* extra thrust up */
  float a_up = vert_world_up_accel(&lvl, push);
  check(fabsf(a_up - 1.0f) < 1e-4f, "VERT-001 +1 m/s^2 push reads +1 up");
}

/* --- VERT-002 / VERT-003: seed + static hold --------------------------- */
static void test_seed_and_static(void) {
  vertical_estimator_t ve;
  vert_est_init_default(&ve);

  /* Predict before any baro must not move altitude (no origin yet). */
  vert_est_predict(&ve, 0.0f, 0.004f);
  check(!ve.initialized && ve.altitude == 0.0f,
        "VERT-002 un-seeded predict is a no-op");

  vert_est_correct(&ve, 100.0f);
  check(ve.initialized && fabsf(ve.altitude - 100.0f) < 1e-6f &&
            ve.climb_rate == 0.0f,
        "VERT-002 first baro seeds alt=baro, climb=0");

  /* Static: a_up=0, baro constant at 100, predict@250Hz, correct@~16Hz. */
  const float dt = 1.0f / 250.0f;
  for (int i = 0; i < 250 * 5; i++) {
    vert_est_predict(&ve, 0.0f, dt);
    if (i % 16 == 0)
      vert_est_correct(&ve, 100.0f);
  }
  check(fabsf(ve.altitude - 100.0f) < 1e-2f && fabsf(ve.climb_rate) < 1e-2f,
        "VERT-003 static: holds altitude, climb ~ 0");
}

/* --- VERT-004: step response to a new baro level ----------------------- */
static void test_step_response(void) {
  vertical_estimator_t ve;
  vert_est_init_default(&ve);
  vert_est_correct(&ve, 100.0f);

  const float dt = 1.0f / 250.0f;
  const float target = 105.0f;
  /* A 5 m INSTANTANEOUS baro step is not physical (the craft cannot teleport);
   * it is here as a stress case. The third state rings slightly more on it than
   * the old 2-state filter did, because the bias state briefly mistakes the
   * step for a real acceleration and then has to unwind. That cost is bounded
   * and measured below rather than assumed away: peak excursion, and a settled
   * state. 12 s covers the unwind (the 2-state filter was clean by 6). */
  float peak_climb = 0.0f;
  for (int i = 0; i < 250 * 12; i++) {
    vert_est_predict(&ve, 0.0f, dt);
    if (i % 16 == 0)
      vert_est_correct(&ve, target);
    if (i > 250 && fabsf(ve.climb_rate) > peak_climb)
      peak_climb = fabsf(ve.climb_rate);
  }
  check(fabsf(ve.altitude - target) < 0.2f,
        "VERT-004 converges to stepped baro level");
  check(fabsf(ve.climb_rate) < 0.2f, "VERT-004 climb settles to ~0 after step");
  check(peak_climb < 3.5f,
        "VERT-004 bias state does not make the step transient blow up");
  printf("         (peak |climb| during the 5 m step: %.2f m/s)\n",
         (double)peak_climb);
}

/* --- VERT-005: climb tracking with noisy baro -------------------------- */
static void test_climb_tracking(void) {
  vertical_estimator_t ve;
  vert_est_init_default(&ve);

  const float dt = 1.0f / 250.0f;
  const int baro_decim = 16; /* ~16 Hz baro */
  /* True profile: rest -> accelerate up 1 m/s^2 for 2 s (to +2 m/s) ->
   * cruise 2 m/s for 3 s -> decelerate 1 m/s^2 for 2 s -> rest. */
  float true_alt = 50.0f, true_v = 0.0f;
  float worst_alt_err = 0.0f, worst_v_err = 0.0f;
  bool seeded = false;

  for (int i = 0; i < 250 * 8; i++) {
    float t = (float)i * dt;
    float a = 0.0f;
    if (t < 2.0f)
      a = 1.0f;
    else if (t < 5.0f)
      a = 0.0f;
    else if (t < 7.0f)
      a = -1.0f;
    /* advance truth */
    true_alt += true_v * dt + 0.5f * a * dt * dt;
    true_v += a * dt;

    vert_est_predict(&ve, a, dt); /* accel feed is exact (no bias) */
    if (i % baro_decim == 0) {
      float baro = true_alt + 0.7f * noise(); /* +/-0.7 m baro noise */
      vert_est_correct(&ve, baro);
      if (!seeded) {
        seeded = true;
        continue; /* skip the seed transient */
      }
    }
    if (seeded && t > 0.3f) { /* allow a brief settle */
      float ae = fabsf(ve.altitude - true_alt);
      float ve_err = fabsf(ve.climb_rate - true_v);
      if (ae > worst_alt_err)
        worst_alt_err = ae;
      if (ve_err > worst_v_err)
        worst_v_err = ve_err;
    }
  }
  check(worst_alt_err < 0.6f, "VERT-005 fused altitude tracks truth (<0.6 m)");
  check(worst_v_err < 0.4f, "VERT-005 climb_rate tracks truth (<0.4 m/s)");
  printf("         (worst alt err %.3f m, worst climb err %.3f m/s)\n",
         (double)worst_alt_err, (double)worst_v_err);

  /* Climb estimate must be far cleaner than naive baro differentiation. */
  float diff_v_rms = 0.0f;
  int n = 0;
  float prev_baro = 50.0f;
  g_rng = 0x12345678u; /* replay same noise stream */
  for (int k = 0; k < 200; k++) {
    float baro = 50.0f + 0.7f * noise(); /* static truth, pure noise */
    float dvdt = (baro - prev_baro) / (1.0f / 16.0f);
    prev_baro = baro;
    diff_v_rms += dvdt * dvdt;
    n++;
  }
  diff_v_rms = sqrtf(diff_v_rms / (float)n);
  check(diff_v_rms > 5.0f, "VERT-005 naive baro-diff velocity is noisy (control)");
  printf("         (naive baro-diff climb RMS %.2f m/s vs fused <0.4)\n",
         (double)diff_v_rms);
}

/* --- VERT-006: accel-bias — the third state cancels it --------------------- */
static void test_accel_bias(void) {
  /* Drive predict and correct at the SAME rate so the steady state is clean.
   * b is NEGATIVE and ~1 m/s^2: that is the sign and size actually measured on
   * the airframe (accel under-reports gravity once the motors run), and the
   * old 2-state filter turned it into climb_rate ~ -1.5 m/s. */
  vertical_estimator_t ve;
  vert_est_init_default(&ve);
  const float H = 100.0f;
  vert_est_correct(&ve, H);

  const float dt = 1.0f / 16.0f; /* one predict per correct */
  const float b = -1.0f;         /* constant accel bias, m/s^2 */
  for (int i = 0; i < 16 * 30; i++) { /* 30 s to steady state */
    vert_est_predict(&ve, b, dt);
    vert_est_correct(&ve, H);
  }
  /* What the 2-state filter would have produced, for the record. */
  float old_offset = b * (ve.k_alt / ve.k_vel - 0.5f * dt);
  check(fabsf(ve.altitude - H) < 0.05f,
        "VERT-006 altitude converges to truth under a constant accel bias");
  check(fabsf(ve.climb_rate) < 0.05f,
        "VERT-006 climb_rate converges to ZERO (was the (k_alt/k_vel)*b offset)");
  check(fabsf(ve.accel_bias - b) < 0.05f,
        "VERT-006 bias state learns the true bias");
  check(!ve.accel_unhealthy,
        "VERT-006 a bias within authority is absorbed, not flagged");
  printf("         (alt %.3f m, climb %.4f m/s, bias %.4f; 2-state would have "
         "given climb %.4f m/s)\n",
         (double)ve.altitude, (double)ve.climb_rate, (double)ve.accel_bias,
         (double)old_offset);

  /* It must converge in a usable time, not eventually: a fix that takes a
   * minute to settle is no use on a 20 s bench hop. Poles at w ~ 0.8 rad/s
   * predict ~3/w ~ 4 s; allow 8. */
  vertical_estimator_t vq;
  vert_est_init_default(&vq);
  vert_est_correct(&vq, H);
  int settle = -1;
  for (int i = 0; i < 16 * 20; i++) {
    vert_est_predict(&vq, b, dt);
    vert_est_correct(&vq, H);
    if (settle < 0 && fabsf(vq.climb_rate) < 0.10f && i > 16)
      settle = i;
  }
  check(settle >= 0 && settle < 16 * 8,
        "VERT-006 climb_rate settles within 8 s of the bias appearing");
  printf("         (settled after %.2f s)\n", settle / 16.0);

  /* And it must not eat REAL motion: a steady climb has zero net accel, so the
   * bias state has nothing to feed on. This is the regression that would make
   * the fix worse than the bug. */
  vertical_estimator_t vr;
  vert_est_init_default(&vr);
  float truth = H;
  const float vz = 1.0f; /* 1 m/s steady climb, no bias */
  vert_est_correct(&vr, truth);
  for (int i = 0; i < 16 * 20; i++) {
    truth += vz * dt;
    vert_est_predict(&vr, 0.0f, dt); /* constant velocity => zero accel */
    vert_est_correct(&vr, truth);
  }
  check(fabsf(vr.climb_rate - vz) < 0.10f,
        "VERT-006 a real steady climb is still tracked (bias stays ~0)");
  check(fabsf(vr.accel_bias) < 0.10f,
        "VERT-006 bias state does not absorb real motion");
}

/* --- VERT-007: reset --------------------------------------------------- */
static void test_reset(void) {
  vertical_estimator_t ve;
  vert_est_init_default(&ve);
  vert_est_correct(&ve, 42.0f);
  vert_est_predict(&ve, 1.0f, 0.1f);
  vert_est_reset(&ve);
  check(!ve.initialized && ve.altitude == 0.0f && ve.climb_rate == 0.0f,
        "VERT-007 reset clears state, keeps un-seeded");
  /* gains preserved */
  check(ve.k_alt == VERT_DEFAULT_K_ALT && ve.k_vel == VERT_DEFAULT_K_VEL &&
            ve.k_bias == VERT_DEFAULT_K_BIAS,
        "VERT-007 reset preserves gains");
  check(ve.accel_bias == 0.0f && !ve.accel_unhealthy,
        "VERT-007 reset clears the bias state and health flag");
}

/* --- VERT-008: bias clamp + unhealthy latch ----------------------------- */
static void test_accel_unhealthy(void) {
  /* A bias far beyond the clamp: the state saturates, the flag latches, and
   * the estimate must NOT pretend to be fine. */
  vertical_estimator_t ve;
  vert_est_init_default(&ve);
  const float H = 50.0f;
  vert_est_correct(&ve, H);
  const float dt = 1.0f / 16.0f;
  for (int i = 0; i < 16 * 30; i++) {
    vert_est_predict(&ve, -8.0f, dt); /* way past VERT_ACCEL_BIAS_MAX */
    vert_est_correct(&ve, H);
  }
  check(ve.accel_bias >= -VERT_ACCEL_BIAS_MAX - 1e-4f,
        "VERT-008 bias state is clamped to its authority limit");
  check(ve.accel_unhealthy, "VERT-008 saturation raises accel_unhealthy");

  /* Probation: the flag must survive a few clean samples, then clear. */
  for (int i = 0; i < 16; i++) {
    vert_est_predict(&ve, 0.0f, dt);
    vert_est_correct(&ve, H);
  }
  check(ve.accel_unhealthy, "VERT-008 flag persists through brief recovery");
  for (int i = 0; i < 16 * 20; i++) {
    vert_est_predict(&ve, 0.0f, dt);
    vert_est_correct(&ve, H);
  }
  check(!ve.accel_unhealthy, "VERT-008 flag clears after sustained recovery");
}

/* --- VERT-009: rangefinder bias aiding ---------------------------------- */

/* Craft on the ground under a rangefinder reading `agl`; at t=0 the motors
 * spool and the accel starts under-reading by `b`. Returns seconds until
 * |climb_rate| is back under 0.15, or -1. */
static float tof_settle(bool use_tof, float b, float agl) {
  const float dt = 1.0f / 250.0f;
  const float H = 100.0f;
  vertical_estimator_t ve;
  vert_est_init_default(&ve);
  vert_est_correct(&ve, H);
  for (int i = 0; i < 250 * 3; i++) { /* settle before the step */
    vert_est_predict(&ve, 0.0f, dt);
    if (i % 16 == 0)
      vert_est_correct(&ve, H);
  }
  float acc = 0.0f;
  for (int i = 0; i < 250 * 16; i++) {
    vert_est_predict(&ve, b, dt);
    if (i % 16 == 0)
      vert_est_correct(&ve, H + 0.3f * noise());
    acc += dt;
    if (use_tof && i % 12 == 0) { /* ~21 Hz, 10 mm noise */
      vert_est_correct_tof(&ve, agl + 0.010f * noise(), acc);
      acc = 0.0f;
    }
    if (i > 250 && fabsf(ve.climb_rate) < 0.15f)
      return i / 250.0f;
  }
  return -1.0f;
}

static void test_tof_aiding(void) {
  const float b = -1.0f, agl = 0.10f;
  float t_baro = tof_settle(false, b, agl);
  float t_tof = tof_settle(true, b, agl);
  check(t_tof > 0.0f && t_baro > 0.0f && t_tof < 0.5f * t_baro,
        "VERT-009 ToF aiding at least halves time-to-converge");
  printf("         (baro only %.2f s -> with ToF %.2f s)\n", (double)t_baro,
         (double)t_tof);

  /* It must move the BIAS and NOTHING else — the whole reason it is safe to
   * feed a terrain-referenced sensor into a baro-referenced filter. */
  {
    vertical_estimator_t ve;
    vert_est_init_default(&ve);
    vert_est_correct(&ve, 100.0f);
    vert_est_correct_tof(&ve, 0.10f, 0.05f); /* seeds history */
    float alt0 = ve.altitude, climb0 = ve.climb_rate, bias0 = ve.accel_bias;
    vert_est_correct_tof(&ve, 0.20f, 0.05f); /* 2 m/s of apparent climb */
    check(ve.altitude == alt0 && ve.climb_rate == climb0,
          "VERT-009 aiding never moves altitude or climb_rate");
    check(ve.accel_bias != bias0, "VERT-009 aiding does move the bias");
  }

  /* A terrain step (flying over a table) must be rejected, not integrated. */
  {
    vertical_estimator_t ve;
    vert_est_init_default(&ve);
    vert_est_correct(&ve, 100.0f);
    const float dt = 1.0f / 250.0f;
    float acc = 0.0f, ground = 0.0f;
    for (int i = 0; i < 250 * 10; i++) {
      vert_est_predict(&ve, 0.0f, dt);
      if (i % 16 == 0)
        vert_est_correct(&ve, 100.0f);
      acc += dt;
      if (i % 12 == 0) {
        if (i == 250 * 5)
          ground = 0.40f; /* the ground steps up 40 cm, instantly */
        vert_est_correct_tof(&ve, 0.60f - ground, acc);
        acc = 0.0f;
      }
    }
    check(fabsf(ve.accel_bias) < 0.15f,
          "VERT-009 a terrain step does not corrupt the bias");
    check(fabsf(ve.climb_rate) < 0.15f,
          "VERT-009 a terrain step does not corrupt climb_rate");
    printf("         (after a 40 cm terrain step: bias %+.3f, climb %+.3f)\n",
           (double)ve.accel_bias, (double)ve.climb_rate);
  }

  /* And a REAL climb must still be corrected: the ToF velocity is truth here,
   * so the innovation goes to zero and the bias has nothing to feed on. This is
   * the regression that would make the aiding worse than the bug.
   *
   * The filter is first brought UP TO SPEED on baro alone — after a seed,
   * climb_rate starts at zero and a constant-velocity climb produces no accel
   * to integrate, so it takes the baro cross-term several seconds to build the
   * velocity. Aiding is then switched on for the stretch that is actually
   * inside the rangefinder's band, which is the real scenario. */
  {
    vertical_estimator_t ve;
    vert_est_init_default(&ve);
    const float dt = 1.0f / 250.0f, vz = 0.5f;
    float truth = 100.0f, acc = 0.0f;
    vert_est_correct(&ve, truth);
    for (int i = 0; i < 250 * 25; i++) { /* baro-only spin-up */
      truth += vz * dt;
      vert_est_predict(&ve, 0.0f, dt);
      if (i % 16 == 0)
        vert_est_correct(&ve, truth);
    }
    check(fabsf(ve.climb_rate - vz) < 0.10f,
          "VERT-009 precondition: filter is tracking the climb");
    float bias_before = ve.accel_bias;
    float agl_now = 0.20f; /* now inside the ToF band, still climbing */
    for (int i = 0; i < 250 * 2; i++) {
      truth += vz * dt;
      agl_now += vz * dt;
      vert_est_predict(&ve, 0.0f, dt);
      if (i % 16 == 0)
        vert_est_correct(&ve, truth);
      acc += dt;
      if (i % 12 == 0) {
        vert_est_correct_tof(&ve, agl_now + 0.010f * noise(), acc);
        acc = 0.0f;
      }
    }
    check(fabsf(ve.climb_rate - vz) < 0.10f,
          "VERT-009 a real climb survives aiding being switched on");
    check(fabsf(ve.accel_bias - bias_before) < 0.10f,
          "VERT-009 aiding does not absorb real motion");
    printf("         (real 0.50 m/s climb: filter %.3f m/s, bias drift %+.3f)\n",
           (double)ve.climb_rate, (double)(ve.accel_bias - bias_before));
  }
}

int main(void) {
  printf("== VERT vertical-estimator verification ==\n");
  test_gravity_removal();
  test_seed_and_static();
  test_step_response();
  test_climb_tracking();
  test_accel_bias();
  test_reset();
  test_accel_unhealthy();
  test_tof_aiding();
  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
