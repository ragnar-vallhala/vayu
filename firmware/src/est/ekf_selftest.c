/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file ekf_selftest.c
 * @brief Branch-coverage correctness scenarios for the attitude EKF.
 *
 * Body active only under -DEKF_SELFTEST (see ekf_selftest.h). Uses the
 * float-only m_* backend and the shared linalg kernel so it builds and runs
 * identically on host (SITL) and on the ARM target. Each check reports through
 * the caller's callback; the function returns the failure count.
 *
 * Covered branches:
 *   - init/reset, 6-state vs 9-state selection
 *   - predict + accel-direction update (6-state): static-attitude convergence
 *   - gyro-bias observability (6-state)
 *   - specific-force update (9-state): accel-bias convergence under motion
 *   - accel-trust gate: |a| far from g -> update skipped
 *   - mag-invalid skip: zero mag vector -> yaw update skipped, no NaN
 *   - degenerate accel guard: zero accel -> no NaN
 *   - numerical stability: quaternion finite + unit-norm after a long soak
 */
#include "est/ekf_selftest.h"

#if defined(EKF_SELFTEST)

#include "est/ekf.h"
#include "est/est.h"
#include "maths/linalg.h"
#include "maths/maths_interface.h"

#define EKF_ST_G EKF_GRAVITY
#define EKF_ST_DEG 0.017453292519943295f
#define EKF_ST_2PI 6.2831853071795864f

/* Body-frame accel (m/s^2) and unit mag synthesized from a true attitude.
 * Matches the driver convention: the accelerometer reports the GRAVITY vector
 * (points down), so a level board reads ~ -g on its vertical axis.
 * @noreq EKF self-test fixture (synthesise accel/mag from a truth attitude). */
static void st_synth(float roll, float pitch, float yaw, float acc[3],
                     float mag[3]) {
  quaternion_t qt;
  m_quat_from_euler_deg(roll, pitch, yaw, &qt);
  const float down[3] = {0.0f, 0.0f, -EKF_ST_G};
  const float north[3] = {1.0f, 0.0f, 0.0f};
  m_quat_rotate_inv(&qt, down, acc);
  m_quat_rotate_inv(&qt, north, mag);
}

/* @noreq EKF self-test helper (angle wrap). */
static float st_wrap180(float d) {
  while (d > 180.0f)
    d -= 360.0f;
  while (d < -180.0f)
    d += 360.0f;
  return d;
}

/* @noreq EKF self-test helper (quaternion finite + unit-norm check). */
static bool st_quat_finite_unit(const quaternion_t *q) {
  float n = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
  /* finite check via self-equality bound; NaN fails both comparisons */
  bool finite = (n > 0.0f) && (n < 1.0e6f);
  return finite && (n > 0.9f) && (n < 1.1f);
}

/* On-target EKF correctness self-test: exercises the EST-EKF-001..106 scenarios.
 * @noreq verification harness, not flight behaviour. */
int ekf_selftest_run(ekf_selftest_report_fn report, void *ctx) {
  int fails = 0;
#define REPORT(pass, name)                                                     \
  do {                                                                         \
    bool _p = (pass);                                                          \
    report(ctx, _p, (name));                                                   \
    if (!_p)                                                                   \
      fails++;                                                                 \
  } while (0)

  const float dt = 1.0f / 500.0f;
  float acc[3], mag[3], bias[3];
  attitude_t ori;

  /* --- 6-state: converge to a known static attitude (accel-dir + mag-yaw) --- */
  {
    ekf_init(false);
    const float TR = 20.0f, TP = -15.0f, TY = 40.0f;
    st_synth(TR, TP, TY, acc, mag);
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    for (int i = 0; i < 3000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], 0.0f, 0.0f, 0.0f, mag[0], mag[1],
                   mag[2], dt, &ori);
    REPORT(m_fabsf(st_wrap180(ori.roll - TR)) < 0.5f, "6-state roll converges");
    REPORT(m_fabsf(st_wrap180(ori.pitch - TP)) < 0.5f,
           "6-state pitch converges");
    REPORT(m_fabsf(st_wrap180(ori.yaw - TY)) < 0.5f, "6-state yaw converges");
  }

  /* --- 6-state: constant gyro bias is observed; attitude does not drift --- */
  {
    ekf_init(false);
    st_synth(0.0f, 0.0f, 0.0f, acc, mag);
    const float bias_dps = 3.0f;
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    for (int i = 0; i < 10000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], bias_dps, 0.0f, 0.0f, mag[0], mag[1],
                   mag[2], dt, &ori);
    ekf_get_gyro_bias(bias);
    REPORT(m_fabsf(bias[0] / EKF_ST_DEG - bias_dps) < 0.5f,
           "6-state gyro-bias converges");
    REPORT(m_fabsf(ori.roll) < 1.0f && m_fabsf(ori.pitch) < 1.0f,
           "6-state attitude steady under gyro bias");
  }

  /* --- 9-state under motion: accel bias becomes observable & converges --- */
  {
    ekf_init(true);
    const float ba_true[3] = {0.25f, -0.15f, 0.10f};
    quaternion_t qt = {1.0f, 0.0f, 0.0f, 0.0f};
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    float t = 0.0f, er = 0.0f, ep = 0.0f, ey = 0.0f;
    for (int i = 0; i < 12000; i++) {
      float wx = 60.0f * m_sin(EKF_ST_2PI * 0.30f * t);
      float wy = 45.0f * m_sin(EKF_ST_2PI * 0.20f * t + 1.0f);
      float wz = 30.0f;
      float dth[3] = {wx * EKF_ST_DEG * dt, wy * EKF_ST_DEG * dt,
                      wz * EKF_ST_DEG * dt};
      quaternion_t dq, qn;
      m_quat_exp(dth, &dq);
      m_quat_mul(&qt, &dq, &qn);
      qt = qn;
      m_quat_normalize(&qt);
      const float down[3] = {0.0f, 0.0f, -EKF_ST_G};
      const float north[3] = {1.0f, 0.0f, 0.0f};
      m_quat_rotate_inv(&qt, down, acc);
      m_quat_rotate_inv(&qt, north, mag);
      acc[0] += ba_true[0];
      acc[1] += ba_true[1];
      acc[2] += ba_true[2];
      m_ekf_filter(acc[0], acc[1], acc[2], wx, wy, wz, mag[0], mag[1], mag[2],
                   dt, &ori);
      float tr, tp, ty;
      m_quat_to_euler_deg(&qt, &tr, &tp, &ty);
      er = st_wrap180(ori.roll - tr);
      ep = st_wrap180(ori.pitch - tp);
      ey = st_wrap180(ori.yaw - ty);
      t += dt;
    }
    ekf_get_accel_bias(bias);
    REPORT(m_fabsf(er) < 1.5f && m_fabsf(ep) < 1.5f && m_fabsf(ey) < 1.5f,
           "9-state attitude tracks under motion");
    REPORT(m_fabsf(bias[0] - ba_true[0]) < 0.08f &&
               m_fabsf(bias[1] - ba_true[1]) < 0.08f &&
               m_fabsf(bias[2] - ba_true[2]) < 0.08f,
           "9-state accel-bias converges under motion");
  }

  /* --- reset zeroes the bias state --- */
  {
    ekf_init(false);
    st_synth(0.0f, 0.0f, 0.0f, acc, mag);
    for (int i = 0; i < 2000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], 5.0f, 0.0f, 0.0f, mag[0], mag[1],
                   mag[2], dt, &ori);
    ekf_reset();
    ekf_get_gyro_bias(bias);
    REPORT(m_fabsf(bias[0]) < 1e-6f && m_fabsf(bias[1]) < 1e-6f &&
               m_fabsf(bias[2]) < 1e-6f,
           "reset clears gyro bias");
  }

  /* --- accel-trust gate: |a| far from g -> accel update skipped --- */
  {
    ekf_init(false);
    st_synth(0.0f, 0.0f, 0.0f, acc, mag);
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    /* level the filter first */
    for (int i = 0; i < 1000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], 0.0f, 0.0f, 0.0f, mag[0], mag[1],
                   mag[2], dt, &ori);
    /* now feed a hard 5 g spike on X: well outside the gravity gate, so the
     * accel correction must be rejected and roll/pitch must not lurch. */
    for (int i = 0; i < 200; i++)
      m_ekf_filter(5.0f * EKF_ST_G, 0.0f, EKF_ST_G, 0.0f, 0.0f, 0.0f, mag[0],
                   mag[1], mag[2], dt, &ori);
    REPORT(m_fabsf(ori.roll) < 2.0f && m_fabsf(ori.pitch) < 2.0f,
           "accel gate rejects high-g (attitude held)");
    REPORT(st_quat_finite_unit(&ori.q), "quaternion sane after gated spike");
  }

  /* --- mag-invalid skip: zero mag -> no yaw update, no NaN --- */
  {
    ekf_init(false);
    st_synth(10.0f, -5.0f, 0.0f, acc, mag);
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    for (int i = 0; i < 3000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   dt, &ori);
    /* tilt still pinned by accel even with no mag */
    REPORT(m_fabsf(st_wrap180(ori.roll - 10.0f)) < 0.5f &&
               m_fabsf(st_wrap180(ori.pitch + 5.0f)) < 0.5f,
           "tilt pinned with mag invalid");
    REPORT(st_quat_finite_unit(&ori.q), "quaternion sane with mag invalid");
  }

  /* --- degenerate accel guard: zero accel -> no update, no NaN --- */
  {
    ekf_init(false);
    st_synth(0.0f, 0.0f, 0.0f, acc, mag);
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    for (int i = 0; i < 500; i++)
      m_ekf_filter(0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.5f, mag[0], mag[1], mag[2],
                   dt, &ori);
    REPORT(st_quat_finite_unit(&ori.q), "quaternion sane with zero accel");
  }

  /* --- numerical stability soak: finite + unit-norm after a long run --- */
  {
    ekf_init(false);
    st_synth(5.0f, -5.0f, 90.0f, acc, mag);
    ori = (attitude_t){0};
    ori.q.w = 1.0f;
    for (int i = 0; i < 20000; i++)
      m_ekf_filter(acc[0], acc[1], acc[2], 0.1f, -0.1f, 0.05f, mag[0], mag[1],
                   mag[2], dt, &ori);
    REPORT(st_quat_finite_unit(&ori.q), "quaternion finite+unit after soak");
  }

#undef REPORT
  return fails;
}

#else /* !EKF_SELFTEST */

/* Keep the translation unit non-empty (ISO C) and make a misbuilt test loud. */
int ekf_selftest_run(ekf_selftest_report_fn report, void *ctx) {
  report(ctx, false, "EKF self-test not built (define EKF_SELFTEST)");
  return 1;
}

#endif /* EKF_SELFTEST */
