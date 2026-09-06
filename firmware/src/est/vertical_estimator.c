/**
 * @file vertical_estimator.c
 * @brief Two-state vertical estimator core (VERT) — pure math, host-testable.
 *
 * The complementary / steady-state-Kalman fusion of baro altitude with
 * accel-derived vertical motion. No HAL, no queues, no globals — the sibling
 * VERT task (src/est/vertical_task.c) owns an instance and wires the I/O.
 *
 * See include/est/vertical_estimator.h for the sign conventions (up-positive)
 * and the model rationale.
 */
#include "est/vertical_estimator.h"
#include "maths/linalg.h"

/* @noreq trivial init (explicit gains). */
void vert_est_init(vertical_estimator_t *ve, float k_alt, float k_vel,
                   float k_bias) {
  ve->k_alt = k_alt;
  ve->k_vel = k_vel;
  ve->k_bias = k_bias;
  vert_est_reset(ve);
}

/* @noreq trivial init (default gains). */
void vert_est_init_default(vertical_estimator_t *ve) {
  vert_est_init(ve, VERT_DEFAULT_K_ALT, VERT_DEFAULT_K_VEL,
                VERT_DEFAULT_K_BIAS);
}

/* @noreq trivial state reset (keeps gains). */
void vert_est_reset(vertical_estimator_t *ve) {
  ve->altitude = 0.0f;
  ve->climb_rate = 0.0f;
  ve->vertical_accel = 0.0f;
  /* The bias is a property of the SENSOR under vibration, not of this flight's
   * altitude reference, so in principle it could survive a re-seed. It is
   * cleared anyway: a reset means we no longer trust where we are, the bias
   * re-converges in a few seconds, and carrying a stale one into a fresh
   * reference is the failure that is hard to see. */
  ve->accel_bias = 0.0f;
  ve->clean_count = 0;
  ve->accel_unhealthy = false;
  ve->initialized = false;
  /* gains preserved */
}

/* @implements EST-ALT-101 */
float vert_world_up_accel(const quaternion_t *q, const float a_body[3]) {
  /* Rotate body specific force into the world frame (FRD: +Z down). For a
   * stationary, level craft the IMU reads a_body ~ (0,0,-g), which rotates to
   * world ~ (0,0,-g); the world-DOWN inertial accel is then (a_world_z + g) = 0
   * once gravity is removed. Up-positive is the negative of that. */
  float a_world[3];
  m_quat_rotate(q, a_body, a_world);
  float a_down_inertial = a_world[2] + VERT_GRAVITY; /* down-positive, ~0 static */
  return -a_down_inertial;                           /* up-positive */
}

/* @implements EST-ALT-001, EST-ALT-101 */
void vert_est_predict(vertical_estimator_t *ve, float a_up, float dt) {
  /* Remove the estimated DC error before integrating. `vertical_accel` caches
   * the CORRECTED value, because that is what every consumer wants: telemetry
   * charting real motion, and the hover estimator's steady-flight gate, which
   * would otherwise be held open or shut by the bias rather than by motion. */
  float a_corr = a_up - ve->accel_bias;
  ve->vertical_accel = a_corr;
  if (!ve->initialized)
    return; /* don't free-run from an unknown origin before the first baro fix */
  if (dt <= 0.0f)
    return;
  /* Semi-implicit integration: advance position with the pre-update velocity
   * plus the half-step accel term, then advance velocity. */
  ve->altitude += ve->climb_rate * dt + 0.5f * a_corr * dt * dt;
  ve->climb_rate += a_corr * dt;
}

/* @implements EST-ALT-001, EST-ALT-101 */
void vert_est_correct(vertical_estimator_t *ve, float baro_alt) {
  if (!ve->initialized) {
    /* Seed bumplessly: trust the first baro reading, assume level hover. */
    ve->altitude = baro_alt;
    ve->climb_rate = 0.0f;
    ve->initialized = true;
    return;
  }
  float err = baro_alt - ve->altitude;
  ve->altitude += ve->k_alt * err;   /* pull position toward baro */
  ve->climb_rate += ve->k_vel * err; /* cross-term bounds accel drift */

  /* Third state. A persistently positive innovation (baro above the filter)
   * means we have been integrating less upward acceleration than was real, so
   * the accel reads LOW and the bias must go negative — hence the minus. This
   * is the same signal PX4's checkVerticalAccelerationHealth() watches, used
   * here to CANCEL the error rather than merely to flag it. */
  ve->accel_bias -= ve->k_bias * err;

  bool saturated = false;
  if (ve->accel_bias > VERT_ACCEL_BIAS_MAX) {
    ve->accel_bias = VERT_ACCEL_BIAS_MAX;
    saturated = true;
  } else if (ve->accel_bias < -VERT_ACCEL_BIAS_MAX) {
    ve->accel_bias = -VERT_ACCEL_BIAS_MAX;
    saturated = true;
  }

  /* Health: saturation means the mismatch is beyond what the bias state can
   * absorb, so climb_rate is once again being dragged by an uncorrected error.
   * Latch immediately, release only after a run of clean samples (probation),
   * so the flag cannot chatter across the clamp. */
  if (saturated) {
    ve->clean_count = 0;
    ve->accel_unhealthy = true;
  } else if (ve->accel_unhealthy &&
             ++ve->clean_count >= VERT_ACCEL_PROBATION_SAMPLES) {
    ve->accel_unhealthy = false;
    ve->clean_count = 0;
  }
}
