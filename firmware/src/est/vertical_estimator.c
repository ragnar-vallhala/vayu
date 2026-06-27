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
void vert_est_init(vertical_estimator_t *ve, float k_alt, float k_vel) {
  ve->altitude = 0.0f;
  ve->climb_rate = 0.0f;
  ve->vertical_accel = 0.0f;
  ve->k_alt = k_alt;
  ve->k_vel = k_vel;
  ve->initialized = false;
}

/* @noreq trivial init (default gains). */
void vert_est_init_default(vertical_estimator_t *ve) {
  vert_est_init(ve, VERT_DEFAULT_K_ALT, VERT_DEFAULT_K_VEL);
}

/* @noreq trivial state reset (keeps gains). */
void vert_est_reset(vertical_estimator_t *ve) {
  ve->altitude = 0.0f;
  ve->climb_rate = 0.0f;
  ve->vertical_accel = 0.0f;
  ve->initialized = false;
  /* gains preserved */
}

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

void vert_est_predict(vertical_estimator_t *ve, float a_up, float dt) {
  ve->vertical_accel = a_up;
  if (!ve->initialized)
    return; /* don't free-run from an unknown origin before the first baro fix */
  if (dt <= 0.0f)
    return;
  /* Semi-implicit integration: advance position with the pre-update velocity
   * plus the half-step accel term, then advance velocity. */
  ve->altitude += ve->climb_rate * dt + 0.5f * a_up * dt * dt;
  ve->climb_rate += a_up * dt;
}

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
}
