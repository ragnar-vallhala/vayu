/**
 * @file vertical_estimator.h
 * @brief Two-state vertical estimator (VERT): fuses baro altitude with the
 *        accelerometer into [altitude, climb_rate].
 *
 * Raw baro altitude (~10-25 Hz, +/-0.5-2 m noise) is too noisy/laggy to
 * differentiate for a usable climb rate. This module runs the minimal version
 * of what every altitude-holding autopilot does: a 2-state complementary /
 * steady-state Kalman filter that integrates world-vertical inertial
 * acceleration (low lag, drifts) and corrects it against baro altitude
 * (absolute, noisy). The cross-term (velocity nudged by position error) bounds
 * accel-bias drift.
 *
 * Conventions (this module is UP-POSITIVE, matching baro altitude):
 *   - `altitude`   metres, increasing upward (MSL or AGL — whatever the baro
 *                  altitude fed to vert_est_correct() references).
 *   - `climb_rate` m/s, positive climbing.
 *   - `a_up`       world-vertical inertial acceleration, m/s^2, positive up,
 *                  gravity removed (a stationary, level craft reads ~0).
 *
 * The IMU/driver reports the gravity vector (a level board reads ~ -g on its
 * body-down Z axis; see src/est/ekf.c accel-update note and
 * firmware/docs/reference/coordinate_ref.md — FRD, +Z down). vert_world_up_accel()
 * rotates body specific force into the world frame and removes gravity, so the
 * sign bookkeeping lives in one place.
 *
 * Pure/host-testable: no globals, no HAL, no queues. The sibling VERT task
 * (src/est/vertical_estimator.c task wrapper) owns one instance and the I/O.
 */
#ifndef VAYU_VERTICAL_ESTIMATOR_H
#define VAYU_VERTICAL_ESTIMATOR_H

#include "maths/maths_interface.h"
#include <stdbool.h>
#include <stdint.h>

/* Standard gravity (m/s^2). Matches EKF_GRAVITY; kept local so this module does
 * not pull in the EKF header. */
#ifndef VERT_GRAVITY
#define VERT_GRAVITY 9.80665f
#endif

/* Default baro-correction gains (per baro sample), a starting point for a
 * ~16 Hz baro + ~250 Hz predict; re-tuned in SITL.
 *
 * Position gain (k_alt) pulls altitude toward baro; velocity gain (k_vel) is
 * the cross-term that bounds accel drift. NOTE: a constant accel bias `b`
 * leaves a steady-state velocity offset of (k_alt/k_vel)*b — a 2-state filter
 * cannot fully separate a DC accel bias from true velocity (that needs a 3rd
 * bias state). So the ratio is kept near 1 (not >> 1) to keep that offset
 * small; k_vel is not driven higher only because it also injects baro noise
 * into climb_rate. See test_vertical_est.c, which asserts this exact offset. */
#ifndef VERT_DEFAULT_K_ALT
#define VERT_DEFAULT_K_ALT 0.15f
#endif
#ifndef VERT_DEFAULT_K_VEL
#define VERT_DEFAULT_K_VEL 0.10f
#endif

typedef struct {
  float altitude;       /**< m, up-positive (see conventions above). */
  float climb_rate;     /**< m/s, up-positive. */
  float vertical_accel; /**< m/s^2, up-positive — last a_up fed to predict
                         *   (cached for telemetry / takeoff detection). */
  float k_alt;          /**< baro position-correction gain (per sample). */
  float k_vel;          /**< baro velocity-correction gain (per sample). */
  bool initialized;     /**< false until the first baro correction seeds it. */
} vertical_estimator_t;

/**
 * @brief Published vertical state — the VERT task drains this into a SPSC ring
 *        for the control loop / IN_AIR detector / telemetry (VERTICAL_STATE).
 *
 * `baro_altitude` is the raw sensor altitude carried alongside the fused
 * outputs so the GCS can chart fused-vs-raw. `valid` mirrors
 * vertical_estimator_t.initialized.
 */
typedef struct {
  float altitude;       /**< fused, m up-positive (absolute/MSL reference). */
  float climb_rate;     /**< fused, m/s up-positive. */
  float vertical_accel; /**< m/s^2 up-positive. */
  float baro_altitude;  /**< raw baro altitude, m (same reference as altitude). */
  float agl;            /**< FC-authoritative height above the ground reference
                         *   (captured while disarmed, frozen at arm), m. */
  float agl_tof;        /**< tilt-compensated rangefinder height, m. Only
                         *   meaningful while tof_valid. Kept SEPARATE from
                         *   `agl` on purpose: different zero, different failure
                         *   modes, and the takeoff/landing detector must keep
                         *   running on the baro reference it was tuned against. */
  bool tof_valid;       /**< agl_tof is fresh, in range, and near-level. */
  bool valid;           /**< filter seeded. */
  uint32_t timestamp;   /**< DWT cycle stamp of the driving sample. */
} vertical_state_t;

/**
 * @brief Initialise a vertical estimator with explicit correction gains.
 *        Clears the state; the first vert_est_correct() seeds altitude.
 */
void vert_est_init(vertical_estimator_t *ve, float k_alt, float k_vel);

/** Initialise with the default VERT_DEFAULT_K_* gains. */
void vert_est_init_default(vertical_estimator_t *ve);

/** Zero the state and require re-seeding on the next correction (keeps gains).
 *  Use when the craft disarms / the ground reference is recaptured. */
void vert_est_reset(vertical_estimator_t *ve);

/**
 * @brief World-vertical inertial acceleration (m/s^2, up-positive), gravity
 *        removed, from body specific force and the attitude quaternion.
 *
 * @param q       body->world attitude quaternion (from the EKF / attitude task).
 * @param a_body  body-frame specific force (m/s^2), as the IMU reports it.
 * @return        a_up: ~0 for a stationary craft at any attitude.
 */
float vert_world_up_accel(const quaternion_t *q, const float a_body[3]);

/**
 * @brief Predict step (fast, IMU/attitude rate): integrate vertical accel.
 *
 * No-op until the estimator has been seeded by a baro correction (so altitude
 * does not free-run from an unknown origin). Caches a_up in vertical_accel.
 *
 * @param a_up  world-up inertial acceleration (m/s^2), e.g. from
 *              vert_world_up_accel().
 * @param dt    integration interval (s).
 */
void vert_est_predict(vertical_estimator_t *ve, float a_up, float dt);

/**
 * @brief Correct step (slow, baro rate): fuse an absolute baro altitude.
 *
 * The first call seeds altitude = baro_alt, climb_rate = 0 (bumpless start).
 * Subsequent calls apply the two-gain complementary correction.
 *
 * @param baro_alt  baro altitude (m, up-positive), same reference as `altitude`.
 */
void vert_est_correct(vertical_estimator_t *ve, float baro_alt);

#endif /* VAYU_VERTICAL_ESTIMATOR_H */
