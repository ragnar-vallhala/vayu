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
 * the cross-term that bounds accel drift; bias gain (k_bias) drives the third
 * state.
 *
 * HISTORY: this filter used to have only two states, and a constant accel bias
 * `b` then left a steady-state velocity offset of (k_alt/k_vel)*b. That is not
 * academic — motor vibration makes the accelerometer under-report gravity by
 * ~1 m/s^2 on this airframe, which produced a phantom -1.5 m/s descent while
 * the craft was climbing, and the height controller answered that with roughly
 * double hover thrust. See docs/plans/vertical-velocity-vibration.md. The third
 * state is the fix: it estimates `b` and subtracts it, so the offset goes away
 * rather than being tolerated.
 *
 * The three gains form a critically-damped 3rd-order complementary filter: with
 * all poles at -w and gains applied at the baro rate f, K1 = 3w, K2 = 3w^2 and
 * K3 = w^3. The pre-existing k_alt = 0.15 at f ~ 16 Hz is w ~ 0.8 rad/s, which
 * puts k_vel at 3w^2/f = 0.12 (the shipped 0.10) and k_bias at w^3/f = 0.032 —
 * so the third state completes the design the first two already implied. */
#ifndef VERT_DEFAULT_K_ALT
#define VERT_DEFAULT_K_ALT 0.15f
#endif
#ifndef VERT_DEFAULT_K_VEL
#define VERT_DEFAULT_K_VEL 0.10f
#endif
#ifndef VERT_DEFAULT_K_BIAS
#define VERT_DEFAULT_K_BIAS 0.03f
#endif

/* Authority limit on the bias state (m/s^2). Bounds how much mismatch the
 * filter will quietly absorb, so a genuinely broken accelerometer (or a baro
 * being blown around by prop wash) cannot wind the state into fiction. Sized
 * ~3x the ~1 m/s^2 vibration bias actually measured on this airframe. */
#ifndef VERT_ACCEL_BIAS_MAX
#define VERT_ACCEL_BIAS_MAX 3.0f
#endif

/* Consecutive clean baro corrections needed to clear `accel_unhealthy` once it
 * latches — PX4's BADACC_PROBATION idea, so the flag cannot chatter at the
 * clamp and drop the height mode in and out. ~5 s at 16 Hz. */
#ifndef VERT_ACCEL_PROBATION_SAMPLES
#define VERT_ACCEL_PROBATION_SAMPLES 80u
#endif

/* --- Rangefinder bias aiding -------------------------------------------
 *
 * The bias state learned from baro alone takes ~8 s to converge, and it is at
 * its weakest exactly when it is needed most: the bias appears BECAUSE the
 * motors spooled, so lift-off happens with the error at full strength and the
 * correction at zero. The reason it is slow is structural — a POSITION
 * measurement only reveals an accel bias after that bias has been integrated
 * twice, so the evidence arrives two integrations late.
 *
 * A rangefinder differentiates into a VELOCITY measurement, which reveals the
 * bias after ONE integration. It is exactly the takeoff window, too: the
 * VL53L0X is in range below ~1.5 m.
 *
 * This aiding drives the BIAS STATE ONLY. It deliberately does not touch
 * altitude or climb_rate, for two separate reasons:
 *   - altitude: the ToF is an AGL reference over whatever is directly below and
 *     steps when the ground does, while the filter tracks a baro reference.
 *     Folding one into the other makes the fused altitude jump on every terrain
 *     step (see the note in vertical_task.c — this preserves that decision).
 *   - climb_rate: a differentiated ToF carries ~0.2-0.3 m/s of noise, and
 *     climb_rate IS the height controller's inner loop. Feeding the bias
 *     instead low-passes that noise through the bias integrator rather than
 *     handing it to the controller. */

/* Per-sample gain on the ToF velocity innovation. Applied at the rangefinder's
 * ~21 Hz, so the effective rate is ~21x this. */
#ifndef VERT_K_TOF_BIAS
#define VERT_K_TOF_BIAS 0.10f
#endif
/* Innovation gate (m/s). A terrain step, a mount-height change or a range
 * glitch all appear as an impossible one-sample velocity; reject rather than
 * let one bad sample slam the bias. Sized well above any real vertical rate
 * this airframe reaches inside the ToF's 1.5 m band. */
#ifndef VERT_TOF_VEL_GATE
#define VERT_TOF_VEL_GATE 3.0f
#endif
/* Longest gap (s) across which differencing two ToF samples is still
 * meaningful. Beyond this the samples are unrelated (dropouts, out-of-range
 * excursions) and the difference is not a velocity. */
#ifndef VERT_TOF_MAX_GAP_S
#define VERT_TOF_MAX_GAP_S 0.25f
#endif

typedef struct {
  float altitude;       /**< m, up-positive (see conventions above). */
  float climb_rate;     /**< m/s, up-positive. */
  float vertical_accel; /**< m/s^2, up-positive — last a_up fed to predict
                         *   (cached for telemetry / takeoff detection). */
  float accel_bias;     /**< m/s^2, estimated DC error in a_up (third state),
                         *   subtracted in predict. Positive means the accel
                         *   reports MORE upward acceleration than is real. */
  float k_alt;          /**< baro position-correction gain (per sample). */
  float k_vel;          /**< baro velocity-correction gain (per sample). */
  float k_bias;         /**< baro bias-correction gain (per sample). */
  float tof_prev;       /**< previous accepted rangefinder AGL (m). */
  bool tof_have_prev;   /**< tof_prev holds a usable sample. */
  uint32_t
      clean_count; /**< consecutive corrections with the bias unsaturated. */
  bool accel_unhealthy; /**< the bias hit its clamp: the accel disagrees with
                         *   the height sources by more than the filter can
                         *   absorb, so climb_rate is NOT trustworthy. Latches;
                         *   clears after VERT_ACCEL_PROBATION_SAMPLES clean
                         *   samples. */
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
  float
      baro_altitude; /**< raw baro altitude, m (same reference as altitude). */
  float agl;         /**< FC-authoritative height above the ground reference
                         *   (captured while disarmed, frozen at arm), m. */
  float agl_tof;     /**< tilt-compensated rangefinder height, m. Only
                         *   meaningful while tof_valid. Kept SEPARATE from
                         *   `agl` on purpose: different zero, different failure
                         *   modes, and the takeoff/landing detector must keep
                         *   running on the baro reference it was tuned against. */
  bool tof_valid;    /**< agl_tof is fresh, in range, and near-level. */
  float hover_est;   /**< measured hover collective (0..1). Seeded from the
                         *   airframe constant, refined in steady level flight.
                         *   The throttle curve centres the stick on this. */
  bool hover_measured;  /**< true once a real in-flight sample moved it. */
  float accel_bias;     /**< m/s^2, the estimator's third state — its estimate
                         *   of the DC error in the vertical accelerometer.
                         *   Published because it is the most direct read-out
                         *   of how badly vibration is corrupting the accel. */
  bool accel_unhealthy; /**< the bias estimate saturated: climb_rate is not
                         *   trustworthy and the height mode must not run. */
  bool valid;           /**< filter seeded. */
  uint32_t timestamp;   /**< DWT cycle stamp of the driving sample. */
} vertical_state_t;

/**
 * @brief Initialise a vertical estimator with explicit correction gains.
 *        Clears the state; the first vert_est_correct() seeds altitude.
 */
void vert_est_init(vertical_estimator_t *ve, float k_alt, float k_vel,
                   float k_bias);

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
 *              vert_world_up_accel(). The estimated bias is subtracted here.
 * @param dt    integration interval (s).
 */
void vert_est_predict(vertical_estimator_t *ve, float a_up, float dt);

/**
 * @brief Correct step (slow, baro rate): fuse an absolute baro altitude.
 *
 * The first call seeds altitude = baro_alt, climb_rate = 0 (bumpless start).
 * Subsequent calls apply the three-gain complementary correction, then update
 * the accel-bias state and the health flag.
 *
 * @param baro_alt  baro altitude (m, up-positive), same reference as `altitude`.
 */
void vert_est_correct(vertical_estimator_t *ve, float baro_alt);

/**
 * @brief Rangefinder aiding for the accel-bias state (see the block comment
 *        above VERT_K_TOF_BIAS).
 *
 * Call ONLY on a fresh, in-range, near-level rangefinder sample — the same
 * gating that qualifies `agl_tof` for publication. Differences it against the
 * previous accepted sample to form a velocity, and drives the bias state with
 * the resulting innovation. Touches NOTHING else: not altitude, not climb_rate.
 *
 * Self-gating: an implausible one-sample velocity, an over-long gap, or an
 * un-seeded filter are all dropped, and the sample history resets so the next
 * pair starts clean.
 *
 * @param agl_tof  tilt-compensated rangefinder height (m).
 * @param dt       seconds since the previous accepted sample.
 */
void vert_est_correct_tof(vertical_estimator_t *ve, float agl_tof, float dt);

/** Drop the rangefinder sample history (call when the ToF stops qualifying, so
 *  the next valid sample is not differenced against a stale one). */
void vert_est_tof_gap(vertical_estimator_t *ve);

#endif /* VAYU_VERTICAL_ESTIMATOR_H */
