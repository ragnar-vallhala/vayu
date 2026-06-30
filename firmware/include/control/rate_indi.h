/**
 * @file rate_indi.h
 * @brief Incremental Nonlinear Dynamic Inversion (INDI) rate loop — an optional,
 *        compile-time-selectable alternative to the PID inner loop.
 *
 * Selected like the attitude estimator (SF_FILTER_USED in variables.h): set
 * RATE_CTRL_ALGO_USED to RATE_CTRL_INDI and the rate task dispatches here
 * instead of v_pid_update(). Same per-axis I/O contract as the PID:
 *     u = update(ctrl, rate_sp_degps, rate_meas_degps, dt)
 * so the surrounding throttle-gate / mix / anti-saturation code is unchanged.
 *
 * --- Why INDI (vs the PID we ship) -----------------------------------------
 * The PID *reacts to* rate error with fixed gains it must be tuned to. INDI
 * instead INVERTS the known plant each tick. For the rate plant  wdot = b*u :
 *
 *     u = u_f + (wdot_des - wdot_f) / b                              (1)
 *
 *   wdot_f   filtered measured angular acceleration (d/dt of the LPF'd gyro)
 *   wdot_des desired angular accel from a single outer gain:
 *               wdot_des = k * (rate_sp - rate_meas)                 (2)
 *   u_f      the previously-applied command, filtered with the SAME filter as
 *            the gyro path (synchronized filtering — the key INDI detail; (1)
 *            must difference quantities of matched delay or it self-excites)
 *   b        control effectiveness [deg/s^2 per unit u], one number per axis
 *            (the sysid plant DC gain K, or a thrust-stand dT/du + geometry).
 *
 * Because it works in INCREMENTS around the *measured* acceleration, slowly-
 * varying model error (mass, battery sag, a chipped prop, a wrong b) cancels;
 * only the relative accuracy of b matters. There is one bandwidth gain k per
 * axis instead of a Kp/Ki/Kd/LPF set, and the per-axis difference that makes
 * pitch (K~1381) oscillate while roll (K~563) stays calm is absorbed into b,
 * not re-tuned. This structurally avoids the pitch relay-limit-cycle the PID
 * inner loop is prone to.
 *
 * NOTE this does NOT manufacture authority: at low throttle the mixer still
 * clips the differential at the motor floor. INDI removes the tuning fragility;
 * it does not exempt you from actuator saturation.
 */
#ifndef VAYU_RATE_INDI_H
#define VAYU_RATE_INDI_H

#include <stdbool.h>
#include <stdint.h>

/** Inner-loop algorithm selector (mirrors sensor_fusion_filter_t). */
typedef enum {
  RATE_CTRL_PID  = 0, /**< the shipped per-axis PID (default) */
  RATE_CTRL_INDI = 1, /**< incremental dynamic inversion (this file) */
} rate_ctrl_algo_t;

/** Per-axis INDI state + config. One per roll/pitch/yaw. */
typedef struct {
  /* config */
  float b;          /**< control effectiveness wdot/u [deg/s^2 per unit u] */
  float k;          /**< outer rate-error gain [1/s]: wdot_des = k*(sp-meas) */
  float lpf_rc;     /**< synchronized LPF time constant [s] (gyro deriv + u) */
  float out_min;    /**< command clamp (matches the PID's [-1,1] authority) */
  float out_max;
  /* state */
  float gyr_f;      /**< filtered gyro [deg/s] */
  float wdot_f;     /**< filtered angular acceleration [deg/s^2] */
  float u_f;        /**< synchronized (filtered) applied command */
  bool  initialized;
} rate_indi_t;

/**
 * @brief Configure an axis. b/k/lpf_rc are the only tunables.
 * @param b       effectiveness [deg/s^2 per unit u] (sysid K; per axis).
 * @param k       outer bandwidth gain [1/s].
 * @param lpf_rc  synchronized filter time constant [s] (<=0 ⇒ passthrough,
 *                NOT recommended — the gyro derivative needs filtering).
 */
void rate_indi_init(rate_indi_t *c, float b, float k, float lpf_rc,
                    float out_min, float out_max);

/** Re-seed filter/feedback state (call on the disarmed→ARMED edge, like
 *  v_pid_reset) so no stale wdot/u carries into a new arm. */
void rate_indi_reset(rate_indi_t *c);

/**
 * @brief One INDI step for one axis. Same role as v_pid_update().
 * @param rate_sp    rate setpoint from the angle loop [deg/s]
 * @param rate_meas  measured body rate (gyro) [deg/s]
 * @param dt         loop interval [s]
 * @return normalized command u in [out_min, out_max].
 */
float rate_indi_update(rate_indi_t *c, float rate_sp, float rate_meas, float dt);

/**
 * @brief Feed back the command that was ACTUALLY applied after mixing/anti-
 *        saturation (post-clip), to be used as u_f next tick. Optional but
 *        important under saturation: if the loop keeps integrating against a
 *        command the motors never delivered it behaves like windup. Call once
 *        per axis after the mixer clips, with the realized per-axis differential
 *        (or the clamped u if you don't reconstruct it). If never called,
 *        update() falls back to its own clamped u (still correct, just not
 *        saturation-aware).
 */
void rate_indi_set_applied(rate_indi_t *c, float u_applied);

/* ---- Per-axis defaults (seed from sysid; confirm b on a thrust stand) ------
 * b: rate-plant DC gain K from the sysid fit wdot/u (deg/s^2 per unit u).
 *    roll/pitch from firmware/docs/store + pitch_tune.json; yaw is a placeholder.
 * k: outer bandwidth ~ desired rate-loop crossover [1/s] (start conservative).
 * lpf_rc: ~30-50 Hz to tame the gyro derivative without killing INDI's lead. */
#define DEAFULT_ROLL_INDI_B   563.0f
#define DEAFULT_PITCH_INDI_B  1381.0f
#define DEAFULT_YAW_INDI_B    400.0f   /* placeholder: identify before relying */
/* Conservative seed. k is held ~3x below the rate-loop crossover so there is
 * phase margin against the motor/filter lags (a k set near its own bandwidth
 * has none and hunts at that frequency). b is kept HIGH on purpose so INDI
 * under-actuates (sluggish but stable) rather than over-drives — stable enough
 * to fly a clean sysid chirp and fit the real b. */
#define DEAFULT_RATE_INDI_K   6.0f     /* [1/s] ~1 Hz bandwidth seed */
#define DEAFULT_RATE_INDI_LPF 0.010f   /* [s] ~16 Hz, denoise the gyro derivative */

#endif /* VAYU_RATE_INDI_H */
