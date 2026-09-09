/**
 * @file est.h
 * @brief Public umbrella header for the estimation module (EST).
 *
 * @implements R2.1
 *
 * Single public surface for the attitude estimator and its support
 * filters (R2.1). The sources live in `src/est/`.
 */
#ifndef VAYU_EST_H
#define VAYU_EST_H

#include "maths/maths_interface.h"
#include <stdbool.h>
#include <stdint.h>

/* ----------------------------------------------------------------------------
 * Attitude estimate
 * --------------------------------------------------------------------------*/
typedef struct {
  float roll;
  float pitch;
  float yaw;
  quaternion_t q;
  bool degraded; /**< Set when estimator_is_degraded() — see EST-MAH-002. */
  uint32_t timestamp; /**< DWT cycle stamp of the source IMU sample (acquisition
                       *   time). Loops derive dt from deltas of this, not DWT
                       *   read at loop time — see vayu_dt_from_cycles(). */
} attitude_t;

/* ----------------------------------------------------------------------------
 * Estimator cost probe (attitude_task ATTITUDE_CYCLE_PROBE)
 * --------------------------------------------------------------------------*/

/**
 * @brief One estimator-cost snapshot, published ~1 Hz to the GCS instead of a
 *        text log. peak_us/mean_us are the per-update estimator (EKF/Mahony)
 *        cost over the last ~1 s window; decim/rate_hz describe its effective
 *        update cadence. All floats so the GCS decodes it like the other
 *        SYSTEM_STATUS float payloads. Wire: SYSTEM_ORIGIN_EST_PERF.
 */
typedef struct __attribute__((packed)) {
  float peak_us;
  float mean_us;
  float decim;
  float rate_hz;
} est_perf_telemetry_t;

/* ----------------------------------------------------------------------------
 * Estimator health (EST-MAH-002 / SYS-SAFE-003)
 * --------------------------------------------------------------------------*/

/** Continuous-rejection horizon (ms) before degraded flag is raised. */
#define EST_DEGRADED_TIMEOUT_MS 100U

/**
 * @brief Inform the estimator whether the just-processed IMU sample
 *        passed validity checks (per SNS-IMU-002).
 *
 * Called from the sensor pipeline once per fusion step. Lock-free.
 *
 * @param valid  false ⇒ the sample is rejected (mag-invalid, I2C
 *               read error, range over-saturation, …).
 *
 * @implements EST-MAH-002
 */
void estimator_mark_sample(bool valid);

/**
 * @brief Predicate: has the estimator been on a continuous rejection
 *        run longer than EST_DEGRADED_TIMEOUT_MS?
 *
 * Lock-free; safe from any context including the rate-loop hot path.
 *
 * @implements EST-MAH-002, SYS-SAFE-003
 */
bool estimator_is_degraded(void);

/**
 * @brief Drive the safety consequence: when degraded persists in a
 *        flight-relevant state, request SYSTEM_STATE_FAILSAFE.
 *
 * Called from the same pipeline that drives the fusion step; the
 * latency to FAILSAFE is one IMU period after the degraded flag
 * is raised.
 *
 * @implements SYS-SAFE-003
 */
void estimator_safety_step(void);

/**
 * @brief Re-initialise the estimator's integral feedback state to zero.
 *
 * @implements EST-MAH-105
 */
void estimator_reset(void);

typedef enum {
  SF_COMPLEMENTARY,
  SF_MAHONY,
  SF_EKF,            /**< Error-state EKF: attitude + gyro bias (6-state). */
  SF_EKF_ACCEL_BIAS, /**< As SF_EKF plus accelerometer bias (9-state). */
} sensor_fusion_filter_t;

void m_acc_mag(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, attitude_t *ori);

/* `dt` is the integration interval in seconds, derived by the caller from the
 * source IMU sample's acquisition timestamp (not read from DWT inside). */
void m_complementary_filter(const float ax, const float ay, const float az,
                            const float gx, const float gy, const float gz,
                            const float mx, const float my, const float mz,
                            float dt, attitude_t *ori);

void m_mahony_filter(const float ax, const float ay, const float az,
                     const float gx, const float gy, const float gz,
                     const float mx, const float my, const float mz, float dt,
                     attitude_t *ori);

/* ----------------------------------------------------------------------------
 * Error-state EKF (MEKF) — see src/est/ekf.c, tunables in est/ekf.h.
 * --------------------------------------------------------------------------*/

/**
 * @brief Initialise the EKF instance and its covariance.
 * @param estimate_accel_bias  false ⇒ 6-state (attitude + gyro bias),
 *                             true  ⇒ 9-state (also accel bias).
 */
void ekf_init(bool estimate_accel_bias);

/** Re-seed the EKF (zero covariance growth, re-level on next sample),
 *  preserving the configured state dimension. Called by estimator_reset(). */
void ekf_reset(void);

/** Current estimated gyro bias (rad/s, body axes). */
void ekf_get_gyro_bias(float out[3]);

/** Current estimated accelerometer bias (m/s^2, body axes); zero unless the
 *  9-state (SF_EKF_ACCEL_BIAS) variant is active. */
void ekf_get_accel_bias(float out[3]);

/* Same I/O contract as m_mahony_filter(); `dt` is the inter-sample interval. */
void m_ekf_filter(const float ax, const float ay, const float az,
                  const float gx, const float gy, const float gz,
                  const float mx, const float my, const float mz, float dt,
                  attitude_t *ori);

/* ----------------------------------------------------------------------------
 * First-order low-pass filter (support)
 * --------------------------------------------------------------------------*/
typedef struct {
  float alpha;
  float output;
} lpf_t;

/**
 * @brief Initialize the LPF.
 * @param alpha Smoothing factor (0.0 to 1.0). Smaller value = more filtering.
 */
void lpf_init(lpf_t *lpf, float alpha);

/**
 * @brief Apply the LPF to a new input sample.
 * @return Filtered output.
 */
float lpf_apply(lpf_t *lpf, float input);

#endif // VAYU_EST_H
