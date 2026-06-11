/**
 * @file est.h
 * @brief Public umbrella header for the estimation module (EST).
 *
 * @implements R2.1
 *
 * Single public surface for the attitude estimator and its support
 * filters (R2.1). Consolidates the former `maths/sensor_fusion.h` and
 * `maths/lpf.h`; the sources live in `src/est/` (R2.6).
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
  bool degraded;      /**< Set when estimator_is_degraded() — see EST-MAH-002. */
  uint32_t timestamp; /**< DWT cycle stamp of the source IMU sample (acquisition
                       *   time). Loops derive dt from deltas of this, not DWT
                       *   read at loop time — see vayu_dt_from_cycles(). */
} attitude_t;

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
                     const float mx, const float my, const float mz,
                     float dt, attitude_t *ori);

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
