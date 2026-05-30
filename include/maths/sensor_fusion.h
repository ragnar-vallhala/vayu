#ifndef VAYU_MATHS_SENSOR_FUSION_H
#define VAYU_MATHS_SENSOR_FUSION_H
#include "maths/maths_interface.h"
#include <stdbool.h>

typedef struct {
  float roll;
  float pitch;
  float yaw;
  quaternion_t q;
  bool degraded;   /**< Set when estimator_is_degraded() — see EST-MAH-002. */
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

typedef enum {
  SF_COMPLEMENTARY,
  SF_MAHONY,
} sensor_fusion_filter_t;

void m_acc_mag(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, attitude_t *ori);

void m_complementary_filter(const float ax, const float ay, const float az,
                            const float gx, const float gy, const float gz,
                            const float mx, const float my, const float mz,
                            attitude_t *ori);

void m_mahony_filter(const float ax, const float ay, const float az,
                     const float gx, const float gy, const float gz,
                     const float mx, const float my, const float mz,
                     attitude_t *ori);

#endif // VAYU_MATHS_SENSOR_FUSION_H