#ifndef VAYU_IMU_BUFFER_H
#define VAYU_IMU_BUFFER_H

#include "sensor/bmx160.h"
#include <stdbool.h>
#include <stdint.h>

#define IMU_BUFFER_SIZE 10

void imu_buffer_init(void);
void imu_buffer_push(const bmx160_all_reading_t *sample);

/**
 * @brief Number of IMU samples discarded by the OVERWRITE ring because
 *        the consumer fell behind. Surfaced through telemetry.
 * @implements SNS-BUF-002
 */
uint32_t imu_buffer_drop_count(void);
bool imu_buffer_peek(bmx160_all_reading_t *out_sample);
int imu_buffer_peek_all(bmx160_all_reading_t *out_samples, int max_count);
int imu_buffer_count(void);
bool imu_queue_telemetry_push(const bmx160_all_reading_t *sample);
bool imu_queue_telemetry_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_telemetry_peek(bmx160_all_reading_t *out_sample);

bool imu_queue_control_push(const bmx160_all_reading_t *sample);
bool imu_queue_control_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_control_peek(bmx160_all_reading_t *out_sample);

/**
 * @brief Block until a fresh IMU control sample is pushed, or the
 *        timeout elapses.
 *
 * Backs the event-driven rate loop (CTRL-RATE-101): the rate task waits
 * here instead of clock-polling with v_delay(). The underlying SPSC ring
 * stays lock-free (R8.6) — this is a pure edge notification, not a lock
 * around the data. The producer (imu_queue_control_push) signals it from
 * task context after each write.
 *
 * @param ticks_to_wait  max ticks to block (use MS_TO_TICKS()).
 * @return true if signalled (a sample is likely available), false on
 *         timeout — the caller falls back to the last sample so the
 *         control / failsafe path keeps running if the IMU stalls.
 *
 * @implements CTRL-RATE-101
 */
bool imu_queue_control_wait(uint32_t ticks_to_wait);

bool attitude_queue_telemetry_push(const attitude_t *attitude);
bool attitude_queue_telemetry_pop(attitude_t *out_attitude);
bool attitude_queue_telemetry_peek(attitude_t *out_attitude);

bool attitude_queue_control_push(const attitude_t *attitude);
bool attitude_queue_control_pop(attitude_t *out_attitude);
bool attitude_queue_control_peek(attitude_t *out_attitude);

bool imu_queue_calibration_telemetry_push(const imu_calibration_telemetry_t *sample);
bool imu_queue_calibration_telemetry_pop(imu_calibration_telemetry_t *out_sample);
bool imu_queue_calibration_telemetry_peek(imu_calibration_telemetry_t *out_sample);

bool imu_queue_calibration_push(const bmx160_all_reading_t *sample);
bool imu_queue_calibration_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_calibration_peek(bmx160_all_reading_t *out_sample);

#endif // VAYU_IMU_BUFFER_H
