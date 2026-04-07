#ifndef VAYU_IMU_BUFFER_H
#define VAYU_IMU_BUFFER_H

#include "sensor/bmx160.h"
#include <stdbool.h>

#define IMU_BUFFER_SIZE 10

void imu_buffer_init(void);
void imu_buffer_push(const bmx160_all_reading_t *sample);
bool imu_buffer_peek(bmx160_all_reading_t *out_sample);
int imu_buffer_peek_all(bmx160_all_reading_t *out_samples, int max_count);
int imu_buffer_count(void);
bool imu_queue_telemetry_push(const bmx160_all_reading_t *sample);
bool imu_queue_telemetry_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_telemetry_peek(bmx160_all_reading_t *out_sample);

bool imu_queue_control_push(const bmx160_all_reading_t *sample);
bool imu_queue_control_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_control_peek(bmx160_all_reading_t *out_sample);

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
