#ifndef VAYU_IMU_BUFFER_H
#define VAYU_IMU_BUFFER_H

#include "sensor/bmx160.h"
#include <stdbool.h>

#define IMU_BUFFER_SIZE 32

void imu_buffer_init(void);
void imu_buffer_push(const bmx160_all_reading_t *sample);
bool imu_buffer_pop(bmx160_all_reading_t *out_sample);
int imu_buffer_pop_all(bmx160_all_reading_t *out_samples, int max_count);
bool imu_buffer_peek(bmx160_all_reading_t *out_sample);
int imu_buffer_peek_all(bmx160_all_reading_t *out_samples, int max_count);
int imu_buffer_count(void);

#endif // VAYU_IMU_BUFFER_H
