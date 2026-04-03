#include "sensor/imu_buffer.h"
#include "sensor/bmx160.h"
#include "structure.h"

#define IMU_BUFFER_INTERNAL_CAPACITY (IMU_BUFFER_SIZE + 1)

static bmx160_all_reading_t _imu_buffer_data[IMU_BUFFER_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_telemetry_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_telemetry_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];

static spsc_fifo_t _imu_fifo;
static spsc_fifo_t _imu_telemetry_queue;
static spsc_fifo_t _imu_control_queue;
static spsc_fifo_t _attitude_telemetry_queue;
static spsc_fifo_t _attitude_control_queue;

void imu_buffer_init(void) {
  spsc_init(&_imu_fifo, _imu_buffer_data, IMU_BUFFER_INTERNAL_CAPACITY,
            sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_fifo, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_telemetry_queue, _imu_telemetry_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_control_queue, _imu_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_telemetry_queue, _attitude_telemetry_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_control_queue, _attitude_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_control_queue, SPSC_POLICY_OVERWRITE);
}

void imu_buffer_push(const bmx160_all_reading_t *sample) {
  // Called from DMA ISR (High Priority)
  if (_imu_fifo.buffer == NULL) {
    return;
  }
  // spsc_write with SPSC_POLICY_OVERWRITE handles full buffer by skipping
  // oldest
  spsc_write(&_imu_fifo, sample, 1);
}

bool imu_buffer_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_fifo, out_sample, 1) == 1;
}

int imu_buffer_peek_all(bmx160_all_reading_t *out_samples, int max_count) {
  return (int)spsc_peek(&_imu_fifo, out_samples, (size_t)max_count);
}

int imu_buffer_count(void) { return (int)spsc_available(&_imu_fifo); }

bool imu_queue_telemetry_push(const bmx160_all_reading_t *sample) {
  return spsc_write(&_imu_telemetry_queue, sample, 1) == 1;
}

bool imu_queue_telemetry_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_telemetry_queue, out_sample, 1) == 1;
}

bool imu_queue_telemetry_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_telemetry_queue, out_sample, 1) == 1;
}

bool imu_queue_control_push(const bmx160_all_reading_t *sample) {
  return spsc_write(&_imu_control_queue, sample, 1) == 1;
}

bool imu_queue_control_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_control_queue, out_sample, 1) == 1;
}

bool imu_queue_control_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_control_queue, out_sample, 1) == 1;
}

bool attitude_queue_telemetry_push(const attitude_t *attitude) {
  return spsc_write(&_attitude_telemetry_queue, attitude, 1);
}
bool attitude_queue_telemetry_pop(attitude_t *out_attitude) {
  return spsc_read(&_attitude_telemetry_queue, out_attitude, 1);
}
bool attitude_queue_telemetry_peek(attitude_t *out_attitude) {
  return spsc_peek(&_attitude_telemetry_queue, out_attitude, 1);
}
bool attitude_queue_control_push(const attitude_t *attitude) {
  return spsc_write(&_attitude_control_queue, attitude, 1);
}
bool attitude_queue_control_pop(attitude_t *out_attitude) {
  return spsc_read(&_attitude_control_queue, out_attitude, 1);
}
bool attitude_queue_control_peek(attitude_t *out_attitude) {
  return spsc_peek(&_attitude_control_queue, out_attitude, 1);
}