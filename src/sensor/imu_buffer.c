#include "sensor/imu_buffer.h"
#include "structure.h"

#define IMU_BUFFER_INTERNAL_CAPACITY (IMU_BUFFER_SIZE + 1)

static bmx160_all_reading_t _imu_buffer_data[IMU_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _imu_fifo;

void imu_buffer_init(void) {
  spsc_init(&_imu_fifo, _imu_buffer_data, IMU_BUFFER_INTERNAL_CAPACITY,
            sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_fifo, SPSC_POLICY_OVERWRITE);
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
