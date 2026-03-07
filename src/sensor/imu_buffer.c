#include "sensor/imu_buffer.h"
#include "core/cortex-m4/interrupt.h"
#include "utils/utils.h"

static bmx160_all_reading_t _imu_ring[IMU_BUFFER_SIZE];
static int _head = 0;
static int _tail = 0;
static int _count = 0;

void imu_buffer_init(void) {
  _head = 0;
  _tail = 0;
  _count = 0;
}

void imu_buffer_push(const bmx160_all_reading_t *sample) {
  // Called from DMA ISR (High Priority)
  // We don't need to disable interrupts here because no other task
  // writes to _head, but pop_all writes to _tail and _count.
  // To be safe against concurrent pop, we use a critical section.
  uint32_t state = hal_disable_global_interrupts();

  _imu_ring[_head] = *sample;
  _head = (_head + 1) % IMU_BUFFER_SIZE;

  if (_count < IMU_BUFFER_SIZE) {
    _count++;
  } else {
    // Overwrite oldest data: Advance tail
    _tail = (_tail + 1) % IMU_BUFFER_SIZE;
  }

  hal_enable_global_interrupts(state);
}

int imu_buffer_pop_all(bmx160_all_reading_t *out_samples, int max_count) {
  // Called from Telemetry Task (Low Priority)
  uint32_t state = hal_disable_global_interrupts();

  int popped = 0;
  while (_count > 0 && popped < max_count) {
    out_samples[popped] = _imu_ring[_tail];
    _tail = (_tail + 1) % IMU_BUFFER_SIZE;
    _count--;
    popped++;
  }

  hal_enable_global_interrupts(state);
  return popped;
}
