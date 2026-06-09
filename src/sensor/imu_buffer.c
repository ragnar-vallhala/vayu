#include "sensor/imu_buffer.h"
#include "ipc.h"            /* CTRL-RATE-101: control-queue notify semaphore */
#include "sensor/bmx160.h"
#include "structure.h"

/* CTRL-RATE-101: binary semaphore the rate loop blocks on. Given once
 * per control-queue push so the loop is woken by IMU-sample arrival
 * rather than clock polling. NULL until imu_buffer_init() runs; pushes
 * before init silently skip the signal (the data ring tolerates it). */
static SemaphoreHandle_t _imu_control_sema = NULL;

/* Binary semaphore the outer (angle) loop blocks on. Given once per attitude
 * control-queue push (one per IMU/control sample), so the angle loop can pace
 * itself off the inner-loop rate (decimated) instead of a fixed clock delay. */
static SemaphoreHandle_t _attitude_control_sema = NULL;

/* SNS-BUF-002: count of IMU samples silently discarded by the OVERWRITE
 * ring when the consumer fell behind. Monotonic; surfaced via telemetry
 * (SYSTEM_ORIGIN_HEALTH). Single 32-bit scalar (R8.6). */
static volatile uint32_t _imu_drop_count = 0;

uint32_t imu_buffer_drop_count(void) { return _imu_drop_count; }

#define IMU_BUFFER_INTERNAL_CAPACITY (IMU_BUFFER_SIZE + 1)
#define IMU_CALIBRATION_TELEMETRY_CAPACITY 2
static bmx160_all_reading_t _imu_buffer_data[IMU_BUFFER_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_telemetry_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_calibration_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_telemetry_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static imu_calibration_telemetry_t _imu_calibration_telemetry_buffer[IMU_CALIBRATION_TELEMETRY_CAPACITY]; // Keep just two
static spsc_fifo_t _imu_fifo;
static spsc_fifo_t _imu_telemetry_queue;
static spsc_fifo_t _imu_calibration_queue;
static spsc_fifo_t _imu_control_queue;
static spsc_fifo_t _attitude_telemetry_queue;
static spsc_fifo_t _attitude_control_queue;
static spsc_fifo_t _imu_calibration_telemetry_queue;

void imu_buffer_init(void) {
  spsc_init(&_imu_fifo, _imu_buffer_data, IMU_BUFFER_INTERNAL_CAPACITY,
            sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_fifo, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_telemetry_queue, _imu_telemetry_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_calibration_queue, _imu_calibration_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_calibration_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_control_queue, _imu_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_telemetry_queue, _attitude_telemetry_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_control_queue, _attitude_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_calibration_telemetry_queue, _imu_calibration_telemetry_buffer,
            IMU_CALIBRATION_TELEMETRY_CAPACITY, sizeof(imu_calibration_telemetry_t));
  spsc_set_policy(&_imu_calibration_telemetry_queue, SPSC_POLICY_OVERWRITE);

  /* CTRL-RATE-101: created empty so the first wait() blocks until the
   * first sample is pushed. */
  _imu_control_sema = v_semaphore_create_binary();
  _attitude_control_sema = v_semaphore_create_binary();
}

void imu_buffer_push(const bmx160_all_reading_t *sample) {
  // Called from DMA ISR (High Priority)
  if (_imu_fifo.buffer == NULL) {
    return;
  }
  /* SNS-BUF-002: spsc leaves one slot empty, so space()==0 means the ring
   * is full and this OVERWRITE write will discard the oldest unread
   * sample — account for it before writing. @implements SNS-BUF-002 */
  if (spsc_space(&_imu_fifo) == 0) {
    _imu_drop_count++;
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
  bool ok = spsc_write(&_imu_control_queue, sample, 1) == 1;
  /* CTRL-RATE-101: wake the rate loop on every arrival. Runs in the
   * IMU task context (bmx160_initiate_read), not an ISR, so the plain
   * give is correct. A binary sema coalesces bursts to a single wake;
   * the OVERWRITE ring guarantees the loop reads the latest sample. */
  if (_imu_control_sema != NULL) {
    v_semaphore_give(_imu_control_sema);
  }
  return ok;
}

bool imu_queue_control_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_control_queue, out_sample, 1) == 1;
}

bool imu_queue_control_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_control_queue, out_sample, 1) == 1;
}

bool imu_queue_control_wait(uint32_t ticks_to_wait) {
  if (_imu_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_control_sema, ticks_to_wait) == VA_PASS;
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
  bool ok = spsc_write(&_attitude_control_queue, attitude, 1);
  /* Wake the outer (angle) loop on every attitude arrival; it decimates
   * these to run at a clean fraction of the inner rate. Same pattern and
   * context (IMU task, not ISR) as imu_queue_control_push. */
  if (_attitude_control_sema != NULL) {
    v_semaphore_give(_attitude_control_sema);
  }
  return ok;
}
bool attitude_queue_control_pop(attitude_t *out_attitude) {
  return spsc_read(&_attitude_control_queue, out_attitude, 1);
}
bool attitude_queue_control_peek(attitude_t *out_attitude) {
  return spsc_peek(&_attitude_control_queue, out_attitude, 1);
}
bool attitude_queue_control_wait(uint32_t ticks_to_wait) {
  if (_attitude_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_attitude_control_sema, ticks_to_wait) == VA_PASS;
}

bool imu_queue_calibration_telemetry_push(const imu_calibration_telemetry_t *sample) {
  return spsc_write(&_imu_calibration_telemetry_queue, sample, 1);
}
bool imu_queue_calibration_telemetry_pop(imu_calibration_telemetry_t *out_sample) {
  return spsc_read(&_imu_calibration_telemetry_queue, out_sample, 1);
}
bool imu_queue_calibration_telemetry_peek(imu_calibration_telemetry_t *out_sample) {
  return spsc_peek(&_imu_calibration_telemetry_queue, out_sample, 1);
}

bool imu_queue_calibration_push(const bmx160_all_reading_t *sample) {
  return spsc_write(&_imu_calibration_queue, sample, 1) == 1;
}
bool imu_queue_calibration_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_calibration_queue, out_sample, 1) == 1;
}
bool imu_queue_calibration_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_calibration_queue, out_sample, 1) == 1;
}
