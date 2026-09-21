/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sensor/imu_buffer.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "ipc.h" /* CTRL-RATE-101: control-queue notify semaphore */
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

/* Wakes the attitude task on each new IMU sample (event-driven estimation). */
static SemaphoreHandle_t _imu_attitude_sema = NULL;

#define IMU_BUFFER_INTERNAL_CAPACITY (IMU_BUFFER_SIZE + 1)
#define IMU_TELEMETRY_INTERNAL_CAPACITY (IMU_TELEMETRY_BUFFER_SIZE + 1)
/* Need 2 usable slots. spsc_init() costs one slot to the ring's empty marker
 * (usable = capacity-1) and, because imu_calibration_telemetry_t is 21 B (not a
 * power of two), another to buffer-start alignment (capacity-1 again when the
 * static array isn't on a 21-byte boundary, which it never is). 2 + 1 + 1 = 4 —
 * a smaller capacity leaves ZERO usable slots, so calibration progress lands in
 * a dead queue and CALIBRATION_STATUS never goes out. */
#define IMU_CALIBRATION_TELEMETRY_CAPACITY 4
#define EST_PERF_TELEMETRY_CAPACITY 4
static bmx160_all_reading_t
    _imu_telemetry_buffer[IMU_TELEMETRY_INTERNAL_CAPACITY];
static bmx160_all_reading_t
    _imu_calibration_buffer[IMU_TELEMETRY_INTERNAL_CAPACITY];
static bmx160_all_reading_t _imu_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_telemetry_buffer[IMU_TELEMETRY_INTERNAL_CAPACITY];
static attitude_t _attitude_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static imu_calibration_telemetry_t
    _imu_calibration_telemetry_buffer[IMU_CALIBRATION_TELEMETRY_CAPACITY];
static bmx160_all_reading_t _imu_attitude_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _imu_telemetry_queue;
static spsc_fifo_t _imu_calibration_queue;
static spsc_fifo_t _imu_control_queue;
/* IMU samples feeding the dedicated attitude task (estimator input). */
static spsc_fifo_t _imu_attitude_queue;
static spsc_fifo_t _attitude_telemetry_queue;
static spsc_fifo_t _attitude_control_queue;
static spsc_fifo_t _imu_calibration_telemetry_queue;
/* Estimator cost-probe snapshots (attitude task -> telemetry task).
 * Aligned to the element size: spsc_init() rounds a power-of-2-sized element
 * up to its own alignment and, if the buffer isn't already aligned, consumes
 * one slot (capacity-1). For a 16-byte element an unaligned 2-slot ring
 * collapses to capacity 1 (zero usable) and silently holds nothing — so force
 * 16-byte alignment and keep a little headroom. */
static est_perf_telemetry_t _est_perf_buffer[EST_PERF_TELEMETRY_CAPACITY]
    __attribute__((aligned(sizeof(est_perf_telemetry_t))));
static spsc_fifo_t _est_perf_queue;

/* Fused vertical state (VERT task -> consumers). Element is 24 B (not a power of
 * two), so — like the calibration ring — give a little headroom so spsc_init's
 * alignment/empty-marker slots don't collapse usable capacity to zero. */
#define VERTICAL_STATE_CAPACITY 4
static vertical_state_t _vertical_state_buffer[VERTICAL_STATE_CAPACITY];
static spsc_fifo_t _vertical_state_queue;

/* attitude task -> VERT task input ring (synchronized {q, accel, dt}). */
static vert_input_t _vert_input_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _vert_input_queue;
static SemaphoreHandle_t _vert_input_sema = NULL;

/** @implements SNS-BUF-001 */
void imu_buffer_init(void) {
  spsc_init(&_imu_telemetry_queue, _imu_telemetry_buffer,
            IMU_TELEMETRY_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_calibration_queue, _imu_calibration_buffer,
            IMU_TELEMETRY_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_calibration_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_control_queue, _imu_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_attitude_queue, _imu_attitude_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(bmx160_all_reading_t));
  spsc_set_policy(&_imu_attitude_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_telemetry_queue, _attitude_telemetry_buffer,
            IMU_TELEMETRY_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_attitude_control_queue, _attitude_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(attitude_t));
  spsc_set_policy(&_attitude_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(
      &_imu_calibration_telemetry_queue, _imu_calibration_telemetry_buffer,
      IMU_CALIBRATION_TELEMETRY_CAPACITY, sizeof(imu_calibration_telemetry_t));
  spsc_set_policy(&_imu_calibration_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_est_perf_queue, _est_perf_buffer, EST_PERF_TELEMETRY_CAPACITY,
            sizeof(est_perf_telemetry_t));
  spsc_set_policy(&_est_perf_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_vertical_state_queue, _vertical_state_buffer,
            VERTICAL_STATE_CAPACITY, sizeof(vertical_state_t));
  spsc_set_policy(&_vertical_state_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_vert_input_queue, _vert_input_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(vert_input_t));
  spsc_set_policy(&_vert_input_queue, SPSC_POLICY_OVERWRITE);

  /* CTRL-RATE-101: created empty so the first wait() blocks until the
   * first sample is pushed. */
  _imu_control_sema = v_semaphore_create_binary();
  _attitude_control_sema = v_semaphore_create_binary();
  _imu_attitude_sema = v_semaphore_create_binary();
  _vert_input_sema = v_semaphore_create_binary();
}

/** @implements SNS-BUF-002 */
int imu_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  const struct {
    uint8_t id;
    const spsc_fifo_t *f;
  } fifos[] = {
      {PERF_FIFO_IMU_TELEMETRY, &_imu_telemetry_queue},
      {PERF_FIFO_IMU_CONTROL, &_imu_control_queue},
      {PERF_FIFO_IMU_CALIB, &_imu_calibration_queue},
      {PERF_FIFO_IMU_CALIB_TELEM, &_imu_calibration_telemetry_queue},
      {PERF_FIFO_ATTITUDE_TELEMETRY, &_attitude_telemetry_queue},
      {PERF_FIFO_ATTITUDE_CONTROL, &_attitude_control_queue},
  };
  int n = 0;
  for (unsigned i = 0; i < sizeof(fifos) / sizeof(fifos[0]) && n < max; i++)
    perf_fifo_fill_row(&rows[n++], fifos[i].id, fifos[i].f);
  return n;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_push(const bmx160_all_reading_t *sample) {
  return spsc_write(&_imu_telemetry_queue, sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_telemetry_queue, out_sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_telemetry_queue, out_sample, 1) == 1;
}

/** @implements CTRL-RATE-101 */
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

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_control_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_control_queue, out_sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_control_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_control_queue, out_sample, 1) == 1;
}

bool imu_queue_control_wait(uint32_t ticks_to_wait) {
  if (_imu_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_control_sema, ticks_to_wait) == VA_PASS;
}

/* IMU -> attitude task: feeds the estimator. Same event-driven pattern as the
 * control queue; the OVERWRITE ring keeps only the latest sample on overrun. */
/** @noreq Thin SPSC ring push (+ event wake). */
bool imu_queue_attitude_push(const bmx160_all_reading_t *sample) {
  bool ok = spsc_write(&_imu_attitude_queue, sample, 1) == 1;
  if (_imu_attitude_sema != NULL) {
    v_semaphore_give(_imu_attitude_sema);
  }
  return ok;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_attitude_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_attitude_queue, out_sample, 1) == 1;
}

/** @noreq Thin SPSC ring wait (event-driven). */
bool imu_queue_attitude_wait(uint32_t ticks_to_wait) {
  if (_imu_attitude_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_attitude_sema, ticks_to_wait) == VA_PASS;
}

/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_telemetry_push(const attitude_t *attitude) {
  return spsc_write(&_attitude_telemetry_queue, attitude, 1);
}
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_telemetry_pop(attitude_t *out_attitude) {
  return spsc_read(&_attitude_telemetry_queue, out_attitude, 1);
}
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_telemetry_peek(attitude_t *out_attitude) {
  return spsc_peek(&_attitude_telemetry_queue, out_attitude, 1);
}
/** @noreq Thin SPSC ring push (+ event wake). */
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
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_control_pop(attitude_t *out_attitude) {
  return spsc_read(&_attitude_control_queue, out_attitude, 1);
}
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_control_peek(attitude_t *out_attitude) {
  return spsc_peek(&_attitude_control_queue, out_attitude, 1);
}
/** @noreq Thin SPSC ring wait (event-driven). */
bool attitude_queue_control_wait(uint32_t ticks_to_wait) {
  if (_attitude_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_attitude_control_sema, ticks_to_wait) == VA_PASS;
}

/** @noreq Thin SPSC ring accessor. */
bool est_perf_queue_push(const est_perf_telemetry_t *perf) {
  return spsc_write(&_est_perf_queue, perf, 1) == 1;
}
/** @noreq Thin SPSC ring accessor. */
bool est_perf_queue_pop(est_perf_telemetry_t *out_perf) {
  return spsc_read(&_est_perf_queue, out_perf, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool vertical_state_queue_push(const vertical_state_t *vs) {
  return spsc_write(&_vertical_state_queue, vs, 1) == 1;
}
/** @noreq Thin SPSC ring accessor. */
bool vertical_state_queue_pop(vertical_state_t *out_vs) {
  return spsc_read(&_vertical_state_queue, out_vs, 1) == 1;
}
/** @noreq Thin SPSC ring accessor. */
bool vertical_state_queue_peek(vertical_state_t *out_vs) {
  return spsc_peek(&_vertical_state_queue, out_vs, 1) == 1;
}

/** @noreq Thin SPSC ring push (+ event wake). */
bool vert_input_queue_push(const vert_input_t *in) {
  bool ok = spsc_write(&_vert_input_queue, in, 1) == 1;
  if (_vert_input_sema != NULL) {
    v_semaphore_give(_vert_input_sema);
  }
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
bool vert_input_queue_pop(vert_input_t *out_in) {
  return spsc_read(&_vert_input_queue, out_in, 1) == 1;
}
/** @noreq Thin SPSC ring wait (event-driven). */
bool vert_input_queue_wait(uint32_t ticks_to_wait) {
  if (_vert_input_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_vert_input_sema, ticks_to_wait) == VA_PASS;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_telemetry_push(
    const imu_calibration_telemetry_t *sample) {
  return spsc_write(&_imu_calibration_telemetry_queue, sample, 1);
}
/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_telemetry_pop(
    imu_calibration_telemetry_t *out_sample) {
  return spsc_read(&_imu_calibration_telemetry_queue, out_sample, 1);
}
/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_telemetry_peek(
    imu_calibration_telemetry_t *out_sample) {
  return spsc_peek(&_imu_calibration_telemetry_queue, out_sample, 1);
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_push(const bmx160_all_reading_t *sample) {
  return spsc_write(&_imu_calibration_queue, sample, 1) == 1;
}
/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_pop(bmx160_all_reading_t *out_sample) {
  return spsc_read(&_imu_calibration_queue, out_sample, 1) == 1;
}
/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_peek(bmx160_all_reading_t *out_sample) {
  return spsc_peek(&_imu_calibration_queue, out_sample, 1) == 1;
}
