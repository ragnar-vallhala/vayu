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
#include "hub/hub.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "ipc.h" /* CTRL-RATE-101: control-queue notify semaphore */
#include "structure.h"
#include "maths/maths_interface.h" /* m_pow, for hub_altitude_m */
#include "port.h"                  /* ENTER/EXIT_CRITICAL */
#include "utils.h"                 /* v_panic, behind PANIC */

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
static imu_sample_t _imu_telemetry_buffer[IMU_TELEMETRY_INTERNAL_CAPACITY];
static imu_sample_t _imu_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static attitude_t _attitude_telemetry_buffer[IMU_TELEMETRY_INTERNAL_CAPACITY];
static attitude_t _attitude_control_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static imu_calibration_telemetry_t
    _imu_calibration_telemetry_buffer[IMU_CALIBRATION_TELEMETRY_CAPACITY];
static imu_sample_t _imu_attitude_buffer[IMU_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _imu_telemetry_queue;
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

#if VAYU_HUB_BUS
#include "bus.h"
/* ---------------------------------------------------------------------------
 * imu.attitude on the vaios bus (first topic of the FIFO migration).
 *
 * Declared as a PIPE: one producer context, one subscriber, a lock-free ring of
 * `reserve` single-block slots with no critical sections, no ref counts and no
 * shared-pool traffic. That is the same shape the SPSC ring it replaces had, and
 * it is the only shape that belongs here -- the producer is the IMU DMA
 * completion callback, in ISR context, inside a 1 kHz path.
 *
 * V_BUS_OVERWRITE matches SPSC_POLICY_OVERWRITE: on overrun the oldest sample
 * goes and the newest survives, which is what an estimator wants. Readers are
 * told how many they missed.
 *
 * 64 B blocks: the widest hub message is vertical_state_t at 48 B, and
 * V_BUS_HDR_SIZE (12) rides along. One size for every topic keeps the pool
 * uniform when the rest follow.
 */
#define HUB_BUS_BLOCK 64u
#define HUB_BUS_BLOCKS 8u /* the pipe takes `reserve` of these at declare */
#define HUB_T_IMU_ATTITUDE "imu.attitude"
V_BUS_POOL(hub_pool, HUB_BUS_BLOCK, HUB_BUS_BLOCKS);
static v_bus_t _hub_bus;
static v_bus_topic_t _t_imu_attitude;
/* attitude_task's read handle. The fd table is PER TASK, so this is only valid
 * in the one task that consumes the topic; it is opened lazily on that task's
 * first wait() for exactly that reason -- opening it at init would put the fd in
 * whichever task ran imu_buffer_init (the boot task), where it is useless. */
static int _fd_imu_attitude = -1;
/* Samples the bus says this reader missed, i.e. slots overwritten before it got
 * to them. The SPSC ring it replaces could not report this at all: an overwrite
 * was indistinguishable from never having been published, so "the estimator is
 * keeping up" was an assumption. Here it is a number. Published through
 * hub_imu_attitude_missed() so a bench run can assert it is zero. */
static volatile uint32_t _imu_attitude_missed;
#endif

/** @implements SNS-BUF-001 */
void imu_buffer_init(void) {
  spsc_init(&_imu_telemetry_queue, _imu_telemetry_buffer,
            IMU_TELEMETRY_INTERNAL_CAPACITY, sizeof(imu_sample_t));
  spsc_set_policy(&_imu_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_control_queue, _imu_control_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(imu_sample_t));
  spsc_set_policy(&_imu_control_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_imu_attitude_queue, _imu_attitude_buffer,
            IMU_BUFFER_INTERNAL_CAPACITY, sizeof(imu_sample_t));
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
#if VAYU_HUB_BUS
  if (v_bus_init(&_hub_bus, hub_pool_blocks, hub_pool_desc, HUB_BUS_BLOCK,
                 HUB_BUS_BLOCKS) != VA_PASS) {
    PANIC("hub: bus init failed");
  }
  static const v_bus_topic_cfg_t imu_attitude_cfg = {
      .overflow = V_BUS_OVERWRITE,
      .reserve = IMU_BUFFER_INTERNAL_CAPACITY, /* the ring's slot count */
      .pipe = 1,
  };
  if (v_bus_topic_declare(&_hub_bus, &_t_imu_attitude, HUB_T_IMU_ATTITUDE,
                          &imu_attitude_cfg) != VA_PASS) {
    PANIC("hub: imu.attitude declare failed");
  }
#endif
  _vert_input_sema = v_semaphore_create_binary();
}

/* ---------------------------------------------------------------------------
 * Latest-value topics (see hub.h). One slot each, written by the producing
 * driver and read by whoever needs the newest reading.
 *
 * The copy runs in a critical section. These publish at tens of Hz so the cost
 * is nothing, and without it a reader can splice two readings together -- the
 * new pressure against the old timestamp, which is precisely the pair that
 * makes a stale sample look fresh.
 * ------------------------------------------------------------------------- */
#define HUB_DEFINE_LATEST(name, type)                                          \
  static type _##name##_latest;                                                \
  static volatile bool _##name##_have;                                         \
  void name##_publish(const type *sample) {                                    \
    if (sample == NULL)                                                        \
      return;                                                                  \
    ENTER_CRITICAL();                                                          \
    _##name##_latest = *sample;                                                \
    _##name##_have = true;                                                     \
    EXIT_CRITICAL();                                                           \
  }                                                                            \
  bool name##_latest(type *out) {                                              \
    if (out == NULL)                                                           \
      return false;                                                            \
    bool had;                                                                  \
    ENTER_CRITICAL();                                                          \
    had = _##name##_have;                                                      \
    if (had)                                                                   \
      *out = _##name##_latest;                                                 \
    EXIT_CRITICAL();                                                           \
    return had;                                                                \
  }

HUB_DEFINE_LATEST(mag, mag_sample_t)
HUB_DEFINE_LATEST(baro, baro_sample_t)
HUB_DEFINE_LATEST(range, range_sample_t)
HUB_DEFINE_LATEST(battery, battery_sample_t)

/** @noreq ISA barometric altitude; pure maths, see hub/sample.h. */
float hub_altitude_m(float pressure_pa, float sea_level_pa) {
  if (sea_level_pa <= 0.0f)
    sea_level_pa = HUB_SEA_LEVEL_PA_DEFAULT;
  return 44330.0f * (1.0f - m_pow(pressure_pa / sea_level_pa, 0.190294957f));
}

/* A driver keeps queues the hub cannot know about -- the IMU's calibration
 * sample ring, for one. Rather than the hub naming them (which would put a
 * device back in its header), a driver hands its own rows over at init. */
static struct {
  uint8_t id;
  const spsc_fifo_t *f;
} _extra_fifos[HUB_PERF_EXTRA_MAX];
static uint8_t _n_extra_fifos;

/** @noreq driver-contributed diagnostic FIFO row. */
void hub_perf_register(uint8_t id, const spsc_fifo_t *fifo) {
  if (fifo != NULL && _n_extra_fifos < HUB_PERF_EXTRA_MAX) {
    _extra_fifos[_n_extra_fifos].id = id;
    _extra_fifos[_n_extra_fifos].f = fifo;
    _n_extra_fifos++;
  }
}

/** @implements SNS-BUF-002 */
int imu_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  const struct {
    uint8_t id;
    const spsc_fifo_t *f;
  } fifos[] = {
      {PERF_FIFO_IMU_TELEMETRY, &_imu_telemetry_queue},
      {PERF_FIFO_IMU_CONTROL, &_imu_control_queue},
      {PERF_FIFO_IMU_CALIB_TELEM, &_imu_calibration_telemetry_queue},
      {PERF_FIFO_ATTITUDE_TELEMETRY, &_attitude_telemetry_queue},
      {PERF_FIFO_ATTITUDE_CONTROL, &_attitude_control_queue},
  };
  int n = 0;
  for (unsigned i = 0; i < sizeof(fifos) / sizeof(fifos[0]) && n < max; i++)
    perf_fifo_fill_row(&rows[n++], fifos[i].id, fifos[i].f);
  for (uint8_t i = 0; i < _n_extra_fifos && n < max; i++)
    perf_fifo_fill_row(&rows[n++], _extra_fifos[i].id, _extra_fifos[i].f);
  return n;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_push(const imu_sample_t *sample) {
  return spsc_write(&_imu_telemetry_queue, sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_pop(imu_sample_t *out_sample) {
  return spsc_read(&_imu_telemetry_queue, out_sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_peek(imu_sample_t *out_sample) {
  return spsc_peek(&_imu_telemetry_queue, out_sample, 1) == 1;
}

/** @implements CTRL-RATE-101 */
bool imu_queue_control_push(const imu_sample_t *sample) {
  bool ok = spsc_write(&_imu_control_queue, sample, 1) == 1;
  /* CTRL-RATE-101: wake the rate loop on every arrival. Runs in the
   * IMU driver task context, not an ISR, so the plain
   * give is correct. A binary sema coalesces bursts to a single wake;
   * the OVERWRITE ring guarantees the loop reads the latest sample. */
  if (_imu_control_sema != NULL) {
    v_semaphore_give(_imu_control_sema);
  }
  return ok;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_control_pop(imu_sample_t *out_sample) {
  return spsc_read(&_imu_control_queue, out_sample, 1) == 1;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_control_peek(imu_sample_t *out_sample) {
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
bool imu_queue_attitude_push(const imu_sample_t *sample) {
#if VAYU_HUB_BUS
  /* No separate give: publish signals the subscription's own semaphore, from an
   * ISR too, so the ring and the wakeup stop being two things that can disagree.
   * That pairing is what the give/take race in this file used to be about. */
  return v_bus_publish(&_t_imu_attitude, sample, sizeof *sample) == VA_PASS;
#else
  bool ok = spsc_write(&_imu_attitude_queue, sample, 1) == 1;
  if (_imu_attitude_sema != NULL) {
    v_semaphore_give(_imu_attitude_sema);
  }
  return ok;
#endif
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_attitude_pop(imu_sample_t *out_sample) {
#if VAYU_HUB_BUS
  if (_fd_imu_attitude < 0) {
    return false; /* wait() opens the handle; nothing to read before that */
  }
  v_bus_rx_t rx = {.buf = out_sample, .cap = (uint16_t)sizeof *out_sample};
  const bool ok = v_bus_recv(_fd_imu_attitude, &rx) == VA_PASS &&
                  rx.len == sizeof *out_sample;
  if (rx.missed != 0u) {
    _imu_attitude_missed += rx.missed;
  }
  return ok;
#else
  return spsc_read(&_imu_attitude_queue, out_sample, 1) == 1;
#endif
}

/** @noreq Thin SPSC ring wait (event-driven). */
bool imu_queue_attitude_wait(uint32_t ticks_to_wait) {
#if VAYU_HUB_BUS
  if (_fd_imu_attitude < 0) {
    /* First call from the consuming task, which is the only task that may hold
     * this fd. A subscriber sees messages published from now on, so the samples
     * produced before this are simply not waited for -- correct for an
     * overwrite topic, where only the newest matters. */
    _fd_imu_attitude = v_bus_open(HUB_T_IMU_ATTITUDE, V_BUS_RD);
    if (_fd_imu_attitude < 0) {
      return false;
    }
  }
  /* wait() then pop(), not v_bus_recv_wait, to keep hub.h's two-step API. The
   * wake is a HINT: an overwrite topic can evict the slot between the signal and
   * this task running, so a wake with nothing to read is normal and costs one
   * more turn of the caller's loop. Every caller already tolerates
   * wait()-true-then-pop()-false. */
  return v_bus_wait(_fd_imu_attitude, ticks_to_wait) == VA_PASS;
#else
  if (_imu_attitude_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_attitude_sema, ticks_to_wait) == VA_PASS;
#endif
}

#if VAYU_HUB_BUS
/** @noreq How many imu.attitude samples this consumer has missed (overwritten
 *  before it read them). Zero on a healthy bench run; a rising count means the
 *  estimator is not keeping up with the IMU. */
uint32_t hub_imu_attitude_missed(void) { return _imu_attitude_missed; }
#endif

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

/* Mounting tilt, deg. Written on a calibration load/capture (calibration task
 * or boot), read by the attitude task each publish. Two independent floats:
 * a torn pair is not possible on a 32-bit store, and the worst case is one
 * axis updating a cycle before the other on the one write per calibration. */
static float _board_trim_roll = 0.0f;
static float _board_trim_pitch = 0.0f;

/** @noreq Thin accessor. */
void hub_set_board_trim(float roll_deg, float pitch_deg) {
  _board_trim_roll = roll_deg;
  _board_trim_pitch = pitch_deg;
}

/** @noreq Thin accessor. */
void hub_get_board_trim(float *roll_deg, float *pitch_deg) {
  if (roll_deg)
    *roll_deg = _board_trim_roll;
  if (pitch_deg)
    *pitch_deg = _board_trim_pitch;
}
