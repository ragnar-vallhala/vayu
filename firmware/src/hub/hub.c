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
/* IMU samples feeding the dedicated attitude task (estimator input). */

/* Fused vertical state (VERT task -> consumers). Element is 24 B (not a power of
 * two), so — like the calibration ring — give a little headroom so spsc_init's
 * alignment/empty-marker slots don't collapse usable capacity to zero. */
#define VERTICAL_STATE_CAPACITY 4

/* attitude task -> VERT task input ring (synchronized {q, accel, dt}). */
static SemaphoreHandle_t _vert_input_sema = NULL;

/* Latest attitude published to telemetry; see attitude_telemetry_latest(). */
static attitude_t _att_tel_latest;
static volatile bool _att_tel_have;

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
/* Every pipe takes its `reserve` out of the pool at declare, so this is the sum
 * of the slot counts below -- 6+6+4+4+6+4+4+6. Short by one and the declare
 * fails, which is why it PANICs rather than carrying on half-built. */
#define HUB_BUS_BLOCKS 40u
V_BUS_POOL(hub_pool, HUB_BUS_BLOCK, HUB_BUS_BLOCKS);
static v_bus_t _hub_bus;

#define HUB_T_IMU_ATTITUDE "imu.attitude"
#define HUB_T_IMU_CONTROL "imu.control"
#define HUB_T_IMU_TELEMETRY "imu.telemetry"
#define HUB_T_ATT_TELEMETRY "att.telemetry"
#define HUB_T_ATT_CONTROL "att.control"
#define HUB_T_IMU_CALIB "imu.calib"
#define HUB_T_EST_PERF "est.perf"
#define HUB_T_VERT_INPUT "vert.input"

static v_bus_topic_t _t_imu_attitude;
static v_bus_topic_t _t_imu_control;
static v_bus_topic_t _t_imu_telemetry;
static v_bus_topic_t _t_att_telemetry;
static v_bus_topic_t _t_att_control;
static v_bus_topic_t _t_imu_calib;
static v_bus_topic_t _t_est_perf;
static v_bus_topic_t _t_vert_input;

/* Poll-side subscriptions. These use the POINTER api (v_bus_subscribe +
 * v_bus_pop), not an fd: a v_bus_sub_t is plain state, not per-task, so it can be
 * set up once here, and the read costs no SVC trap. Only a consumer that has to
 * BLOCK needs an fd, because v_bus_wait is behind the fd api -- v_bus_pop has no
 * timeout. Two consumers block: attitude (imu.attitude) and vertical
 * (vert.input). */
#if !VAIOS_DEVFS
/* The blocking topics need a poll subscription too: without devfs there is no fd
 * api, so their consumers read through this and take a semaphore for the wake. */
static v_bus_sub_t _s_imu_attitude;
static v_bus_sub_t _s_vert_input;
#endif
#if !VAIOS_DEVFS
static v_bus_sub_t _s_imu_control;
#endif
static v_bus_sub_t _s_imu_telemetry;
static v_bus_sub_t _s_att_telemetry;
#if !VAIOS_DEVFS
static v_bus_sub_t _s_att_control;
#endif
static v_bus_sub_t _s_imu_calib;
static v_bus_sub_t _s_est_perf;

/* Per-topic counters for the FIFOS perf packet.
 *
 * The rings reported peak fill, capacity and drops straight out of the ring
 * object. A bus topic exposes none of that per topic -- v_bus_stats is bus-wide on
 * purpose -- so the hub keeps its own, and they are not a worse answer: every
 * published message is either popped or reported missed, so
 *
 *     in flight = published - popped - missed
 *
 * is exact, and its high-water is what `peak` always meant. `drops` becomes the
 * bus's own `missed`, a counted fact rather than the ring's inference. Capacity is
 * the topic's slot reserve, known at declare. */
typedef struct {
  uint32_t published;
  uint32_t popped;
  uint32_t missed;   /* what a reader was TOLD it lost, learned at pop */
  uint32_t evicted;  /* what the writer overwrote, known at publish */
  uint16_t peak;     /* high-water of in-flight */
  uint16_t capacity; /* slots, from the declare */
} hub_stat_t;

static hub_stat_t _st_imu_telemetry;
static hub_stat_t _st_imu_control;
static hub_stat_t _st_imu_calib;
static hub_stat_t _st_att_telemetry;
static hub_stat_t _st_att_control;

static void hub_stat_pub(hub_stat_t *st, bool ok) {
  if (!ok) {
    return;
  }
  st->published++;
  /* Count the eviction HERE, where the writer causes it, not at the pop that
   * eventually reports `missed`. A V_BUS_OVERWRITE pipe of `capacity` slots drops
   * its oldest whenever a publish would exceed them, so published - popped past
   * capacity is exactly one eviction per publish.
   *
   * Writer-side on purpose: pop-time accounting reads zero for as long as the
   * consumer is stalled, which is precisely when data is being lost and when
   * somebody would want to see it. `missed` is kept as the reader's own view and
   * the two should agree once it catches up. */
  if (st->published - st->popped > st->capacity) {
    st->evicted++;
  }
  const uint32_t live = st->published - st->popped - st->evicted;
  if (live > st->peak) {
    st->peak = (uint16_t)live;
  }
}

static void hub_stat_pop(hub_stat_t *st, bool ok, uint32_t missed) {
  st->missed += missed;
  if (ok) {
    st->popped++;
  }
}
#if VAIOS_DEVFS
/* attitude_task's read handle. The fd table is PER TASK, so this is only valid
 * in the one task that consumes the topic; it is opened lazily on that task's
 * first wait() for exactly that reason -- opening it at init would put the fd in
 * whichever task ran imu_buffer_init (the boot task), where it is useless. */
static int _fd_imu_attitude = -1;
static int _fd_vert_input = -1;
/* imu.control and att.control expose wait() too (no firmware task uses it, but the
 * API does and the host tests exercise it). A topic must be read through ONE
 * subscription: v_bus_open makes its own, so a wait on an fd and a pop on a
 * pointer subscription would be two different queues, and the fd's would fill with
 * messages nobody receives. So where wait() uses an fd, pop() uses the same fd. */
static int _fd_imu_control = -1;
static int _fd_att_control = -1;
#endif
/* Samples the bus says this reader missed, i.e. slots overwritten before it got
 * to them. The SPSC ring it replaces could not report this at all: an overwrite
 * was indistinguishable from never having been published, so "the estimator is
 * keeping up" was an assumption. Here it is a number. Published through
 * hub_imu_attitude_missed() so a bench run can assert it is zero. */
static volatile uint32_t _imu_attitude_missed;
static volatile uint32_t _vert_input_missed;

/* Declare one pipe topic: one producer context, one subscriber, a lock-free ring
 * of `slots` single-block entries, newest-wins on overrun. That is the SPSC ring
 * these replace, and `slots` is each ring's own capacity, so queue depth does not
 * change with the mechanism. */
static void hub_topic(v_bus_topic_t *t, const char *name, uint16_t slots) {
  const v_bus_topic_cfg_t cfg = {
      .overflow = V_BUS_OVERWRITE,
      .reserve = slots,
      .pipe = 1,
  };
  if (v_bus_topic_declare(&_hub_bus, t, name, &cfg) != VA_PASS) {
    PANIC("hub: topic declare failed (pool short of blocks?)");
  }
}

/* As hub_topic, plus the poll-side subscription for consumers that never block. */
static void hub_topic_sub(v_bus_topic_t *t, v_bus_sub_t *sub, const char *name,
                          uint16_t slots) {
  hub_topic(t, name, slots);
  if (v_bus_subscribe(t, sub) != VA_PASS) {
    PANIC("hub: subscribe failed");
  }
}

/** @implements SNS-BUF-001 */
void imu_buffer_init(void) {

  /* CTRL-RATE-101: created empty so the first wait() blocks until the
   * first sample is pushed. */
  _imu_control_sema = v_semaphore_create_binary();
  _attitude_control_sema = v_semaphore_create_binary();
  _imu_attitude_sema = v_semaphore_create_binary();
  if (v_bus_init(&_hub_bus, hub_pool_blocks, hub_pool_desc, HUB_BUS_BLOCK,
                 HUB_BUS_BLOCKS) != VA_PASS) {
    PANIC("hub: bus init failed");
  }
  /* Blocking consumers: declared only. Their reader opens an fd on first wait(),
   * in the task that consumes the topic, because the fd table is per task. */
#if VAIOS_DEVFS
  /* Blocking consumers: declared only. Their reader opens an fd on first wait(),
   * in the task that consumes the topic, because the fd table is per task. */
  hub_topic(&_t_imu_attitude, HUB_T_IMU_ATTITUDE, IMU_BUFFER_INTERNAL_CAPACITY);
  hub_topic(&_t_vert_input, HUB_T_VERT_INPUT, IMU_BUFFER_INTERNAL_CAPACITY);
#else
  /* No fd api: subscribe here like any poll consumer and let the semaphore carry
   * the wake. Same topic, same data, different wake. */
  hub_topic_sub(&_t_imu_attitude, &_s_imu_attitude, HUB_T_IMU_ATTITUDE,
                IMU_BUFFER_INTERNAL_CAPACITY);
  hub_topic_sub(&_t_vert_input, &_s_vert_input, HUB_T_VERT_INPUT,
                IMU_BUFFER_INTERNAL_CAPACITY);
#endif
  /* Poll consumers: declared and subscribed here, once. */
  _st_imu_telemetry.capacity = IMU_TELEMETRY_INTERNAL_CAPACITY;
  _st_imu_control.capacity = IMU_BUFFER_INTERNAL_CAPACITY;
  _st_imu_calib.capacity = IMU_CALIBRATION_TELEMETRY_CAPACITY;
  _st_att_telemetry.capacity = IMU_TELEMETRY_INTERNAL_CAPACITY;
  _st_att_control.capacity = IMU_BUFFER_INTERNAL_CAPACITY;
#if VAIOS_DEVFS
  hub_topic(&_t_imu_control, HUB_T_IMU_CONTROL, IMU_BUFFER_INTERNAL_CAPACITY);
#else
  hub_topic_sub(&_t_imu_control, &_s_imu_control, HUB_T_IMU_CONTROL,
                IMU_BUFFER_INTERNAL_CAPACITY);
#endif
  hub_topic_sub(&_t_imu_telemetry, &_s_imu_telemetry, HUB_T_IMU_TELEMETRY,
                IMU_TELEMETRY_INTERNAL_CAPACITY);
  hub_topic_sub(&_t_att_telemetry, &_s_att_telemetry, HUB_T_ATT_TELEMETRY,
                IMU_TELEMETRY_INTERNAL_CAPACITY);
#if VAIOS_DEVFS
  hub_topic(&_t_att_control, HUB_T_ATT_CONTROL, IMU_BUFFER_INTERNAL_CAPACITY);
#else
  hub_topic_sub(&_t_att_control, &_s_att_control, HUB_T_ATT_CONTROL,
                IMU_BUFFER_INTERNAL_CAPACITY);
#endif
  hub_topic_sub(&_t_imu_calib, &_s_imu_calib, HUB_T_IMU_CALIB,
                IMU_CALIBRATION_TELEMETRY_CAPACITY);
  hub_topic_sub(&_t_est_perf, &_s_est_perf, HUB_T_EST_PERF,
                EST_PERF_TELEMETRY_CAPACITY);
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
  /* From the hub's own counters, not a ring object: these topics are bus pipes and
   * the bus keeps no per-topic depth. See hub_stat_t. */
  const struct {
    uint8_t id;
    const hub_stat_t *st;
  } fifos[] = {
      {PERF_FIFO_IMU_TELEMETRY, &_st_imu_telemetry},
      {PERF_FIFO_IMU_CONTROL, &_st_imu_control},
      {PERF_FIFO_IMU_CALIB_TELEM, &_st_imu_calib},
      {PERF_FIFO_ATTITUDE_TELEMETRY, &_st_att_telemetry},
      {PERF_FIFO_ATTITUDE_CONTROL, &_st_att_control},
  };
  int n = 0;
  for (unsigned i = 0; i < sizeof(fifos) / sizeof(fifos[0]) && n < max; i++) {
    const hub_stat_t *st = fifos[i].st;
    rows[n].fifo_id = fifos[i].id;
    rows[n]._pad = 0;
    rows[n].peak = st->peak;
    rows[n].capacity = st->capacity;
    rows[n].drops = (uint16_t)(st->evicted > 0xFFFFu ? 0xFFFFu : st->evicted);
    n++;
  }
  for (uint8_t i = 0; i < _n_extra_fifos && n < max; i++)
    perf_fifo_fill_row(&rows[n++], _extra_fifos[i].id, _extra_fifos[i].f);
  return n;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_push(const imu_sample_t *sample) {
  const bool ok = v_bus_publish(&_t_imu_telemetry, sample,
                                (uint16_t)sizeof *sample) == VA_PASS;
  hub_stat_pub(&_st_imu_telemetry, ok);
  return ok;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_telemetry_pop(imu_sample_t *out_sample) {
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_imu_telemetry, out_sample, (uint16_t)sizeof *out_sample,
                &len, &missed) == VA_PASS &&
      len == sizeof *out_sample;
  hub_stat_pop(&_st_imu_telemetry, ok, missed);
  return ok;
}

/** @implements CTRL-RATE-101 */
bool imu_queue_control_push(const imu_sample_t *sample) {
  const bool ok = v_bus_publish(&_t_imu_control, sample,
                                (uint16_t)sizeof *sample) == VA_PASS;
  hub_stat_pub(&_st_imu_control, ok);
#if !VAIOS_DEVFS
  /* No fd api: the wake has to be given explicitly, as it always was. With devfs,
   * publish signals the subscription itself. */
  if (_imu_control_sema != NULL) {
    v_semaphore_give(_imu_control_sema);
  }
#endif
  return ok;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_control_pop(imu_sample_t *out_sample) {
#if VAIOS_DEVFS
  if (_fd_imu_control < 0) {
    return false; /* wait() opens the handle */
  }
  v_bus_rx_t rx = {.buf = out_sample, .cap = (uint16_t)sizeof *out_sample};
  const bool ok = v_bus_recv(_fd_imu_control, &rx) == VA_PASS &&
                  rx.len == sizeof *out_sample;
  hub_stat_pop(&_st_imu_control, ok, rx.missed);
  return ok;
#else
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_imu_control, out_sample, (uint16_t)sizeof *out_sample, &len,
                &missed) == VA_PASS &&
      len == sizeof *out_sample;
  hub_stat_pop(&_st_imu_control, ok, missed);
  return ok;
#endif
}

bool imu_queue_control_wait(uint32_t ticks_to_wait) {
#if VAIOS_DEVFS
  if (_fd_imu_control < 0) {
    /* See imu_queue_attitude_wait: per-task fd table, so it is opened by the task
     * that consumes the topic, on its first wait. */
    _fd_imu_control = v_bus_open(HUB_T_IMU_CONTROL, V_BUS_RD);
    if (_fd_imu_control < 0) {
      return false;
    }
  }
  return v_bus_wait(_fd_imu_control, ticks_to_wait) == VA_PASS;
#else
  if (_imu_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_control_sema, ticks_to_wait) == VA_PASS;
#endif
}

/* IMU -> attitude task: feeds the estimator. Same event-driven pattern as the
 * control queue; the OVERWRITE ring keeps only the latest sample on overrun. */
/** @noreq Thin SPSC ring push (+ event wake). */
bool imu_queue_attitude_push(const imu_sample_t *sample) {
  /* With devfs, publish signals the subscription's own semaphore -- from an ISR
   * too -- so the queue and the wake stop being two things that can disagree,
   * which is what the give/take race in this file's history was about. Without
   * devfs there is no fd api, so the wake is given explicitly below. */
  const bool ok = v_bus_publish(&_t_imu_attitude, sample,
                                (uint16_t)sizeof *sample) == VA_PASS;
#if !VAIOS_DEVFS
  if (_imu_attitude_sema != NULL) {
    v_semaphore_give(_imu_attitude_sema);
  }
#endif
  return ok;
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_attitude_pop(imu_sample_t *out_sample) {
#if VAIOS_DEVFS
  if (_fd_imu_attitude < 0) {
    return false; /* wait() opens the handle */
  }
  v_bus_rx_t rx = {.buf = out_sample, .cap = (uint16_t)sizeof *out_sample};
  const bool ok = v_bus_recv(_fd_imu_attitude, &rx) == VA_PASS &&
                  rx.len == sizeof *out_sample;
  if (rx.missed != 0u) {
    _imu_attitude_missed += rx.missed;
  }
  return ok;
#else
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_imu_attitude, out_sample, (uint16_t)sizeof *out_sample,
                &len, &missed) == VA_PASS &&
      len == sizeof *out_sample;
  _imu_attitude_missed += missed;
  return ok;
#endif
}

/** @noreq Thin SPSC ring wait (event-driven). */
bool imu_queue_attitude_wait(uint32_t ticks_to_wait) {
#if VAIOS_DEVFS
  if (_fd_imu_attitude < 0) {
    /* First call from the consuming task, the only task that may hold this fd:
     * the fd table is per task, so opening it at init would file it under
     * whichever task ran boot. A subscriber sees messages published from now on,
     * correct for an overwrite topic where only the newest matters. */
    _fd_imu_attitude = v_bus_open(HUB_T_IMU_ATTITUDE, V_BUS_RD);
    if (_fd_imu_attitude < 0) {
      return false;
    }
  }
  /* wait() then pop(), not v_bus_recv_wait, to keep this header's two-step API.
   * The wake is a HINT: an overwrite topic can evict the slot between the signal
   * and this task running, so a wake with nothing to read is normal and costs one
   * more turn of the caller's loop. Every caller tolerates that. */
  return v_bus_wait(_fd_imu_attitude, ticks_to_wait) == VA_PASS;
#else
  /* No fd api without devfs, so the wake is the semaphore the publisher gives.
   * The DATA goes through the bus either way -- only the wake differs -- which is
   * what lets the host tests exercise the real topic. */
  if (_imu_attitude_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_imu_attitude_sema, ticks_to_wait) == VA_PASS;
#endif
}

/** @noreq How many imu.attitude samples this consumer has missed (overwritten
 *  before it read them). Zero on a healthy bench run; a rising count means the
 *  estimator is not keeping up with the IMU. */
uint32_t hub_imu_attitude_missed(void) { return _imu_attitude_missed; }
/** @noreq As above, for vert.input (the vertical estimator's feed). */
uint32_t hub_vert_input_missed(void) { return _vert_input_missed; }

/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_telemetry_push(const attitude_t *attitude) {
  /* Also kept as a latest value. A reader that wants "the current attitude" -- the
   * SITL display -- must not share this queue with telemetry: reading the head of a
   * single-consumer queue hands out the OLDEST unread item, so such a reader sees
   * attitude as stale as telemetry is behind. See the vertical.state note for the
   * same mistake caught on a live control path. */
  ENTER_CRITICAL();
  _att_tel_latest = *attitude;
  _att_tel_have = true;
  EXIT_CRITICAL();
  const bool ok = v_bus_publish(&_t_att_telemetry, attitude,
                                (uint16_t)sizeof *attitude) == VA_PASS;
  hub_stat_pub(&_st_att_telemetry, ok);
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_telemetry_pop(attitude_t *out_attitude) {
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_att_telemetry, out_attitude, (uint16_t)sizeof *out_attitude,
                &len, &missed) == VA_PASS &&
      len == sizeof *out_attitude;
  hub_stat_pop(&_st_att_telemetry, ok, missed);
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
/** @noreq The newest attitude published to telemetry, consuming nothing and
 *  taking nothing from the telemetry consumer. For readers that want the current
 *  value rather than the stream -- which is every reader that used to peek. */
bool attitude_telemetry_latest(attitude_t *out_attitude) {
  if (out_attitude == NULL) {
    return false;
  }
  bool had;
  ENTER_CRITICAL();
  had = _att_tel_have;
  if (had) {
    *out_attitude = _att_tel_latest;
  }
  EXIT_CRITICAL();
  return had;
}
/** @noreq Thin SPSC ring push (+ event wake). */
bool attitude_queue_control_push(const attitude_t *attitude) {
  const bool ok = v_bus_publish(&_t_att_control, attitude,
                                (uint16_t)sizeof *attitude) == VA_PASS;
  hub_stat_pub(&_st_att_control, ok);
#if !VAIOS_DEVFS
  /* No fd api: the wake has to be given explicitly, as it always was. With devfs,
   * publish signals the subscription itself. */
  if (_attitude_control_sema != NULL) {
    v_semaphore_give(_attitude_control_sema);
  }
#endif
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
bool attitude_queue_control_pop(attitude_t *out_attitude) {
#if VAIOS_DEVFS
  if (_fd_att_control < 0) {
    return false; /* wait() opens the handle */
  }
  v_bus_rx_t rx = {.buf = out_attitude, .cap = (uint16_t)sizeof *out_attitude};
  const bool ok = v_bus_recv(_fd_att_control, &rx) == VA_PASS &&
                  rx.len == sizeof *out_attitude;
  hub_stat_pop(&_st_att_control, ok, rx.missed);
  return ok;
#else
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_att_control, out_attitude, (uint16_t)sizeof *out_attitude,
                &len, &missed) == VA_PASS &&
      len == sizeof *out_attitude;
  hub_stat_pop(&_st_att_control, ok, missed);
  return ok;
#endif
}
/** @noreq Thin SPSC ring wait (event-driven). */
bool attitude_queue_control_wait(uint32_t ticks_to_wait) {
#if VAIOS_DEVFS
  if (_fd_att_control < 0) {
    /* See imu_queue_attitude_wait: per-task fd table, so it is opened by the task
     * that consumes the topic, on its first wait. */
    _fd_att_control = v_bus_open(HUB_T_ATT_CONTROL, V_BUS_RD);
    if (_fd_att_control < 0) {
      return false;
    }
  }
  return v_bus_wait(_fd_att_control, ticks_to_wait) == VA_PASS;
#else
  if (_attitude_control_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_attitude_control_sema, ticks_to_wait) == VA_PASS;
#endif
}

/** @noreq Thin SPSC ring accessor. */
bool est_perf_queue_push(const est_perf_telemetry_t *perf) {
  return v_bus_publish(&_t_est_perf, perf, (uint16_t)sizeof *perf) == VA_PASS;
}
/** @noreq Thin SPSC ring accessor. */
bool est_perf_queue_pop(est_perf_telemetry_t *out_perf) {
  uint16_t len = 0;
  return v_bus_pop(&_s_est_perf, out_perf, (uint16_t)sizeof *out_perf, &len,
                   NULL) == VA_PASS &&
         len == sizeof *out_perf;
}

/* ---------------------------------------------------------------------------
 * vertical.state: ONE SLOT, not a queue.
 *
 * It never wanted to be a ring. Its two readers want different things and a depth
 * of four served neither: angle_controller wants the NEWEST state at every control
 * step and must not consume it, while telemetry wants to send only when there is
 * something new -- so that a stalled estimator shows up as vert frames stopping
 * rather than as the same numbers repeating.
 *
 * A plain latest-value slot (HUB_DEFINE_LATEST above) gives the first and breaks
 * the second: it would answer true forever once populated, and telemetry would
 * re-send stale state at its scheduled rate, masking exactly the fault a reader
 * would want to see. A sequence counter gives both from one slot: peek reads it,
 * pop reads it only when the sequence has moved.
 *
 * This is also why vertical.state is not a bus topic. The bus has no
 * non-consuming peek -- v_bus_peek pins a slot and v_bus_release consumes it -- so
 * a queue primitive of any shape is wrong for a reader that wants "the current
 * value".
 *
 * ONE popper. _vs_seq_popped is a single reader's bookmark; a second consumer
 * calling pop would share it and the two would steal each other's updates. Give
 * the next one its own bookmark (or have it peek) rather than reusing this.
 * ------------------------------------------------------------------------- */
static vertical_state_t _vs_latest;
static volatile uint32_t _vs_seq; /* bumped on every publish; 0 = never */
static uint32_t _vs_seq_popped;   /* the sequence pop() last handed out */

/** @noreq Publish the newest vertical state. */
bool vertical_state_queue_push(const vertical_state_t *vs) {
  if (vs == NULL) {
    return false;
  }
  /* Critical section for the same reason the latest-value topics above use one:
   * without it a reader can splice a new field onto an old timestamp, which is
   * precisely the pair that makes a stale sample look fresh. */
  ENTER_CRITICAL();
  _vs_latest = *vs;
  _vs_seq++;
  EXIT_CRITICAL();
  return true;
}

/** @noreq The newest vertical state, consuming nothing. True once one exists. */
bool vertical_state_queue_peek(vertical_state_t *out_vs) {
  if (out_vs == NULL) {
    return false;
  }
  bool had;
  ENTER_CRITICAL();
  had = (_vs_seq != 0u);
  if (had) {
    *out_vs = _vs_latest;
  }
  EXIT_CRITICAL();
  return had;
}

/** @noreq The newest vertical state, but only if it is NEW since the last pop.
 *  False means the producer has not published since -- which is what lets a
 *  stalled estimator stop the telemetry stream instead of repeating itself. */
bool vertical_state_queue_pop(vertical_state_t *out_vs) {
  if (out_vs == NULL) {
    return false;
  }
  bool fresh;
  ENTER_CRITICAL();
  fresh = (_vs_seq != _vs_seq_popped);
  if (fresh) {
    *out_vs = _vs_latest;
    _vs_seq_popped = _vs_seq;
  }
  EXIT_CRITICAL();
  return fresh;
}

/** @noreq Thin SPSC ring push (+ event wake). */
bool vert_input_queue_push(const vert_input_t *in) {
  const bool ok =
      v_bus_publish(&_t_vert_input, in, (uint16_t)sizeof *in) == VA_PASS;
#if !VAIOS_DEVFS
  if (_vert_input_sema != NULL) {
    v_semaphore_give(_vert_input_sema);
  }
#endif
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
bool vert_input_queue_pop(vert_input_t *out_in) {
#if VAIOS_DEVFS
  if (_fd_vert_input < 0) {
    return false; /* wait() opens the handle */
  }
  v_bus_rx_t rx = {.buf = out_in, .cap = (uint16_t)sizeof *out_in};
  const bool ok =
      v_bus_recv(_fd_vert_input, &rx) == VA_PASS && rx.len == sizeof *out_in;
  if (rx.missed != 0u) {
    _vert_input_missed += rx.missed;
  }
  return ok;
#else
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok = v_bus_pop(&_s_vert_input, out_in, (uint16_t)sizeof *out_in,
                            &len, &missed) == VA_PASS &&
                  len == sizeof *out_in;
  _vert_input_missed += missed;
  return ok;
#endif
}
/** @noreq Thin SPSC ring wait (event-driven). */
bool vert_input_queue_wait(uint32_t ticks_to_wait) {
#if VAIOS_DEVFS
  if (_fd_vert_input < 0) {
    /* See imu_queue_attitude_wait for why the handle is opened here. */
    _fd_vert_input = v_bus_open(HUB_T_VERT_INPUT, V_BUS_RD);
    if (_fd_vert_input < 0) {
      return false;
    }
  }
  return v_bus_wait(_fd_vert_input, ticks_to_wait) == VA_PASS;
#else
  if (_vert_input_sema == NULL) {
    return false;
  }
  return v_semaphore_take(_vert_input_sema, ticks_to_wait) == VA_PASS;
#endif
}

/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_telemetry_push(
    const imu_calibration_telemetry_t *sample) {
  const bool ok =
      v_bus_publish(&_t_imu_calib, sample, (uint16_t)sizeof *sample) == VA_PASS;
  hub_stat_pub(&_st_imu_calib, ok);
  return ok;
}
/** @noreq Thin SPSC ring accessor. */
bool imu_queue_calibration_telemetry_pop(
    imu_calibration_telemetry_t *out_sample) {
  uint16_t len = 0;
  uint32_t missed = 0;
  const bool ok =
      v_bus_pop(&_s_imu_calib, out_sample, (uint16_t)sizeof *out_sample, &len,
                &missed) == VA_PASS &&
      len == sizeof *out_sample;
  hub_stat_pop(&_st_imu_calib, ok, missed);
  return ok;
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
