/**
 * @file src/sys/sys_utils.c
 * @brief System utilities — timestamp, device id, CRC32.
 */
#include "sys/sys_utils.h"

#include "ipc.h"
#include "navhal.h" /* hal_crc_* */
#include "variables.h"
#include <stdint.h>

static volatile uint64_t _time_stamp_high_freq = 0;
/* @noreq HF monotonic tick increment; runs as a TIM5 callback. */
void increment_high_freq_timer(void) { _time_stamp_high_freq++; }
static uint8_t _device_id = 0;

/* Disciplined clock offset (ms). _offset_target is where the offset should be;
 * _offset_applied is slewed toward it each discipline tick so the unix clock
 * changes smoothly. double (not float) so the offset is exact to the ms. */
static volatile double _offset_applied = 0.0;
static volatile int64_t _offset_target = 0;
static uint64_t _last_discipline_ticks = 0;

/* §10.5 security gate: the FC starts UNSYNCHRONISED and stays so until the GCS
 * has disciplined its clock at least once via the time-sync handshake. Commands
 * are rejected until then (navlink_router's command_gate consults this). Set
 * inside time_sync_set_offset — the one point where the clock is actually
 * aligned to GCS time — so a completed-but-uncorrected round trip doesn't count. */
static volatile uint8_t _clock_synced = 0;
/* @implements SYS-SAFE-007 */
uint8_t time_sync_is_synced(void) { return _clock_synced; }

/* Max slew (ms per second). Kept far below the 1000 ms/s tick rate so
 * get_timestamp_unix() is strictly increasing (1.0 + slew/1000 > 0). */
#define TIME_SYNC_MAX_SLEW_MS_PER_S 50.0
/* Proportional time constant (s): offset closes ~exponentially with this tau,
 * subject to the slew clamp. */
#define TIME_SYNC_TAU_S 2.0
/* Corrections larger than this step immediately — slewing a big offset (cold
 * start, resync, post-reboot) at the clamp would take minutes. Smaller residual
 * corrections slew smoothly to stay monotonic and filter jitter. */
#define TIME_SYNC_STEP_MS 200

/* @noreq trivial accessor: raw monotonic HF tick count. */
uint64_t get_timestamp(void) { return _time_stamp_high_freq; }

/* Raw monotonic ms from the HF tick counter (no offset). 64-bit so it does not
 * wrap at the uint32 ~49.7-day boundary and can carry full epoch ms.
 * @noreq ticks->ms conversion helper. */
static uint64_t timestamp_raw_ms(void) {
  return _time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000);
}

/* @implements SYS-TIM-006 disciplined wall clock = monotonic ms + GCS offset. */
uint64_t get_timestamp_unix(void) {
  return (uint64_t)((int64_t)timestamp_raw_ms() + (int64_t)_offset_applied);
}

/* Apply a GCS clock correction — the measured FC-GCS error to remove. Large
 * errors step immediately (cold start / resync / post-reboot); small residuals
 * slew. Either way the new applied offset converges to GCS time and a
 * disciplined clock reports correction ~= 0 (windup-free).
 * @implements SYS-TIM-006 */
void time_sync_set_offset64(int64_t correction_ms) {
  _clock_synced = 1; /* GCS has disciplined our clock — FC is now synchronised (§10.5) */
  if (correction_ms > TIME_SYNC_STEP_MS || correction_ms < -TIME_SYNC_STEP_MS) {
    _offset_applied += (double)correction_ms;   /* step */
    _offset_target = (int64_t)_offset_applied;  /* park the slew target */
  } else {
    _offset_target = (int64_t)_offset_applied + correction_ms; /* slew */
  }
}

/* @implements SYS-TIM-006 32-bit offset path (delegates to the 64-bit setter). */
void time_sync_set_offset(int32_t correction_ms) {
  time_sync_set_offset64((int64_t)correction_ms);
}

/* @implements SYS-TIM-106 slew-limited clock discipline tick. */
void time_sync_discipline_tick(void) {
  uint64_t now = _time_stamp_high_freq;
  if (_last_discipline_ticks == 0) {
    _last_discipline_ticks = now;
    return;
  }
  double dt = (double)(now - _last_discipline_ticks) / (double)HIGH_FREQ_TIMER_FREQ;
  _last_discipline_ticks = now;
  if (dt <= 0.0)
    return;

  double err = (double)_offset_target - _offset_applied;
  double slew = err / TIME_SYNC_TAU_S; /* ms per second */
  if (slew > TIME_SYNC_MAX_SLEW_MS_PER_S)
    slew = TIME_SYNC_MAX_SLEW_MS_PER_S;
  else if (slew < -TIME_SYNC_MAX_SLEW_MS_PER_S)
    slew = -TIME_SYNC_MAX_SLEW_MS_PER_S;
  _offset_applied += slew * dt;
}

/* One-shot setter: a thin wrapper that jams the offset absolutely. The time-sync
 * handshake drives clock discipline instead.
 * @implements SYS-TIM-006 */
void set_timestamp(uint64_t timestamp) {
  _offset_target = (int64_t)timestamp - (int64_t)timestamp_raw_ms();
  _offset_applied = (double)_offset_target;
}

/* @noreq trivial accessor/setter for the device id. */
uint8_t get_device_id(void) { return _device_id; }
/* @noreq trivial accessor/setter for the device id. */
void set_device_id(uint8_t device_id) { _device_id = device_id; }

static MutexHandle_t crc_mutex = NULL;

/* @noreq mutex-guarded wrapper over the HAL CRC32 unit (HAL-CRC-001);
 * shared helper, no standalone SYS requirement. */
uint32_t utils_compute_crc32(const uint8_t *data, uint32_t len) {
  if (crc_mutex == NULL) {
    crc_mutex = v_mutex_create();
  }
  if (v_mutex_lock(crc_mutex, 0xFFFFFFFF)) {
    hal_crc_config_t crc_cfg = {.polynomial = HAL_CRC_POLY_CRC32,
                                .init_value = 0xFFFFFFFF};
    hal_crc_init(&crc_cfg);
    uint32_t computed_crc = hal_crc_compute(data, len);

    v_mutex_unlock(crc_mutex);
    return computed_crc;
  }
  return 0;
}

/* @noreq non-blocking (try-lock) variant of utils_compute_crc32. */
uint32_t utils_try_compute_crc32(const uint8_t *data, uint32_t len) {
  if (crc_mutex == NULL) {
    crc_mutex = v_mutex_create();
  }
  if (v_mutex_lock(crc_mutex, 0)) {
    hal_crc_config_t crc_cfg = {.polynomial = HAL_CRC_POLY_CRC32,
                                .init_value = 0xFFFFFFFF};
    hal_crc_init(&crc_cfg);
    uint32_t computed_crc = hal_crc_compute(data, len);

    v_mutex_unlock(crc_mutex);
    return computed_crc;
  }
  return 0;
}
