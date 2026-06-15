/**
 * @file src/sys/sys_utils.c
 * @brief System utilities — timestamp, device id, CRC32.
 *
 * Moved here from src/utils/utils.c per Phase 4 R2.6 (the text-logging
 * half went to src/logger/log_text.c).
 */
#include "sys/sys_utils.h"

#include "ipc.h"
#include "navhal.h" /* hal_crc_* */
#include "variables.h"
#include <stdint.h>

static volatile uint64_t _time_stamp_high_freq = 0;
void increment_high_freq_timer(void) { _time_stamp_high_freq++; }
static uint8_t _device_id = 0;

/* Disciplined clock offset (ms). _offset_target is where the offset should be;
 * _offset_applied is slewed toward it each discipline tick so the unix clock
 * changes smoothly. double (not float) so the offset is exact to the ms. */
static volatile double _offset_applied = 0.0;
static volatile int32_t _offset_target = 0;
static uint64_t _last_discipline_ticks = 0;

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

uint64_t get_timestamp(void) { return _time_stamp_high_freq; }

/* Raw monotonic ms from the HF tick counter (no offset). */
static uint32_t timestamp_raw_ms(void) {
  return (uint32_t)(_time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000));
}

uint32_t get_timestamp_unix(void) {
  return timestamp_raw_ms() + (uint32_t)(int32_t)_offset_applied;
}

/* Apply a GCS clock correction — the measured FC-GCS error to remove. Large
 * errors step immediately (cold start / resync / post-reboot); small residuals
 * slew. Either way the new applied offset converges to GCS time and a
 * disciplined clock reports correction ~= 0 (windup-free). */
void time_sync_set_offset(int32_t correction_ms) {
  if (correction_ms > TIME_SYNC_STEP_MS || correction_ms < -TIME_SYNC_STEP_MS) {
    _offset_applied += (double)correction_ms;   /* step */
    _offset_target = (int32_t)_offset_applied;  /* park the slew target */
  } else {
    _offset_target = (int32_t)_offset_applied + correction_ms; /* slew */
  }
}

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

/* Legacy one-shot setter kept as a thin wrapper (absolute jam). The heartbeat
 * path no longer calls this; the time-sync handshake drives discipline instead. */
void set_timestamp(uint32_t timestamp) {
  _offset_target = (int32_t)(timestamp - timestamp_raw_ms());
  _offset_applied = (double)_offset_target;
}

uint8_t get_device_id(void) { return _device_id; }
void set_device_id(uint8_t device_id) { _device_id = device_id; }

static MutexHandle_t crc_mutex = NULL;

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
