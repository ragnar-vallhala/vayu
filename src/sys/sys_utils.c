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
static uint32_t _time_stamp_high_freq_offset = 0;
static uint8_t _device_id = 0;

uint64_t get_timestamp(void) { return _time_stamp_high_freq; }

uint32_t get_timestamp_unix(void) {
  return (uint32_t)(_time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000)) +
         _time_stamp_high_freq_offset;
}

void set_timestamp(uint32_t timestamp) {
  _time_stamp_high_freq_offset =
      timestamp -
      (uint32_t)(_time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000));
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
