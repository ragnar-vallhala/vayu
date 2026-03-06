#include "utils/utils.h"
#include "variables.h"
#include <stdint.h>

static volatile uint64_t _time_stamp_high_freq = 0;
void increment_high_freq_timer(void) { _time_stamp_high_freq++; }
static uint32_t _time_stamp_high_freq_offset = 0;
static uint8_t _device_id = 0;

uint64_t get_timestamp(void) { return _time_stamp_high_freq; }

uint32_t get_timestamp_unix(void) {
  return _time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000) +
         _time_stamp_high_freq_offset;
}

void set_timestamp(uint32_t timestamp) {
  _time_stamp_high_freq_offset =
      timestamp - _time_stamp_high_freq / (HIGH_FREQ_TIMER_FREQ / 1000);
}

uint8_t get_device_id(void) { return _device_id; }
void set_device_id(uint8_t device_id) { _device_id = device_id; }