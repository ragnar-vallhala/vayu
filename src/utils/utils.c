#include "utils/utils.h"
#include "comm/serializer.h"
#include "navhal.h"
#include "structure.h"
#include "utils.h" // Kernels utils for vaprint_fmt_buf
#include "variables.h"
#include <stdarg.h>
#include <stdint.h>

static uint8_t first_log = 1;
mpmc_queue_t vayu_log_queue;

static uint8_t log_queue_buffer[VAYU_LOG_QUEUE_SIZE];
static char log_buf[128];

void vayu_log(const char *fmt, ...) {
  if (first_log) {

    mpmc_init(&vayu_log_queue, log_queue_buffer, VAYU_LOG_QUEUE_SIZE, sizeof(char));
    mpmc_set_policy(&vayu_log_queue, MPMC_POLICY_OVERWRITE);
    first_log = 0;
  }

  va_list args;
  va_start(args, fmt);
  int len = vaprint_fmt_buf(log_buf, sizeof(log_buf), fmt, args);
  va_end(args);

  if (len > 0) {
    mpmc_push_bulk(&vayu_log_queue, log_buf, len);
  }
}

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