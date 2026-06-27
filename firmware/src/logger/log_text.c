/**
 * @file src/logger/log_text.c
 * @brief Text-log producer: vayu_log() and its lock-free queue.
 *
 * @implements LOG-TXT-001
 *
 * The text-logging concern of the LOG module. The telemetry task drains
 * vayu_log_queue to the LOG channel (LOG-TXT-002). Declarations live in
 * utils/utils.h.
 */
#include "storage/fs_owner.h"

#include "ipc.h"
#include "structure.h"
#include "utils.h" /* vaios vaprint_fmt_buf */
#include "variables.h"

#include <stdarg.h>
#include <stdint.h>

static uint8_t first_log = 1;
mpmc_queue_t vayu_log_queue;

static uint8_t log_queue_buffer[VAYU_LOG_QUEUE_SIZE];
static char log_buf[128];

/**
 * @implements LOG-TXT-001
 */
void vayu_log(const char *fmt, ...) {
  if (first_log) {
    mpmc_init(&vayu_log_queue, log_queue_buffer, VAYU_LOG_QUEUE_SIZE,
              sizeof(char));
    mpmc_set_policy(&vayu_log_queue, MPMC_POLICY_OVERWRITE);
    first_log = 0;
  }

  va_list args;
  va_start(args, fmt);
  int len = vaprint_fmt_buf(log_buf, sizeof(log_buf), fmt, args);
  va_end(args);

  if (len > 0) {
    mpmc_push_bulk(&vayu_log_queue, log_buf, (uint8_t)len);
  }
}
