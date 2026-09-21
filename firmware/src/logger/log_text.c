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
