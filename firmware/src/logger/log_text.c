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
#include "storage/imu_hs_log.h" /* imu_hs_log_wire_txt (blackbox text stream) */
#include "sys/clock.h"          /* vayu_clock_cycles */

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
    /* Same line to the card, as a byte stream in the blackbox. Teed here at
     * the producer rather than at the telemetry drain, because the drain only
     * runs at the telemetry cadence and only while a link is up -- and the
     * flight whose log you actually need is the one that ended with the link
     * already gone.
     *
     * It lands in the same ring, on the same cycle stamps, as the samples it
     * explains, which is what lets a reader put "[EST] degraded RAISED" next
     * to the estimator output that triggered it.
     *
     * Non-blocking and lossy under pressure -- vayu_log is called from the
     * control path and a text log is never worth stalling a caller for.
     * @implements LOG-PERSIST-001 */
    imu_hs_log_wire_txt((const uint8_t *)log_buf, (uint16_t)len,
                        vayu_clock_cycles());
  }
}
