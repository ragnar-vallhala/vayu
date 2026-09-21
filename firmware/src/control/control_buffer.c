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
#include "control/control_buffer.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "structure.h"

static spsc_fifo_t _telemetry_queue;
static control_telemetry_t _telemetry_buffer[CONTROL_TELEMETRY_BUFFER_SIZE];

/** @noreq control-telemetry SPSC buffer init (infrastructure). */
void control_telemetry_buffer_init(void) {
  spsc_init(&_telemetry_queue, _telemetry_buffer, CONTROL_TELEMETRY_BUFFER_SIZE,
            sizeof(control_telemetry_t));
  spsc_set_policy(&_telemetry_queue, SPSC_POLICY_OVERWRITE);
}

/** @noreq perf-telemetry FIFO-row plumbing (glue). */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  if (max < 1)
    return 0;
  perf_fifo_fill_row(&rows[0], PERF_FIFO_TELEMETRY, &_telemetry_queue);
  return 1;
}

/** @noreq SPSC telemetry-queue push (infrastructure). */
bool control_telemetry_queue_push(const control_telemetry_t *data) {
  return spsc_write(&_telemetry_queue, data, 1) == 1;
}

/** @noreq SPSC telemetry-queue pop (infrastructure). */
bool control_telemetry_queue_pop(control_telemetry_t *out_data) {
  return spsc_read(&_telemetry_queue, out_data, 1) == 1;
}
