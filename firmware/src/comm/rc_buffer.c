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
#include "comm/rc_buffer.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "structure.h"

#define RC_BUFFER_INTERNAL_CAPACITY (RC_BUFFER_SIZE + 1)
#define RC_TELEMETRY_INTERNAL_CAPACITY (RC_TELEMETRY_BUFFER_SIZE + 1)

static ibus_data_t _rc_telemetry_buffer[RC_TELEMETRY_INTERNAL_CAPACITY];
static ibus_data_t _rc_control_buffer[RC_BUFFER_INTERNAL_CAPACITY];

static spsc_fifo_t _rc_telemetry_queue;
static spsc_fifo_t _rc_control_queue;

/** @noreq SPSC RC queue construction (OVERWRITE policy) */
void rc_buffer_init(void) {
  spsc_init(&_rc_telemetry_queue, _rc_telemetry_buffer,
            RC_TELEMETRY_INTERNAL_CAPACITY, sizeof(ibus_data_t));
  spsc_set_policy(&_rc_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_rc_control_queue, _rc_control_buffer, RC_BUFFER_INTERNAL_CAPACITY,
            sizeof(ibus_data_t));
  spsc_set_policy(&_rc_control_queue, SPSC_POLICY_OVERWRITE);
}

/** @noreq perf-row gather for the RC FIFOs (observability glue) */
int rc_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  int n = 0;
  if (n < max)
    perf_fifo_fill_row(&rows[n++], PERF_FIFO_RC_TELEMETRY,
                       &_rc_telemetry_queue);
  if (n < max)
    perf_fifo_fill_row(&rows[n++], PERF_FIFO_RC_CONTROL, &_rc_control_queue);
  return n;
}

/** @noreq trivial SPSC accessor */
bool rc_queue_telemetry_push(const ibus_data_t *data) {
  return spsc_write(&_rc_telemetry_queue, data, 1) == 1;
}

/** @noreq trivial SPSC accessor */
bool rc_queue_telemetry_pop(ibus_data_t *out_data) {
  return spsc_read(&_rc_telemetry_queue, out_data, 1) == 1;
}

/** @noreq trivial SPSC accessor */
bool rc_queue_control_push(const ibus_data_t *data) {
  return spsc_write(&_rc_control_queue, data, 1) == 1;
}

/** @noreq trivial SPSC accessor */
bool rc_queue_control_pop(ibus_data_t *out_data) {
  return spsc_read(&_rc_control_queue, out_data, 1) == 1;
}
