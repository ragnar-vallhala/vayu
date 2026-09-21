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
#ifndef VAYU_RC_BUFFER_H
#define VAYU_RC_BUFFER_H

#include "comm/ibus.h"
#include "comm/perf_packet.h"
#include <stdbool.h>

/* Usable slots = SIZE - 1 (see imu_buffer.h for where the two lost slots go).
 * Control keeps 4: it is the failsafe path and 30 B/slot is not worth trimming.
 * Telemetry is a mailbox -- one pop per gate tick against a ~140 Hz producer --
 * so depth past 2 only ages the frame that gets read. */
#define RC_BUFFER_SIZE 5
#define RC_TELEMETRY_BUFFER_SIZE 3

void rc_buffer_init(void);

/* Fill perf wire rows for this module's SPSC fifos. Returns rows written. */
int rc_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

bool rc_queue_telemetry_push(const ibus_data_t *data);
bool rc_queue_telemetry_pop(ibus_data_t *out_data);

bool rc_queue_control_push(const ibus_data_t *data);
bool rc_queue_control_pop(ibus_data_t *out_data);

#endif // VAYU_RC_BUFFER_H
