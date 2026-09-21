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
#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include "comm/perf_packet.h"
#include "variables.h"
#include <stdbool.h>

/* Mailbox, not a queue: the control loop pushes at loop rate and the telemetry
 * task pops at most one per gate tick. 4 => 2 usable slots (see imu_buffer.h).
 * Was 10, which sat permanently full and only served stale samples. */
#define CONTROL_TELEMETRY_BUFFER_SIZE 4

void control_telemetry_buffer_init(void);
bool control_telemetry_queue_push(const control_telemetry_t *data);
bool control_telemetry_queue_pop(control_telemetry_t *out_data);

/* Fill perf wire rows for this module's SPSC fifos. Returns rows written. */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

#endif // VAYU_CONTROL_BUFFER_H
