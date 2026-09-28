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
#include "control/control_buffer.h"
#include <stdbool.h>

/* Mailbox, not a queue: the control loop pushes at loop rate and the telemetry
 * task pops at most one per gate tick. 4 => 2 usable slots (see imu_buffer.h).
 * Was 10, which sat permanently full and only served stale samples. */
#define CONTROL_TELEMETRY_BUFFER_SIZE 4

void control_telemetry_buffer_init(void);
/* One frame of the cascade's working state, for telemetry and the blackbox.
 * Declared beside its queue rather than in the old variables.h grab-bag. */
typedef struct __attribute__((packed)) {
  float roll_angle_sp;
  float pitch_angle_sp;
  float yaw_angle_sp;
  float roll_angle_curr;
  float pitch_angle_curr;
  float yaw_angle_curr;
  float roll_rate_sp;
  float pitch_rate_sp;
  float yaw_rate_sp;
  float roll_rate_curr;
  float pitch_rate_curr;
  float yaw_rate_curr;
  float roll_out;
  float pitch_out;
  float yaw_out;
  float thro_out;
  float outer_dt;
  float inner_dt;
} control_telemetry_t;

bool control_telemetry_queue_push(const control_telemetry_t *data);
bool control_telemetry_queue_pop(control_telemetry_t *out_data);

/* Fill perf wire rows for this module's SPSC fifos. Returns rows written. */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

#endif // VAYU_CONTROL_BUFFER_H
