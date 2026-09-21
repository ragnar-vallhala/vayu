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
#ifndef VAYU_TASKS_H
#define VAYU_TASKS_H

#include "comm/comm_types.h"

void boot_task(void *args);
void heartbeat_task(void *args);
void comm_processor_task(void *args);
/* Dispatch one received packet (heartbeat / command) to its handler.
 * Exposed for SITL so the GCS -> FC command path is unit-testable. */
void comm_processor_dispatch(const packet_t *pkt);
void flush_task(void *args);
void imu_telemetry_task(void *args);
void rc_ibus_task(void *args);
void motor_task(void *args);
void calibration_task(void *args);
/* Attitude estimation (fusion) — consumes timestamped IMU samples, publishes
 * timestamped attitude. Split out of the IMU driver. */
void attitude_task(void *args);
/* Vertical estimator (VERT) — sibling of the attitude task. Drains the
 * synchronized {q, accel, dt} input, fuses baro altitude, publishes the
 * fused {altitude, climb_rate, vertical_accel} for control / IN_AIR / telemetry. */
void vertical_estimator_task(void *args);
/* Periodic kernel/observability reporter (FC -> GCS, PACKET_TYPE_PERF_STATS). */
void perf_telemetry_task(void *args);
/* Centralised filesystem owner — sole runtime SD/VFS writer (blackbox logger +
 * PID/calib persistence). Declared in full in storage/fs_owner.h. */
void fs_owner_task(void *args);
/* Bulk-transfer (FTP) substrate service task — runs the xfer state machine off
 * the comm + control tasks: deferred provider->open, paced XFER_DATA emission,
 * periodic acks, timeouts. Declared in full in comm/xfer/navlink_xfer.h. */
void xfer_service_task(void *args);

#endif // !VAYU_TASKS_H
