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
#ifndef VAYU_SIM_HOST_IMU_FEEDER_H
#define VAYU_SIM_HOST_IMU_FEEDER_H

/* Spawn the IMU-feeder pthread.
 * Reads bmx160_all_converted_reading_t frames from /tmp/vayu_imu.fifo
 * (gz_imu_to_vayu.py writes them), pushes them into imu_queue_*, runs
 * the mahony filter and pushes the resulting attitude into
 * attitude_queue_*. */
void host_imu_feeder_start(void);

/* Single-shot pump for the RTOS cooperative stepper (Phase 4). open() returns
 * the IMU FIFO fd (<0 on failure); pump() blocks for one framed IMU sample and
 * injects it into the firmware queues, returning 1 on success / 0 on EOF. */
int host_imu_feeder_open(void);
int host_imu_feeder_pump(void);

#endif
