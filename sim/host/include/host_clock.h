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
/*
 * host_clock.h -- the SITL virtual sim clock + a wall-clock escape hatch.
 *
 * Phase 1 of the lockstep-sim plan (firmware/docs/plans/sitl-lockstep-sim.md). The
 * firmware's vaios delay primitives (v_delay / task_delay / task_delay_until /
 * v_get_ticks, implemented in host_vaios.c) now block on / read THIS clock
 * instead of CLOCK_MONOTONIC. The clock advances only when the IMU feeder
 * delivers a sample (one fixed step per sample), so firmware timing is slaved
 * to SIM time, not wall time — the basis for running faster than realtime.
 *
 * Layer split (see the plan's delay-caller audit):
 *   - Firmware RTOS tasks  -> virtual clock (the v_delay family).
 *   - Host INFRASTRUCTURE  -> host_wall_delay_ms() (process-alive idle loops,
 *                             serial/pty I/O backoff). These wait on external
 *                             wall-clock events, NOT sim time, so they must
 *                             never block on the virtual clock (it would
 *                             deadlock them when the sim is idle/paused).
 */
#ifndef HOST_CLOCK_H
#define HOST_CLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Advance virtual sim time by `us` microseconds and wake every task blocked in
 * a virtual delay. Called by the IMU feeder once per delivered sample. */
void host_clock_advance_us(uint64_t us);

/* Mark the clock as externally driven (the IMU feeder advances it). The live
 * stack calls this at startup. When DRIVEN, v_delay() blocks until time
 * advances (lockstep). When NOT driven — e.g. unit tests with no feeder —
 * v_delay() self-advances the clock and returns immediately, so the firmware's
 * v_get_ticks() still moves forward deterministically without any wall sleep. */
void host_clock_set_driven(int driven);

/* Current virtual sim time, microseconds since process start (starts at 0). */
uint64_t host_clock_now_us(void);

/* Wake all virtual-delay waiters and keep them awake (shutdown): subsequent
 * v_delay() calls return immediately so detached firmware tasks can unwind. */
void host_clock_stop(void);

/* Wall-clock delay (raw nanosleep), for host infrastructure that must pace on
 * real time regardless of sim speed. NOT slaved to the virtual clock. */
void host_wall_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HOST_CLOCK_H */
