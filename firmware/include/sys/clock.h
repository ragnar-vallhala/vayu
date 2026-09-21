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
 * @file sys/clock.h
 * @brief Cycle-counter timebase, decoupled from the HAL.
 *
 * The one place the CPU cycle rate is known, and the only cycle-counter API the
 * logic layers (control, est, maths) may use — they must not name `hal_*`.
 *
 * The rate is read from the live clock tree at boot rather than assumed, so a
 * PLL that does not land on SYS_CLOCK_FREQ scales dt correctly instead of
 * silently mis-scaling every gyro integral, derivative and EKF predict.
 * SYS_CLOCK_FREQ remains the *expected* value that boot.c checks the clock
 * against; it is not what the maths divides by.
 */
#ifndef VAYU_SYS_CLOCK_H
#define VAYU_SYS_CLOCK_H

#include <stdint.h>

/**
 * Cache the live CPU (== DWT) rate.
 *
 * Call once from main() after the clock tree and the cycle counter are up.
 * Until it runs, vayu_clock_hz() reports SYS_CLOCK_FREQ.
 */
void vayu_clock_init(void);

/** CPU/DWT rate in Hz as measured at vayu_clock_init(). Never 0. */
uint32_t vayu_clock_hz(void);

/** Current cycle stamp. Wraps every 2^32 cycles (~51 s at 84 MHz). */
uint32_t vayu_clock_cycles(void);

/**
 * Seconds between two cycle stamps (wrap-safe unsigned delta), clamped.
 *
 * IMU samples and attitude estimates carry their acquisition cycle stamp
 * (bmx160_all_converted_reading_t.timestamp, attitude_t.timestamp); the control
 * loops and the fusion step derive dt from deltas of those instead of reading
 * the counter at execution time, so dt is the true inter-sample interval,
 * immune to scheduler jitter. The floor prevents a div-by-0 in the PID
 * derivative on a duplicate or first stamp; the ceiling bounds a wrap or stall.
 */
float vayu_dt_from_cycles(uint32_t now_cyc, uint32_t prev_cyc);

#endif /* VAYU_SYS_CLOCK_H */
