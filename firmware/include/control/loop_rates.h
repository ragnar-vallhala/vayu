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
 * @file control/loop_rates.h
 * @brief How fast each loop runs, and the timestep it integrates with.
 *
 * Separate from the gains in control/tuning.h because these are a scheduling
 * contract, not a tune: change one and the sensor path, the rate loop, the
 * angle loop and every dt in the cascade move together. A gain change is
 * local to one axis; a rate change is not.
 */
#ifndef VAYU_CONTROL_LOOP_RATES_H
#define VAYU_CONTROL_LOOP_RATES_H

/* Control-loop rates. The inner (rate) loop is hard-pinned to INNER_LOOP_FREQ_HZ
 * with a drift-free periodic wait (task_delay_until); the outer (angle) loop
 * runs at inner / OUTER_LOOP_DECIM. Each loop reads the freshest IMU sample /
 * attitude (the producers run at sensor rate and the OVERWRITE rings keep the
 * latest) and integrates with a CONSTANT dt = 1/freq, so the PID sees a fixed
 * timestep regardless of execution jitter or sensor-rate variation.
 *
 * NOTE: with the 1 ms SysTick the loop period is quantised to whole ms, so
 * INNER_LOOP_FREQ_HZ is effectively capped at 1000 and should divide 1000
 * (1000, 500, 250, ...). Override INNER_LOOP_FREQ_HZ before this header to
 * retune. */
#ifndef INNER_LOOP_FREQ_HZ
#define INNER_LOOP_FREQ_HZ 1000 /* rate (inner) loop, Hz */
#endif

/* IMU acquisition rate. The accel/gyro (FAST) reads are paced to this off the
 * HIGH_FREQ_TIMER (the 1 ms SysTick can't time sub-ms periods), decoupling the
 * sensor rate from the I2C free-run speed (~2.8 kHz). Default oversamples the
 * 1 kHz control loop 2x for gyro anti-aliasing / fusion fidelity while freeing
 * the CPU the extra ~0.8 kHz of per-sample work was burning. Must be <= the
 * I2C-bound ceiling (~2.8 kHz) and divide cleanly into 1e6 us. */
#ifndef IMU_SAMPLE_FREQ_HZ
#define IMU_SAMPLE_FREQ_HZ 2000
#endif
#define IMU_FAST_PERIOD_US (1000000u / IMU_SAMPLE_FREQ_HZ)
#define OUTER_LOOP_DECIM 4 /* outer = inner / OUTER_LOOP_DECIM */
#define OUTER_LOOP_FREQ_HZ (INNER_LOOP_FREQ_HZ / OUTER_LOOP_DECIM)

/* Periods in SysTick ticks (1 tick = SYSTICK_PERIOD us = 1 ms by default). */
#define INNER_LOOP_PERIOD_TICKS MS_TO_TICKS(1000 / INNER_LOOP_FREQ_HZ)
#define OUTER_LOOP_PERIOD_TICKS (INNER_LOOP_PERIOD_TICKS * OUTER_LOOP_DECIM)

/* Constant integration timesteps (seconds) derived from the pinned rates. */
#define INNER_LOOP_DT (1.0f / (float)INNER_LOOP_FREQ_HZ)
#define OUTER_LOOP_DT ((float)OUTER_LOOP_DECIM / (float)INNER_LOOP_FREQ_HZ)

#endif // VAYU_CONTROL_LOOP_RATES_H
