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
/** @file sys/clock.c @brief Cycle-counter timebase (see sys/clock.h). */

#include "sys/clock.h"

#include "navhal.h" /* hal_clock_get_sysclk, hal_cycle_counter_get */
#include "sys/clock.h"

/* Seeded with the expected rate so a caller that runs before vayu_clock_init()
 * gets the old behaviour rather than a divide by zero. */
static uint32_t s_hz = (uint32_t)SYS_CLOCK_FREQ;

/** @noreq timebase seam: caches the live CPU rate. */
void vayu_clock_init(void) {
  uint32_t hz = hal_clock_get_sysclk();
  if (hz != 0u) {
    s_hz = hz;
  }
}

/** @noreq timebase seam. */
uint32_t vayu_clock_hz(void) { return s_hz; }

/** @noreq timebase seam. */
uint32_t vayu_clock_cycles(void) { return hal_cycle_counter_get(); }

/** @noreq timebase seam. */
float vayu_dt_from_cycles(uint32_t now_cyc, uint32_t prev_cyc) {
  uint32_t d = now_cyc - prev_cyc; /* wrap-safe */
  float dt = (float)d / (float)s_hz;
  if (dt < 1e-4f) {
    dt = 1e-4f;
  }
  if (dt > 0.1f) {
    dt = 0.1f;
  }
  return dt;
}
