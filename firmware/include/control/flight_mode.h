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
/* flight_mode.h — stabilise (angle) vs acro (rate) flight-mode arbitration.
 *
 * The active mode comes from one of two sources:
 *   - RC: the physical ACRO_SWITCH_CH channel (default behavior).
 *   - GCS: a CMD_SET_FLIGHT_MODE command, which sets a sticky override.
 *
 * When a GCS override is active it wins over the RC switch, until the GCS
 * releases it (arg 2). The effective mode + source are published back to the
 * GCS via SYSTEM_ORIGIN_FLIGHT_MODE telemetry so the UI reflects reality.
 */
#ifndef VAYU_FLIGHT_MODE_H
#define VAYU_FLIGHT_MODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  FLIGHT_MODE_ANGLE = 0, // stabilise / self-levelling, bank-angle limited
  FLIGHT_MODE_ACRO = 1,  // rate mode, sticks command body rate directly
} flight_mode_t;

typedef enum {
  FLIGHT_MODE_SRC_RC = 0,  // resolved from the RC switch
  FLIGHT_MODE_SRC_GCS = 1, // forced by a GCS override
} flight_mode_src_t;

/* GCS commands. set_override pins the mode; release hands control back to RC. */
void flight_mode_set_override(flight_mode_t mode);
void flight_mode_release(void);

/* Resolve the effective acro flag given the RC-derived acro request, recording
 * the effective mode + source for telemetry. Call once per outer-loop update. */
bool flight_mode_resolve_acro(bool rc_acro);

flight_mode_t flight_mode_get(void); // last resolved effective mode
flight_mode_src_t flight_mode_get_source(void);

/* Parse a CMD_SET_FLIGHT_MODE NavLink payload ([cmd:2][argc:1][arg0:f32]).
 * Returns true if applied. */
bool flight_mode_apply_command(const uint8_t *payload, uint8_t length);

#endif // VAYU_FLIGHT_MODE_H
