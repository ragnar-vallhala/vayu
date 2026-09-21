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
/* flight_mode.c — stabilise/acro arbitration between RC and GCS. */
#include "control/flight_mode.h"

#include "utils.h" /* v_memcpy */

/* Sticky GCS override. When active it wins over the RC switch. */
static volatile bool s_override_active = false;
static volatile flight_mode_t s_override_mode = FLIGHT_MODE_ANGLE;

/* Last resolved effective state, published as telemetry. */
static volatile flight_mode_t s_effective = FLIGHT_MODE_ANGLE;
static volatile flight_mode_src_t s_source = FLIGHT_MODE_SRC_RC;

/** @implements SYS-CTRL-004 GCS flight-mode override setter. */
void flight_mode_set_override(flight_mode_t mode) {
  s_override_mode = mode;
  s_override_active = true;
}

/** @implements SYS-CTRL-004 clears the GCS flight-mode override. */
void flight_mode_release(void) { s_override_active = false; }

/** @implements SYS-CTRL-004 */
bool flight_mode_resolve_acro(bool rc_acro) {
  flight_mode_t mode;
  flight_mode_src_t src;
  if (s_override_active) {
    mode = s_override_mode;
    src = FLIGHT_MODE_SRC_GCS;
  } else {
    mode = rc_acro ? FLIGHT_MODE_ACRO : FLIGHT_MODE_ANGLE;
    src = FLIGHT_MODE_SRC_RC;
  }
  s_effective = mode;
  s_source = src;
  return mode == FLIGHT_MODE_ACRO;
}

/** @noreq trivial effective-mode accessor. */
flight_mode_t flight_mode_get(void) { return s_effective; }
/** @noreq trivial mode-source accessor. */
flight_mode_src_t flight_mode_get_source(void) { return s_source; }

/**
 * Parse a CMD_SET_FLIGHT_MODE payload, validating argc/length before reading
 * the float arg (COMM-CMD-002). It drives the sticky GCS mode override.
 *
 * @implements COMM-CMD-002, SYS-CTRL-004
 */
bool flight_mode_apply_command(const uint8_t *payload, uint8_t length) {
  /* [cmd_id:2][argc:1][arg0:f32] — need at least one float arg. */
  if (length < 7) {
    return false;
  }
  uint8_t argc = payload[2];
  if (argc < 1) {
    return false;
  }
  float v;
  v_memcpy(&v, &payload[3], 4);
  int iv = (int)(v + (v >= 0.0f ? 0.5f : -0.5f));
  switch (iv) {
  case FLIGHT_MODE_ANGLE:
    flight_mode_set_override(FLIGHT_MODE_ANGLE);
    break;
  case FLIGHT_MODE_ACRO:
    flight_mode_set_override(FLIGHT_MODE_ACRO);
    break;
  default: /* 2 (or anything else) = release back to the RC switch */
    flight_mode_release();
    break;
  }
  return true;
}
