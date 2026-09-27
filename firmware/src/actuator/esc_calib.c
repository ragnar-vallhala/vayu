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
#include "actuator/esc_calib.h"

#include "storage/fs_owner.h" /* vayu_log */
#include "sys/state.h"
#include "utils.h" /* v_get_ticks */
#include "vayu_status.h"

/* Stick layout, 0-indexed, as angle_controller.c reads them:
 *   [0] roll   (right stick horizontal)
 *   [1] pitch  (right stick vertical)
 *   [2] throttle (left stick vertical)
 *   [3] yaw    (left stick horizontal)
 *
 * Both gestures hold THROTTLE and PITCH at minimum -- both sticks pulled fully
 * down -- and differ only in which way the two horizontal axes point. That is
 * deliberate: the stick that could command thrust is at its stop throughout,
 * so performing or holding either gesture cannot ask for power.
 *
 *   ENTER   left stick down-RIGHT (yaw high), right stick down-LEFT  (roll low)
 *   CLOSE   left stick down-LEFT  (yaw low),  right stick down-RIGHT (roll high)
 *
 * They are mirror images, so neither can be reached by drifting out of the
 * other, and neither is a resting position. */
static bool gesture_enter(const ibus_data_t *rc) {
  return rc->channels[2] < ESC_CALIB_STICK_LO && /* throttle down */
         rc->channels[1] < ESC_CALIB_STICK_LO && /* pitch down    */
         rc->channels[3] > ESC_CALIB_STICK_HI && /* yaw right     */
         rc->channels[0] < ESC_CALIB_STICK_LO;   /* roll left     */
}

static bool gesture_close(const ibus_data_t *rc) {
  return rc->channels[2] < ESC_CALIB_STICK_LO && /* throttle down */
         rc->channels[1] < ESC_CALIB_STICK_LO && /* pitch down    */
         rc->channels[3] < ESC_CALIB_STICK_LO && /* yaw left      */
         rc->channels[0] > ESC_CALIB_STICK_HI;   /* roll right    */
}

typedef enum {
  ESC_CAL_OFF = 0,
  ESC_CAL_HIGH,   /* holding maximum; operator powers the ESCs */
  ESC_CAL_SETTLE, /* holding minimum after the closing gesture */
} esc_cal_phase_t;

/* RC-task-owned. motor_task reads s_phase through esc_calib_output(); one
 * writer, one word, so no lock. */
static volatile esc_cal_phase_t s_phase = ESC_CAL_OFF;
static uint32_t s_gesture_ms; /* when the gesture currently held began */
static uint32_t s_phase_ms;   /* when the current phase began          */
static bool s_holding_enter;  /* the enter gesture is being held       */
static bool s_holding_close;  /* the close gesture is being held       */

/** @noreq gesture hold timer: returns true once `held` has lasted long enough */
static bool held_long_enough(bool now, bool *was, uint32_t *since) {
  const uint32_t t = v_get_ticks();
  if (!now) {
    *was = false;
    return false;
  }
  if (!*was) {
    *was = true;
    *since = t;
    return false;
  }
  return (uint32_t)(t - *since) >= ESC_CALIB_GESTURE_MS;
}

/** @noreq leave calibration, motors first */
static void esc_calib_finish(const char *why) {
  s_phase = ESC_CAL_OFF;
  s_holding_enter = false;
  s_holding_close = false;
  /* Back to STANDBY only if we are still the state that owns the motors. A
   * watchdog may already have moved us to FAILSAFE, which is also safe and
   * must not be overwritten. */
  if (system_state_get() == SYSTEM_STATE_ESC_CALIB) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
  }
  vayu_log("esc_calib: %s", why);
}

/** @noreq per-frame service; owns entry to and exit from SYSTEM_STATE_ESC_CALIB */
void esc_calib_rc_step(const ibus_data_t *rc) {
  if (rc == NULL) {
    return;
  }
  const sys_state_t st = system_state_get();

  /* A failsafe frame is not a gesture. Substituted channel values are whatever
   * the safety layer chose, and reading a gesture out of them would start a
   * calibration from a lost link. */
  if (rc->is_failsafe) {
    if (s_phase != ESC_CAL_OFF) {
      esc_calib_finish("aborted: RC failsafe");
    }
    s_holding_enter = false;
    s_holding_close = false;
    return;
  }

  if (s_phase == ESC_CAL_OFF) {
    /* Entry only from STANDBY: disarmed, RC healthy, nothing else claiming the
     * motors. The transition table refuses it from anywhere else anyway, but
     * checking here keeps the gesture from being armed-and-waiting. */
    if (st != SYSTEM_STATE_STANDBY) {
      s_holding_enter = false;
      return;
    }
    if (!held_long_enough(gesture_enter(rc), &s_holding_enter, &s_gesture_ms)) {
      return;
    }
    if (system_state_set(SYSTEM_STATE_ESC_CALIB) != VAYU_OK) {
      s_holding_enter = false;
      return;
    }
    s_phase = ESC_CAL_HIGH;
    s_phase_ms = v_get_ticks();
    s_holding_enter = false;
    s_holding_close = false;
    vayu_log("esc_calib: MAX held -- power the ESCs now, props OFF");
    return;
  }

  /* Anything that took the state away from us ends it: the watchdog dropping to
   * FAILSAFE on RC loss is the case that matters, and the motors are already
   * zeroed by motor_task the moment the state changed. */
  if (st != SYSTEM_STATE_ESC_CALIB) {
    s_phase = ESC_CAL_OFF;
    s_holding_enter = false;
    s_holding_close = false;
    vayu_log("esc_calib: aborted: state left ESC_CALIB");
    return;
  }

  /* Backstop, whichever phase we are in. */
  if ((uint32_t)(v_get_ticks() - s_phase_ms) >= ESC_CALIB_TIMEOUT_MS) {
    esc_calib_finish("aborted: timed out");
    return;
  }

  if (s_phase == ESC_CAL_HIGH) {
    if (held_long_enough(gesture_close(rc), &s_holding_close, &s_gesture_ms)) {
      s_phase = ESC_CAL_SETTLE;
      s_phase_ms = v_get_ticks();
      s_holding_close = false;
      vayu_log("esc_calib: MIN held -- endpoints storing");
    }
    return;
  }

  /* ESC_CAL_SETTLE: hold minimum long enough for the ESC to store and beep. */
  if ((uint32_t)(v_get_ticks() - s_phase_ms) >= ESC_CALIB_SETTLE_MS) {
    esc_calib_finish("done");
  }
}

/** @noreq the output every motor gets during calibration */
float esc_calib_output(void) { return (s_phase == ESC_CAL_HIGH) ? 1.0f : 0.0f; }

/** @noreq state predicate */
bool esc_calib_active(void) { return s_phase != ESC_CAL_OFF; }
