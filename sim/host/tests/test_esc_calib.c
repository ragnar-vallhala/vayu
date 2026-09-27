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
/* ESC endpoint calibration (firmware/src/actuator/esc_calib.c).
 *
 * This is the only procedure that drives every motor to 100% from a disarmed
 * aircraft, so what is worth pinning is not that it works but that it CANNOT
 * start by accident. Every check below is a way in that must stay shut:
 *
 *   - a gesture swept through, not held
 *   - a gesture held from any state other than STANDBY
 *   - a gesture read out of a failsafe-substituted frame
 *   - arming during calibration
 *
 * Plus the two that make it useful: the real gesture does start it, and the
 * closing gesture ends it with the output back at minimum.
 */
#include <stdio.h>
#include <string.h>

#include "actuator/esc_calib.h"
#include "sys/state.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

/* Sticks: [0] roll, [1] pitch, [2] throttle, [3] yaw. Both gestures hold
 * throttle and pitch down; they differ in yaw/roll direction. */
static ibus_data_t frame_neutral(void) {
  ibus_data_t r;
  memset(&r, 0, sizeof r);
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++)
    r.channels[i] = 1500;
  r.channels[2] = 1000; /* throttle down */
  return r;
}

static ibus_data_t frame_enter(void) {
  ibus_data_t r = frame_neutral();
  r.channels[1] = 1000; /* pitch down  */
  r.channels[3] = 2000; /* yaw right   */
  r.channels[0] = 1000; /* roll left   */
  return r;
}

static ibus_data_t frame_close(void) {
  ibus_data_t r = frame_neutral();
  r.channels[1] = 1000; /* pitch down  */
  r.channels[3] = 1000; /* yaw left    */
  r.channels[0] = 2000; /* roll right  */
  return r;
}

/* The gesture must be HELD, and the hold is measured on the RTOS tick, so a
 * host test has to let real time pass. Feed frames for a little over the
 * required hold at the rate the RC task runs. */
static void hold(const ibus_data_t *f, unsigned ms) {
  extern void v_delay(uint32_t);
  const unsigned step = 20; /* 50 Hz, as rc_ibus_task */
  for (unsigned t = 0; t < ms; t += step) {
    esc_calib_rc_step(f);
    v_delay(step);
  }
}

int main(void) {
  printf("== esc_calib ==\n");

  /* Start disarmed with a healthy link, which is the only way in. */
  _system_current_status = SYSTEM_STATE_STANDBY;

  printf("  [1] a gesture swept through does not start it\n");
  {
    ibus_data_t f = frame_enter();
    esc_calib_rc_step(&f); /* one frame only */
    ibus_data_t n = frame_neutral();
    esc_calib_rc_step(&n);
    CHECK(!esc_calib_active(), "one frame of the gesture is not a hold");
    CHECK(system_state_get() == SYSTEM_STATE_STANDBY, "still STANDBY");
  }

  printf("  [2] a failsafe frame is never read as a gesture\n");
  {
    ibus_data_t f = frame_enter();
    f.is_failsafe = true;
    hold(&f, ESC_CALIB_GESTURE_MS + 400u);
    CHECK(!esc_calib_active(), "held gesture on a failsafe frame is ignored");
    CHECK(system_state_get() == SYSTEM_STATE_STANDBY, "still STANDBY");
  }

  printf("  [3] it cannot start from ARMED\n");
  {
    _system_current_status = SYSTEM_STATE_ARMED;
    ibus_data_t f = frame_enter();
    hold(&f, ESC_CALIB_GESTURE_MS + 400u);
    CHECK(!esc_calib_active(), "the gesture is inert while ARMED");
    CHECK(system_state_get() == SYSTEM_STATE_ARMED, "state untouched");
    _system_current_status = SYSTEM_STATE_STANDBY;
  }

  printf("  [4] entry cuts the signal FIRST, and only then shows maximum\n");
  {
    ibus_data_t f = frame_enter();
    hold(&f, ESC_CALIB_GESTURE_MS + 400u);
    CHECK(esc_calib_active(), "held gesture starts calibration");
    CHECK(system_state_get() == SYSTEM_STATE_ESC_CALIB, "state is ESC_CALIB");
    /* The whole point: a powered ESC handed maximum while running just spins
     * the motor up. Entry must stop the outputs first so it shuts down. */
    CHECK(esc_calib_signal_off(), "the PWM signal is cut on entry");
    CHECK(esc_calib_output() == 0.0f,
          "and nothing is commanded while it is cut");

    ibus_data_t n = frame_neutral();
    hold(&n, ESC_CALIB_SIGNAL_CUT_MS + 400u);
    CHECK(!esc_calib_signal_off(), "the signal returns after the cut window");
    CHECK(esc_calib_output() == 1.0f, "then the MAXIMUM endpoint is shown");
  }

  printf("  [5] arming is refused while it runs\n");
  {
    CHECK(system_state_set(SYSTEM_STATE_ARMED) != VAYU_OK,
          "ESC_CALIB -> ARMED is not an allowed transition");
    CHECK(system_state_get() == SYSTEM_STATE_ESC_CALIB, "still ESC_CALIB");
  }

  printf("  [6] the closing gesture ends it, at minimum\n");
  {
    ibus_data_t f = frame_close();
    hold(&f, ESC_CALIB_GESTURE_MS + 400u);
    CHECK(esc_calib_output() == 0.0f, "output drops to the MINIMUM endpoint");
    /* Still settling: the endpoint has to be held while the ESC stores it. */
    CHECK(esc_calib_active(), "it holds minimum through the settle window");
    hold(&f, ESC_CALIB_SETTLE_MS + 400u);
    CHECK(!esc_calib_active(), "then it finishes");
    CHECK(system_state_get() == SYSTEM_STATE_STANDBY, "and returns to STANDBY");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
