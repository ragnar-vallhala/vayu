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
/* The ESC driver (firmware/src/driver/esc.c).
 *
 * Four ESCs share one advanced-control timer, and that sharing is the whole
 * risk. esc_init used to take the timer and call hal_pwm_init, which
 * reconfigures the WHOLE timer's prescaler and auto-reload -- four ESCs, four
 * re-inits, no owner (fault line F7). esc_disarm stopped that shared timer,
 * so disarming one motor would have cut the PWM to all four; nothing called
 * it, which is the only reason that never bit.
 *
 * So the property pinned here is INDEPENDENCE: anything done to one motor
 * must leave the other three exactly where they were. The host stubs at the
 * hal_pwm level, so the timer registers are not visible from here -- channel
 * independence is the same hazard expressed at the level this build can see.
 *
 * The duty arithmetic is checked too, because it decides how fast a prop
 * turns: a 1..2 ms pulse on a 2.5 ms period is duty 0.4..0.8. */
#include <math.h>
#include <stdio.h>

#include "driver/esc.h"
#include "host_pwm.h"
#include "vayu_board.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-4f)

static ESC_Handle esc[4];

static void bring_up(void) {
  esc_group_init(BOARD_ESC_TIMER, VAYU_ESC_PWM_FREQ);
  esc_init(&esc[0], 1, BOARD_ESC_M1_PIN, BOARD_ESC_AF);
  esc_init(&esc[1], 2, BOARD_ESC_M2_PIN, BOARD_ESC_AF);
  esc_init(&esc[2], 3, BOARD_ESC_M3_PIN, BOARD_ESC_AF);
  esc_init(&esc[3], 4, BOARD_ESC_M4_PIN, BOARD_ESC_AF);
}

/* Duty for a pulse width at the ESC's frequency: pulse / (1000 / freq). */
static float duty_for_ms(float ms) {
  return ms / (VAYU_ESC_MS_PER_SECOND / (float)VAYU_ESC_PWM_FREQ);
}

int main(void) {
  printf("== esc ==\n");

  printf("  [1] bring-up leaves every channel at the minimum pulse\n");
  {
    bring_up();
    float want = duty_for_ms(VAYU_ESC_MIN_PULSE_MS);
    for (unsigned ch = 1; ch <= 4; ch++) {
      CHECK(NEAR(host_pwm_duty(ch), want), "channel idles at min pulse");
    }
    /* Not min throttle -- MIN PULSE. A 0.4 duty here and a 0 duty would look
     * equally "off" on a scope trace but mean opposite things to an ESC. */
    CHECK(want > 0.39f && want < 0.41f, "min pulse is ~0.4 duty at 400 Hz");
  }

  printf("  [2] throttle maps across the 1..2 ms band\n");
  {
    esc_set_throttle(&esc[0], 0.0f);
    CHECK(NEAR(host_pwm_duty(1), duty_for_ms(VAYU_ESC_MIN_PULSE_MS)),
          "0 throttle is the min pulse");
    esc_set_throttle(&esc[0], 1.0f);
    CHECK(NEAR(host_pwm_duty(1), duty_for_ms(VAYU_ESC_MAX_PULSE_MS)),
          "full throttle is the max pulse");
    esc_set_throttle(&esc[0], 0.5f);
    CHECK(NEAR(host_pwm_duty(1),
               duty_for_ms((VAYU_ESC_MIN_PULSE_MS + VAYU_ESC_MAX_PULSE_MS) /
                           2.0f)),
          "half throttle is the middle of the band");
  }

  printf("  [3] out-of-range throttle clamps, it does not wrap\n");
  {
    esc_set_throttle(&esc[0], 5.0f);
    CHECK(NEAR(host_pwm_duty(1), duty_for_ms(VAYU_ESC_MAX_PULSE_MS)),
          "above 1 clamps to max");
    esc_set_throttle(&esc[0], -5.0f);
    CHECK(NEAR(host_pwm_duty(1), duty_for_ms(VAYU_ESC_MIN_PULSE_MS)),
          "below 0 clamps to min, not to some huge pulse");
  }

  printf("  [4] one motor's throttle does not move the others\n");
  {
    bring_up();
    esc_set_throttle(&esc[2], 0.75f); /* motor 3 */
    float idle = duty_for_ms(VAYU_ESC_MIN_PULSE_MS);
    CHECK(!NEAR(host_pwm_duty(3), idle), "motor 3 moved");
    CHECK(NEAR(host_pwm_duty(1), idle), "motor 1 did not");
    CHECK(NEAR(host_pwm_duty(2), idle), "motor 2 did not");
    CHECK(NEAR(host_pwm_duty(4), idle), "motor 4 did not");
  }

  printf("  [5] disarming one motor does not stop the other three\n");
  {
    /* The hazard F7 left behind: esc_disarm called hal_pwm_stop, which stops
     * the timer every channel shares. One motor's disarm would have silenced
     * the whole aircraft. */
    bring_up();
    for (int i = 0; i < 4; i++) {
      esc_arm(&esc[i]);
    }
    for (unsigned ch = 1; ch <= 4; ch++) {
      CHECK(host_pwm_started(ch), "all four armed");
    }

    esc_disarm(&esc[1]); /* motor 2 */
    CHECK(!host_pwm_started(2), "motor 2 is disarmed");
    CHECK(host_pwm_started(1), "motor 1 still running");
    CHECK(host_pwm_started(3), "motor 3 still running");
    CHECK(host_pwm_started(4), "motor 4 still running");

    /* And it is recoverable: re-arming the one channel brings it back. */
    esc_arm(&esc[1]);
    CHECK(host_pwm_started(2), "motor 2 re-arms");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
