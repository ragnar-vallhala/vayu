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
/* The annunciator driver (firmware/src/driver/indicator.c).
 *
 * On a headless flight controller the three LEDs and the buzzer are the only
 * diagnostic the pilot has standing next to the craft, so "the LED lies" is a
 * real failure. It lied twice before: an output pin cannot be read back, so
 * the state is tracked in software, and two modules used to drive the blue
 * LED from separate timers (fault line F5).
 *
 * The property worth pinning is therefore not "set works" but that the TRACKED
 * state and the PIN AGREE after every operation -- host_navhal.c's recording
 * stub is what makes that checkable. */
#include <stdio.h>

#include "driver/indicator.h"
#include "host_gpio.h"
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

/* The board pin behind each indicator, in indicator_t order. */
static const unsigned pin[IND_COUNT] = {BOARD_LED_BLUE, BOARD_LED_GREEN,
                                        BOARD_LED_RED, BOARD_BUZZER};

/* The invariant: what the driver says it is, is what the pin is driving. */
static int agrees(indicator_t w) {
  return (host_gpio_level[pin[w]] != 0) == (indicator_get(w) != 0);
}

int main(void) {
  printf("== indicator ==\n");
  indicator_init();

  printf("  [1] init leaves every indicator off, buzzer included\n");
  for (int i = 0; i < IND_COUNT; i++) {
    CHECK(!indicator_get((indicator_t)i), "starts off");
    CHECK(agrees((indicator_t)i), "pin agrees after init");
  }

  printf("  [2] set drives the pin and is readable back\n");
  indicator_set(IND_LED_RED, true);
  CHECK(indicator_get(IND_LED_RED), "red reads on");
  CHECK(host_gpio_level[pin[IND_LED_RED]] == 1, "red pin is high");
  CHECK(!indicator_get(IND_LED_BLUE), "set is not a broadcast");

  printf("  [3] toggle tracks, because the pin cannot be read back\n");
  {
    /* The old code kept this state in heartbeat.c while a second writer drove
     * the same pin, so the two drifted apart and a toggle could be a no-op.
     * Twenty of them must leave it exactly where it started. */
    indicator_set(IND_LED_BLUE, false);
    for (int i = 0; i < 20; i++) {
      indicator_toggle(IND_LED_BLUE);
      CHECK(agrees(IND_LED_BLUE), "pin agrees after every toggle");
    }
    CHECK(!indicator_get(IND_LED_BLUE), "an even number of toggles is a no-op");
    indicator_toggle(IND_LED_BLUE);
    CHECK(indicator_get(IND_LED_BLUE), "an odd one is not");
  }

  printf("  [4] an out-of-range index is ignored, not written\n");
  {
    uint8_t before = host_gpio_level[pin[IND_LED_RED]];
    indicator_set((indicator_t)IND_COUNT, true);
    indicator_set((indicator_t)-1, true);
    indicator_toggle((indicator_t)IND_COUNT);
    CHECK(!indicator_get((indicator_t)IND_COUNT), "out of range reads false");
    CHECK(host_gpio_level[pin[IND_LED_RED]] == before,
          "a bad index did not scribble on a real pin");
  }

  printf("  [5] all_off clears everything, buzzer included\n");
  {
    /* This is the state-transition clear and the disarm path. A buzzer left
     * on because all_off only covered the LEDs is the loudest possible bug. */
    for (int i = 0; i < IND_COUNT; i++) {
      indicator_set((indicator_t)i, true);
    }
    indicator_all_off();
    for (int i = 0; i < IND_COUNT; i++) {
      CHECK(!indicator_get((indicator_t)i), "off");
      CHECK(host_gpio_level[pin[i]] == 0, "pin driven low");
    }
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
