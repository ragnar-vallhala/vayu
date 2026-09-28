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
/* See driver/indicator.h. The pin map is the board's; this file only knows
 * the order of the indicator_t enum. */
#include "driver/indicator.h"

#include "navhal.h"
#include "vayu_board.h"

static const hal_gpio_pin_t _pin[IND_COUNT] = {
    [IND_LED_BLUE] = BOARD_LED_BLUE,
    [IND_LED_GREEN] = BOARD_LED_GREEN,
    [IND_LED_RED] = BOARD_LED_RED,
    [IND_BUZZER] = BOARD_BUZZER,
};

/* Output pins do not read back reliably, so the truth lives here. */
static bool _on[IND_COUNT];

/** @implements SYS-HMI-001 */
void indicator_init(void) {
  for (int i = 0; i < IND_COUNT; i++) {
    hal_gpio_set_mode(_pin[i], HAL_GPIO_MODE_OUTPUT, HAL_GPIO_PULL_NONE);
    _on[i] = false;
    hal_gpio_write(_pin[i], HAL_GPIO_LOW);
  }
}

/** @noreq thin pin accessor */
void indicator_set(indicator_t which, bool on) {
  if ((unsigned)which >= IND_COUNT) {
    return;
  }
  _on[which] = on;
  hal_gpio_write(_pin[which], on ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
}

/** @noreq thin pin accessor */
void indicator_toggle(indicator_t which) {
  if ((unsigned)which >= IND_COUNT) {
    return;
  }
  indicator_set(which, !_on[which]);
}

/** @noreq thin pin accessor */
bool indicator_get(indicator_t which) {
  return (unsigned)which < IND_COUNT ? _on[which] : false;
}

/** @noreq thin pin accessor */
void indicator_all_off(void) {
  for (int i = 0; i < IND_COUNT; i++) {
    indicator_set((indicator_t)i, false);
  }
}
