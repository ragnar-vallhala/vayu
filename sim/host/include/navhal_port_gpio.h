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
/* Host SITL stub for NavHAL cortex-m4 port_gpio.h.
 * Inline hot-path GPIO accessors that target STM32 BSRR / IDR don't apply
 * to the host build - degrade them to no-ops. */
#ifndef VAYU_SIM_NAVHAL_PORT_GPIO_H
#define VAYU_SIM_NAVHAL_PORT_GPIO_H

#include "common/hal_gpio.h"

static inline void hal_gpio_write(hal_gpio_pin_t pin, hal_gpio_state_t state) {
  (void)pin;
  (void)state;
}

static inline hal_gpio_state_t hal_gpio_read(hal_gpio_pin_t pin) {
  (void)pin;
  return HAL_GPIO_LOW;
}

static inline void hal_gpio_toggle(hal_gpio_pin_t pin) { (void)pin; }

#endif /* VAYU_SIM_NAVHAL_PORT_GPIO_H */
