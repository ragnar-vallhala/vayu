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
/* What the host's hal_gpio_write stub last drove onto each pin.
 *
 * Real hardware cannot read an output pin back -- that is why
 * driver/indicator.c tracks LED state in software. This is how a host test
 * checks that the tracked state and the pin actually agree. */
#ifndef HOST_GPIO_H
#define HOST_GPIO_H

#include <stdint.h>

#define HOST_GPIO_PIN_MAX 256

/** 1 = last driven HIGH, 0 = LOW (or never written). Indexed by pin. */
extern uint8_t host_gpio_level[HOST_GPIO_PIN_MAX];

#endif // HOST_GPIO_H
