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
#ifndef VAYU_TIMER_CALLBACKS_H
#define VAYU_TIMER_CALLBACKS_H

#include "navhal.h"
#include <stdint.h>

// TIM5 is used for timer callbacks
/**
 * @brief Initialize the high-frequency timer callback system.
 * @param freq_hz Desired execution frequency in Hertz.
 *
 * @implements SYS-TIM-005
 */
void timer_callback_init(uint32_t freq_hz);
uint32_t timer_get_callback_frequency(void);

/**
 * @brief Register a function to be called at high frequency.
 * @param fn Function pointer to register.
 * @return 0 on success, -1 if array is full (capacity MAX_TIMER_CALLBACKS = 4).
 *
 * @implements SYS-TIM-005
 */
int timer_callback_register(void (*fn)(void), uint32_t us_delay);

#endif // !VAYU_TIMER_CALLBACKS_H
