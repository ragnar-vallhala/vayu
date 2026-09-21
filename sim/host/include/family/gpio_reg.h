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
/* Host SITL stub for NavHAL's family/gpio_reg.h (cortex-m4 STM32 register
 * file). The host build never dereferences GPIO registers, so this only
 * needs to satisfy the #include. */
#ifndef VAYU_SIM_GPIO_REG_H
#define VAYU_SIM_GPIO_REG_H

#include <stdint.h>

typedef struct {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIO_GET_PORT(pin) ((GPIO_TypeDef *)0)
#define GPIO_GET_PIN(pin) ((uint32_t)(pin) & 0xF)

#endif /* VAYU_SIM_GPIO_REG_H */
