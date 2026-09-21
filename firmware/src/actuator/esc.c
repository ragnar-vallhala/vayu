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
/**
 * @file esc.c
 * @brief Implementation of the ESC driver using PWM.
 *
 * @copyright © NAVROBOTEC PVT. LTD.
 */

#include "actuator/actuator.h"
#include "navhal.h"
#include <stdint.h>

#define DEFAULT_MIN_PULSE_MS 1.0f /**< 1ms for min throttle */
#define DEFAULT_MAX_PULSE_MS 2.0f /**< 2ms for max throttle */
#define DEFAULT_PWM_FREQ 400      /**< Typical 400Hz frequency for ESCs */
#define MS_PER_SECOND 1000.0f     /**< ms<->Hz period conversion (R10.3) */

void esc_init(ESC_Handle *esc, hal_timer_t timer, uint32_t channel,
              hal_gpio_pin_t pin) {
  esc->min_pulse_ms = DEFAULT_MIN_PULSE_MS;
  esc->max_pulse_ms = DEFAULT_MAX_PULSE_MS;
  esc->frequency = DEFAULT_PWM_FREQ;

  esc->pwm.timer = timer;
  esc->pwm.channel = channel;

  // Configure GPIO pin for PWM (Alternate Function)
  hal_gpio_enable_clock(pin);
  hal_gpio_set_mode(pin, HAL_GPIO_MODE_AF, HAL_GPIO_PULL_NONE);
  hal_gpio_set_output_speed(pin, HAL_GPIO_SPEED_VERY_HIGH);

  // For TIM1-TIM5, AF1 is usually the timer AF.
  // TIM1 and TIM2 use AF1. TIM3,4,5 use AF2.
  if (timer == TIM1 || timer == TIM2) {
    hal_gpio_set_alternate_function(pin, HAL_GPIO_AF1);
  } else if (timer == TIM3 || timer == TIM4 || timer == TIM5) {
    hal_gpio_set_alternate_function(pin, HAL_GPIO_AF2);
  }

  // Initialize PWM at the required frequency with 0 throttle (min pulse)
  // At 400Hz, the period is 2.5ms.
  // 1ms is 1.0/2.5 = 0.4 fraction duty cycle.
  float min_duty =
      (esc->min_pulse_ms / (MS_PER_SECOND / (float)esc->frequency));
  hal_pwm_init(&esc->pwm, esc->frequency, min_duty);
}

void esc_arm(ESC_Handle *esc) {
  // Start the PWM signal at min throttle
  hal_pwm_start(&esc->pwm);
}

void esc_disarm(ESC_Handle *esc) {
  // Stop the PWM signal
  hal_pwm_stop(&esc->pwm);
}

void esc_set_throttle(ESC_Handle *esc, float throttle) {
  if (throttle < 0.0f)
    throttle = 0.0f;
  if (throttle > 1.0f)
    throttle = 1.0f;

  // Calculate pulse width in ms
  float pulse_ms =
      esc->min_pulse_ms + (throttle * (esc->max_pulse_ms - esc->min_pulse_ms));

  // Convert pulse width ms to duty cycle fraction (0.0 to 1.0)
  // Duty cycle = (pulse_ms / period_ms)
  // period_ms = 1000ms / frequency
  float period_ms = MS_PER_SECOND / (float)esc->frequency;
  float duty = (pulse_ms / period_ms);

  hal_pwm_set_duty_cycle(&esc->pwm, duty);
}
