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

#include "driver/esc.h"
#include "navhal.h"
#include <stdint.h>

/* The group's timer. One advanced-control timer drives all four channels, so
 * its prescaler and auto-reload are a property of the GROUP, not of any one
 * ESC -- esc_group_init sets them once and nothing else touches them.
 *
 * It used to be per-channel: esc_init took a timer and called hal_pwm_init,
 * which reconfigures PSC/ARR for the whole timer. Four ESCs meant four
 * timer re-inits, each resetting CNT, with no owner and the last call
 * winning (fault line F7). It worked only because all four asked for the
 * same frequency and all four ran before anything armed. */
static hal_timer_t s_timer;
static uint32_t s_freq = VAYU_ESC_PWM_FREQ;

/* Pulse width -> duty for the group's frequency. */
static float _duty_for_pulse_ms(float pulse_ms) {
  return pulse_ms / (VAYU_ESC_MS_PER_SECOND / (float)s_freq);
}

void esc_group_init(hal_timer_t timer, uint32_t freq_hz) {
  s_timer = timer;
  s_freq = freq_hz ? freq_hz : VAYU_ESC_PWM_FREQ;

  /* The one and only timer configuration. hal_pwm_init derives PSC/ARR from
   * the bus clock and writes them; it also sets up channel 1, which is
   * harmless -- esc_init re-establishes every channel it is given. */
  hal_pwm_handle_t group = {.timer = s_timer, .channel = 1};
  hal_pwm_init(&group, s_freq, _duty_for_pulse_ms(VAYU_ESC_MIN_PULSE_MS));

  /* Running the timer is a GROUP action, so it happens here and exactly once.
   * The old esc_arm did it per channel via hal_pwm_start -- harmless in
   * itself, since starting does not touch PSC or ARR, but it left four
   * callers able to start and stop something they did not own. */
  hal_timer_start(s_timer);
}

void esc_init(ESC_Handle *esc, uint32_t channel, hal_gpio_pin_t pin,
              hal_gpio_af_t af) {
  esc->min_pulse_ms = VAYU_ESC_MIN_PULSE_MS;
  esc->max_pulse_ms = VAYU_ESC_MAX_PULSE_MS;
  esc->frequency = s_freq;

  esc->pwm.timer = s_timer;
  esc->pwm.channel = channel;

  /* Which alternate function carries a timer out of a pin is a package fact,
   * not a driver one -- this file used to decide it from the timer instance
   * with a ladder whose comment said "usually" (F8). The board knows. */
  hal_gpio_enable_clock(pin);
  hal_gpio_set_mode(pin, HAL_GPIO_MODE_AF, HAL_GPIO_PULL_NONE);
  hal_gpio_set_output_speed(pin, HAL_GPIO_SPEED_VERY_HIGH);
  hal_gpio_set_alternate_function(pin, af);

  /* Per-channel setup, without touching the timer: hal_pwm_set_duty_cycle
   * reads the group's ARR and writes this channel's CCR, PWM mode, preload
   * and output enable. That is exactly what hal_pwm_init did for a channel,
   * minus the timer re-init this channel has no business doing. */
  hal_pwm_set_duty_cycle(&esc->pwm, _duty_for_pulse_ms(esc->min_pulse_ms));
}

/* Arm and disarm are CHANNEL scoped. They used to start and stop the timer,
 * which is shared: disarming one ESC stopped the PWM for all four. Nothing
 * called esc_disarm (F6), so it never fired -- but it was a live wire. */
void esc_arm(ESC_Handle *esc) {
  hal_timer_enable_channel(esc->pwm.timer, esc->pwm.channel);
}

void esc_disarm(ESC_Handle *esc) {
  hal_timer_disable_channel(esc->pwm.timer, esc->pwm.channel);
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
  float period_ms = VAYU_ESC_MS_PER_SECOND / (float)esc->frequency;
  float duty = (pulse_ms / period_ms);

  hal_pwm_set_duty_cycle(&esc->pwm, duty);
}
