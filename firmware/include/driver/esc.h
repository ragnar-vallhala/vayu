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
 * @file driver/esc.h
 * @brief Electronic Speed Controller driven over PWM.
 *
 * A device driver: it owns the pulse band the ESC expects and the PWM channel
 * that carries it. The mixer that decides what to send lives in
 * actuator/motor.h and is not a driver.
 *
 * @copyright © NAVROBOTEC PVT. LTD.
 */
#ifndef VAYU_DRIVER_ESC_H
#define VAYU_DRIVER_ESC_H

#include "navhal.h"
#include <stdint.h>

/* ----------------------------------------------------------------------------
 * ESC — single Electronic Speed Controller driven over PWM.
 * --------------------------------------------------------------------------*/

/** @brief ESC handle structure. */
typedef struct {
  hal_pwm_handle_t pwm;
  float min_pulse_ms;
  float max_pulse_ms;
  uint32_t frequency;
} ESC_Handle;

/* ESC signal band. Exported rather than kept private to esc.c because the SITL
 * host HAL has to strip this exact band back off to recover the linear motor
 * command -- it used to re-derive the duty figures by hand, so changing the
 * band here silently desynced sim from hardware. One definition, both sides. */
#define VAYU_ESC_MIN_PULSE_MS 1.0f     /**< pulse width at zero throttle */
#define VAYU_ESC_MAX_PULSE_MS 2.0f     /**< pulse width at full throttle */
#define VAYU_ESC_PWM_FREQ 400u         /**< typical ESC frame rate */
#define VAYU_ESC_MS_PER_SECOND 1000.0f /**< ms<->Hz period conversion (R10.3) */

/** Duty cycle (0..1) a given pulse width occupies in one ESC frame. */
#define VAYU_ESC_DUTY(pulse_ms)                                                \
  ((pulse_ms) / (VAYU_ESC_MS_PER_SECOND / (float)VAYU_ESC_PWM_FREQ))
/** Duty at zero throttle (0.4 at 1 ms / 400 Hz). */
#define VAYU_ESC_MIN_DUTY VAYU_ESC_DUTY(VAYU_ESC_MIN_PULSE_MS)
/** Duty at full throttle (0.8 at 2 ms / 400 Hz). */
#define VAYU_ESC_MAX_DUTY VAYU_ESC_DUTY(VAYU_ESC_MAX_PULSE_MS)

/**
 * @brief Initialize an ESC on a specific timer and channel.
 * @param esc Pointer to the ESC handle.
 * @param timer Hardware timer.
 * @param channel PWM channel.
 * @param pin GPIO pin for PWM output.
 *
 * @implements ACT-ESC-001
 */
/**
 * @brief Configure the timer every ESC channel shares. Call once, before any
 *        esc_init. The prescaler and auto-reload belong to the group; no
 *        per-channel call may change them.
 */
void esc_group_init(hal_timer_t timer, uint32_t freq_hz);

void esc_init(ESC_Handle *esc, uint32_t channel, hal_gpio_pin_t pin,
              hal_gpio_af_t af);

/** @brief Arm the ESC (sends min throttle for a period).
 *  @noreq thin channel-enable primitive; the boot arming sequence is
 *  ACT-ESC-002. Channel scoped: it does not start the shared timer. */
void esc_arm(ESC_Handle *esc);

/** @brief Disarm the ESC (stops PWM or sends a safe signal).
 *  @noreq thin hal_pwm_stop primitive (not the failsafe path; see ACT-FAIL-001). */
void esc_disarm(ESC_Handle *esc);

/**
 * @brief Set the throttle level for the ESC.
 * @param throttle Throttle value from 0.0 to 1.0.
 *
 * @implements ACT-MOT-001
 */
void esc_set_throttle(ESC_Handle *esc, float throttle);

#endif /* VAYU_DRIVER_ESC_H */
