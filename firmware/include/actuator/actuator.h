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
 * @file actuator.h
 * @brief Public umbrella header for the actuator module (ESC + motor mixer).
 *
 * @implements R2.1
 *
 * Single public surface for the module per R2.1 — external code includes
 * only this header, never the per-type sources.
 *
 * @copyright © NAVROBOTEC PVT. LTD.
 */
#ifndef VAYU_ACTUATOR_H
#define VAYU_ACTUATOR_H

#include "navhal.h"
#include "structure.h"
#include <stdbool.h>
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

/**
 * @brief Initialize an ESC on a specific timer and channel.
 * @param esc Pointer to the ESC handle.
 * @param timer Hardware timer.
 * @param channel PWM channel.
 * @param pin GPIO pin for PWM output.
 *
 * @implements ACT-ESC-001
 */
void esc_init(ESC_Handle *esc, hal_timer_t timer, uint32_t channel,
              hal_gpio_pin_t pin);

/** @brief Arm the ESC (sends min throttle for a period).
 *  @noreq thin hal_pwm_start primitive; the boot arming sequence is ACT-ESC-002. */
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

/* ----------------------------------------------------------------------------
 * Motor mixer — the four-rotor output stage.
 * --------------------------------------------------------------------------*/

#define NUM_MOTORS 4
#define MOTOR_QUEUE_SIZE 4

typedef struct {
  float m1;
  float m2;
  float m3;
  float m4;
} motor_outputs_t;

/** @implements ACT-ESC-002, ACT-MOT-003 */
void motor_init(void);
/** @noreq trivial motor-ready flag setter */
void set_motor_ready(bool ready);
/** @noreq trivial motor-ready flag getter */
bool get_motor_ready(void);
/** @noreq motor-output FIFO producer; thin spsc_write wrapper */
void motor_set_outputs(motor_outputs_t motor_outputs);
/** @implements ACT-MOT-002, ACT-FAIL-001 */
void motor_task(void *arg);
/** @noreq motor-telemetry FIFO accessor; thin spsc_read wrapper */
bool motor_telemetry_queue_pop(motor_outputs_t *out_data);

#endif // VAYU_ACTUATOR_H
