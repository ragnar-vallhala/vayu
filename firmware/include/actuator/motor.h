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
 * @file actuator/motor.h
 * @brief Four-rotor mixer and the output-stage task.
 *
 * Maps a control demand onto four motor commands and drives the ESCs through
 * driver/esc.h. Mixing is policy, not a device, so it stays out of driver/.
 *
 * @implements R2.1
 * @copyright © NAVROBOTEC PVT. LTD.
 */
#ifndef VAYU_ACTUATOR_MOTOR_H
#define VAYU_ACTUATOR_MOTOR_H

#include "structure.h"
#include <stdbool.h>
#include <stdint.h>

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

#endif /* VAYU_ACTUATOR_MOTOR_H */
