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
#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "control/pid.h"
#include "variables.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  struct PID pid[NUM_AXES];
} angle_controller_t;
typedef struct {
  float angle_rates[NUM_AXES];
  float throttle;
  float angle_sp[NUM_AXES];
  float angle_curr[NUM_AXES];
  float dt;
} angle_controller_outputs_t;

void angle_controller_init(void);
void angle_controller_task(void *arg);
bool angle_controller_get_outputs(angle_controller_outputs_t *outputs);

/**
 * @brief Latest commanded throttle (normalised 0..1), non-destructive.
 *
 * The outputs FIFO is consumed by the rate controller, so observers that must
 * not steal samples (e.g. the takeoff/landing detector) read the throttle here.
 * O(1) volatile load; 0 until the first control loop runs.
 */
float angle_controller_last_throttle(void);

/* Packed height-mode status for telemetry — the mode is otherwise invisible,
 * which turns "I flipped the switch and nothing happened" into a log hunt.
 *
 *   bit 0-1  requested mode: 0=OFF 1=HOLD 2=LAND (straight from the switch)
 *   bit 2    engaged   — actually driving the collective right now
 *   bit 3    failed    — runaway guard gave up; latched until switch centred
 *   bit 4    landed    — LAND touched down; latched until switch centred
 *   bit 5    handback  — holding collective until the pilot's stick catches up
 *   bit 6    armed_ok  — the switch has been seen CENTRED since arming
 *   bit 7    blocked   — a mode is requested but something is vetoing it
 *                        (acro, upset recovery, or armed_ok still 0)
 *
 * "blocked" with "armed_ok" clear is the common one: the switch was never at
 * centre while armed, so the interlock has not released. */
#define HEIGHT_STATE_MODE_MASK 0x03u
#define HEIGHT_STATE_ENGAGED 0x04u
#define HEIGHT_STATE_FAILED 0x08u
#define HEIGHT_STATE_LANDED 0x10u
#define HEIGHT_STATE_HANDBACK 0x20u
#define HEIGHT_STATE_ARMED_OK 0x40u
#define HEIGHT_STATE_BLOCKED 0x80u
uint8_t angle_controller_height_state(void);

/**
 * @brief Set the live angle-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true on apply.
 * @implements COMM-CMD-003
 */
bool angle_controller_set_gains(uint8_t axis, float kp, float ki, float kd,
                                float kff);
#endif // ANGLE_CONTROLLER_H