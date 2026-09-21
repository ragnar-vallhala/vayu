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
#ifndef VAYU_ANGLE_RATE_CONTROLLER_H
#define VAYU_ANGLE_RATE_CONTROLLER_H

#include "control/pid.h"
#include "variables.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  struct PID pid[NUM_AXES];
} AngleRateController;

typedef enum {
  NORMALIZED_RC2ANGLE_RATE_LINEAR, // Linear mode
  NORMALIZED_RC2ANGLE_RATE_CUBIC,  // Cubic mode
} normalized_rc2angle_rate_mode_t;

void angle_rate_controller_init(void);
void angle_rate_controller_task(void *arg);

/**
 * @brief Set the live rate-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true on apply.
 * @implements COMM-CMD-003
 */
bool angle_rate_controller_set_gains(uint8_t axis, float kp, float ki, float kd,
                                     float kff);

/**
 * @brief Read the live rate-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true and fills out-params on ok.
 * @implements COMM-CMD-003
 */
bool angle_rate_controller_get_gains(uint8_t axis, float *kp, float *ki,
                                     float *kd, float *kff);

/**
 * @brief Set the gyro low-pass time constant [s] for one axis (rc <= 0 = off).
 * Input filter on the rate measurement, co-tuned with the gains.
 */
bool angle_rate_controller_set_gyro_lpf(uint8_t axis, float rc);

/** @brief Read the live gyro LPF time constant [s] for one axis. */
float angle_rate_controller_get_gyro_lpf(uint8_t axis);

/**
 * @brief Set the D-term low-pass time constant [s] for one rate axis
 * (rc <= 0 = passthrough / raw derivative). Filters the PID's derivative
 * path; co-tuned with Kd.
 */
bool angle_rate_controller_set_d_lpf(uint8_t axis, float rc);

/** @brief Read the live D-term LPF time constant [s] for one rate axis. */
float angle_rate_controller_get_d_lpf(uint8_t axis);

/**
 * @brief Set the per-motor mix signs from the airframe geometry (motor body
 * positions [m] + spin +1/-1), so roll/pitch/yaw->motor mixing matches the
 * actual layout. Keeps the firmware mixer consistent with the sim/vehicle.
 */
void angle_rate_controller_set_motor_geometry(const float pos_x[4],
                                              const float pos_y[4],
                                              const int spin[4]);

/** @brief Apply a CMD_SET_MOTOR_GEOMETRY payload (12 floats: x[4],y[4],spin[4]). */
bool angle_rate_controller_apply_geometry_command(const uint8_t *payload,
                                                  uint16_t len);

#endif // VAYU_ANGLE_RATE_CONTROLLER_H