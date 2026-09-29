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
 * @file sensor/imu.h
 * @brief What "an IMU" can be asked to do, whichever chip it is.
 *
 * @details
 * Samples do not come through here -- an IMU publishes those to `hub/` and the
 * estimator reads them there, which is the hot path and stays a direct ring
 * write. This is the small set of things the autopilot *asks an IMU to do*,
 * and until now it asked a BMX160 specifically.
 *
 * What that cost, concretely: `comm_processor.c` handled CMD_CALIBRATE_IMU by
 * allocating the calibration task's argument block, creating that task with a
 * 3072-word stack it had measured itself, tracking the handle, and cancelling
 * through `bmx160_calib_request_cancel()`. The command layer knew the stack
 * depth of a routine inside the IMU driver. A second IMU would have had to
 * match that shape exactly or the command layer would have needed a branch.
 *
 * Now the driver fills one const struct of function pointers and the command
 * layer asks the model. A driver that cannot do something leaves the pointer
 * NULL and the caller gets VAYU_ERR_NOT_IMPL rather than a link error, so a
 * minimal backend is a legal backend.
 *
 * The ops live in flash, reached through the driver's `sensor_driver_t` (see
 * sensor/sensor.h). There is no registration step and no RAM behind this.
 */
#ifndef VAYU_SENSOR_IMU_H
#define VAYU_SENSOR_IMU_H

#include "vayu_status.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * The IMU model. Every entry may be NULL on a backend that does not offer it.
 */
typedef struct {
  /**
   * Begin a calibration run, and own everything it needs: its arguments, its
   * task, and that task's stack. The caller supplies only what was asked for
   * on the wire.
   *
   * @param imu_id which sub-device the routine targets (accel, gyro, mag, or
   *               the board-level mounting trim), as the GCS numbers them.
   * @param type   which routine.
   * @return VAYU_OK if a run started; VAYU_ERR_BUSY if one is already in
   *         flight; VAYU_ERR_FAULT if resources could not be had.
   */
  vayu_status_t (*calibrate_start)(uint32_t imu_id, uint32_t type);

  /**
   * Ask a running calibration to stop. Cooperative by contract: it raises a
   * flag the routine checks at a loop boundary, so the routine tears itself
   * down -- restoring state and freeing its own arguments -- rather than being
   * killed part-way through with the chip left half-configured.
   */
  void (*calibrate_cancel)(void);

  /** True while a calibration run owns the device. */
  bool (*calibrating)(void);

  /* There is no identity entry here. One was added with this header and had no
 * caller -- the boot log names the backend from the descriptor's compile-time
 * `name`, which costs no call and cannot fail. Add a chip_id op when something
 * actually needs the value the device reports, not before. */
} imu_ops_t;

/**
 * The selected IMU's model, or NULL if this build has no IMU backend (SITL,
 * which feeds the hub directly, is the normal case for that).
 */
const imu_ops_t *imu_ops(void);

#endif // VAYU_SENSOR_IMU_H
