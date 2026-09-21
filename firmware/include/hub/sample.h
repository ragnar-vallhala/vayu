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
 * @file hub/sample.h
 * @brief What a sensor hands the core, in SI units and with no chip in it.
 *
 * The core reads these. It does not read driver headers, and these types name
 * no device -- swapping a BMX160 for another IMU changes which driver is
 * compiled, not what the estimator consumes.
 *
 * This is the type that used to be `bmx160_all_converted_reading_t`. Its
 * contents were already SI; the problem was the name and the company it kept.
 * A consumer that wanted a gyro reading had to include the BMX160 header,
 * which includes navhal.h, so the control layer ended up holding the whole
 * STM32 register map to read three floats.
 *
 * Narrower than what it replaces, on purpose. The driver's working set --
 * uncalibrated accel/gyro, pre-offset magnetometer -- stayed behind: nothing
 * outside the driver read it, and a public type that exposes intermediate
 * results invites someone to depend on them.
 *
 * MEASUREMENT AND HEALTH, NOT POLICY. A driver reports what it measured and
 * whether it trusts it; what to do about it is the estimator's call. So the
 * magnetometer crosses as microtesla plus a validity flag -- the driver owns
 * the finiteness, Earth-field-magnitude and disturbance checks, because those
 * need device knowledge -- and the estimator normalises and decides how much
 * to weigh it, because that is fusion policy.
 */
#ifndef VAYU_HUB_SAMPLE_H
#define VAYU_HUB_SAMPLE_H

#include <stdint.h>

/** One inertial measurement. */
typedef struct {
  float acc[3];  /**< m/s^2, calibrated (bias + soft-iron corrected)      */
  float gyr[3];  /**< deg/s, calibrated (bias corrected)                  */
  float mag[3];  /**< uT, calibrated (hard- + soft-iron corrected)        */
  float temp_c;  /**< die temperature, degrees Celsius                    */
  uint32_t t_cyc;/**< cycle stamp at acquisition; dt comes from deltas of
                  *   this, never from reading the counter at use time --
                  *   see vayu_dt_from_cycles() in sys/clock.h            */
  /** Driver's verdict on mag[]: finite, Earth-field magnitude, and no
   *  detected disturbance. 0 means measured but not trustworthy, which is
   *  not the same as absent -- the estimator coasts rather than resets. */
  uint8_t mag_valid;
  /** Which sensor of this kind produced it. 0 is the primary; a board
   *  carrying two or three IMUs publishes one stream per instance and the
   *  estimator decides what to do with the extras. */
  uint8_t instance;
} imu_sample_t;

/** One barometric measurement. */
typedef struct {
  float pressure_pa;
  float temp_c;
  uint32_t t_cyc;
  uint8_t valid;
  uint8_t instance;
} baro_sample_t;

/** One rangefinder measurement, along the sensor's own axis. */
typedef struct {
  float range_m; /**< metres; meaning of out-of-range is the driver's to flag */
  uint32_t t_cyc;
  uint8_t valid;
  uint8_t instance;
} range_sample_t;

#endif /* VAYU_HUB_SAMPLE_H */
