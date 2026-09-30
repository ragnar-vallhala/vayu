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

/*
 * Every sample carries its own `temp_c`. A die's temperature is the input a
 * temperature-compensation curve needs, and each die has its own -- an
 * accelerometer's bias drift and a magnetometer's scale drift are different
 * curves against different temperatures. Today the BMX160 reports one
 * temperature for the whole package, so the IMU and compass samples carry the
 * same number; on a board whose compass is a separate chip they will not, and
 * nothing downstream has to change for that to start being true.
 */

/** One inertial measurement: accelerometer and rate gyro. */
typedef struct {
  float acc[3];     /**< m/s^2, calibrated (bias + soft-iron corrected)     */
  float gyr[3];     /**< deg/s, calibrated (bias corrected)                 */
  float temp_c;     /**< the inertial die's temperature                     */
  uint32_t t_cyc;   /**< cycle stamp at acquisition; dt comes from deltas of
                   *   this, never from reading the counter at use time --
                   *   see vayu_dt_from_cycles() in sys/clock.h           */
  uint8_t instance; /**< 0 is the primary; boards may carry two or three  */
} imu_sample_t;

/**
 * One magnetic field measurement.
 *
 * Separate from the inertial sample because a compass is a separate sensor --
 * separate die inside the BMX160, and on other boards a separate chip on a
 * separate bus. It runs at its own rate (tens of Hz against the gyro's
 * thousands), so pinning it to the inertial sample meant republishing an
 * unchanged field thousands of times a second and gave an estimator no way to
 * tell a fresh reading from a repeat.
 */
typedef struct {
  float mag[3]; /**< uT, calibrated (hard- + soft-iron corrected)       */
  float temp_c; /**< the magnetometer die's temperature                 */
  uint32_t t_cyc;
  /** Driver's verdict: finite, Earth-field magnitude, no detected
   *  disturbance. 0 means measured but not trustworthy, which is not the
   *  same as absent -- the estimator coasts rather than resets. */
  uint8_t valid;
  uint8_t instance;
} mag_sample_t;

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

/** One pack-voltage reading.
 *
 * Here rather than read from the driver directly for the same reason every
 * other measurement is: the things that want it (telemetry, the recorder) are
 * in gated sections and must not name a device. `valid` is false when no
 * conversion has succeeded yet, which is how an unplugged pack and a broken
 * ADC read differently from 0.0 V. */
typedef struct {
  float volts;
  uint32_t t_cyc;
  /* The raw ADC count the volts came from. Carried in the sample rather than
   * fetched from the driver, because the consumers that want it -- telemetry
   * and the recorder -- are in sections that must not name a device. It earns
   * its place: the scale is a calibrated board constant, so when a reading
   * looks wrong the count is what says whether the ADC or the constant is at
   * fault. */
  uint16_t counts;
  uint8_t valid;
  uint8_t instance;
} battery_sample_t;

/** ISA standard sea-level pressure, the default altitude datum. */
#define HUB_SEA_LEVEL_PA_DEFAULT 101325.0f

/**
 * Barometric altitude from pressure. Pure maths with no device in it, so it
 * belongs beside the sample rather than inside whichever barometer produced
 * it -- a driver holding the altitude datum is a driver holding navigation
 * state.
 */
float hub_altitude_m(float pressure_pa, float sea_level_pa);

#endif /* VAYU_HUB_SAMPLE_H */
