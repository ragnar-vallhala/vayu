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
 * @file sensor.h
 * @brief Public umbrella header for the sensor module (SNS).
 *
 * @implements R2.1
 *
 * Single public entry point for the sensor subsystem: the BMX160 IMU
 * driver, the IMU sample buffers/queues, and the I2C bus manager.
 * External modules include only this header.
 *
 * The BMX160 register map and driver internals remain in
 * `sensor/bmx160.h` (a large hardware-facing header reached through this
 * umbrella); a public/private split of that driver header is deferred —
 * it is not worth the churn/risk on a 260-line flight driver for this
 * pass, and external code only needs the umbrella.
 */
#ifndef VAYU_SENSOR_H
#define VAYU_SENSOR_H

#include "sensor/bme280.h" /* barometer/humidity (pressure/temp/RH/alt) driver */
#include "sensor/bmx160.h"      /* IMU driver + reading/calibration types */
#include "sensor/i2c_manager.h" /* shared I2C bus manager */
#include "sensor/imu_buffer.h"  /* IMU sample + attitude queues */
#include "sensor/vl53l0x.h"     /* ToF rangefinder (I2C1 ride-along) */

#endif // VAYU_SENSOR_H
