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
 * @file sensor/ride_along.c
 * @brief The devices sharing the IMU's bus (see sensor/ride_along.h).
 *
 * This file exists to be the ONLY place that knows which devices ride the
 * loop. Everything here is a list; the bus facts belong to each driver's own
 * header. When the bus arbiter lands, each driver registers itself and this
 * file is deleted rather than edited.
 */

#include "driver/ride_along.h"

#include "driver/bme280.h"
#include "driver/vl53l0x.h"

/* Contention order: the barometer is first because it is the slower device and
 * the one the vertical estimator depends on; a ToF slot that loses a collision
 * is retried a slot later and costs nothing at 21 Hz. The two periods are
 * coprime (see each driver's *_RIDE_EVERY_N), so collisions are rare anyway. */
static const sensor_ride_t s_rides[] = {
    {.addr = BME280_I2C_ADDR,
     .reg = BME280_REG_DATA,
     .len = BME280_DATA_LEN,
     .every_n = BME280_RIDE_EVERY_N,
     .present = bme280_is_present,
     .ingest = bme280_ingest_raw},
    {.addr = VL53L0X_I2C_ADDR,
     .reg = VL53L0X_REG_BURST_START,
     .len = VL53L0X_DATA_LEN,
     .every_n = VL53L0X_RIDE_EVERY_N,
     .present = vl53l0x_is_present,
     .ingest = vl53l0x_ingest_raw},
};

/** @noreq ride-along registry accessor. */
const sensor_ride_t *sensor_rides(uint8_t *count) {
  if (count != NULL) {
    *count = (uint8_t)(sizeof(s_rides) / sizeof(s_rides[0]));
  }
  return s_rides;
}
