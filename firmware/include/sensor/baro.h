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
 * @file sensor/baro.h
 * @brief What "a barometer" can be asked for, whichever chip it is.
 *
 * @details
 * The estimator does NOT use this. It takes `baro_sample_t` from `hub/` --
 * pressure and temperature, the two things every barometer measures -- and
 * that stays the hot path.
 *
 * It exists for exactly one thing: humidity, which not every barometer
 * measures (a BMP280 or an MS5611 has none) and which the control core has no
 * use for. Widening `baro_sample_t` to hold it would put a chip-specific
 * field in the type the control core reads, which is what fault line F11 was
 * about.
 *
 * Everything else about a barometer comes from the hub. Pressure and
 * temperature are `baro_sample_t`. Altitude is NOT a barometer's to report --
 * turning pressure into height needs a sea-level datum, which is navigation
 * state; `hub_altitude_m()` is the one derivation and it takes that datum as
 * an argument.
 */
#ifndef VAYU_SENSOR_BARO_H
#define VAYU_SENSOR_BARO_H

#include "vayu_status.h"

/** The barometer model. Any entry may be NULL. */
typedef struct {
  /**
   * Relative humidity, for the parts that measure it.
   *
   * This is the ONLY thing the model carries, because it is the only thing
   * about a barometer the hub deliberately does not: `baro_sample_t` holds
   * pressure and temperature, which is what every barometer measures and what
   * the flight code needs.
   *
   * There used to be a `baro_reading_t` here that restated pressure,
   * temperature and the timestamp alongside humidity and an altitude. It was
   * a second type for one device's output -- it even renamed a field, hub
   * `temp_c` against model `temperature_c`, which is how two names for one
   * quantity start drifting apart. Altitude was worse: see the note on
   * hub_altitude_m in hub/sample.h.
   *
   * @return VAYU_OK, VAYU_ERR_NOT_IMPL on a part with no humidity sensor
   *         (a BMP280, an MS5611), or VAYU_ERR_FAULT before the first sample.
   *         Reporting 0 with a status is a different statement from "0% RH".
   */
  vayu_status_t (*humidity)(float *rh);
} baro_ops_t;

/** The selected barometer's model, or NULL if this build has no backend. */
const baro_ops_t *baro_ops(void);

#endif // VAYU_SENSOR_BARO_H
