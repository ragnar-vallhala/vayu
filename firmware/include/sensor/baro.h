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
 * This exists for the one thing the hub sample deliberately does not carry:
 * the full reading that goes out on the BARO telemetry message, which also
 * includes humidity and a derived altitude. Humidity is not something a
 * barometer has (a BMP280 or an MS5611 has none), and altitude is computed
 * from pressure against a datum rather than measured. Widening the SI sample
 * to hold either would put a chip-specific field in the type the control core
 * reads, which is the mistake fault line F11 was about.
 *
 * So the split is deliberate: measurements the flight code needs go through
 * the hub, and a reporting-only readout is asked for here. `has_humidity` is
 * what keeps that honest -- a chip without one reports 0 AND says so, which is
 * a different statement from "0% relative humidity".
 */
#ifndef VAYU_SENSOR_BARO_H
#define VAYU_SENSOR_BARO_H

#include "vayu_status.h"
#include <stdint.h>

/** A barometer's full readout, as reported rather than as flown. */
typedef struct {
  float pressure_pa;
  float temperature_c;
  /** Relative humidity. Meaningless unless `has_humidity`. */
  float humidity_rh;
  /** Derived from pressure against the configured sea-level datum. */
  float altitude_m;
  /** Cycle stamp at acquisition (see vayu_dt_from_cycles). */
  uint32_t t_cyc;
  /** 0 on a part that does not measure humidity. */
  uint8_t has_humidity;
} baro_reading_t;

/** The barometer model. Any entry may be NULL. */
typedef struct {
  /**
   * The latest full readout.
   *
   * @return VAYU_OK, or VAYU_ERR_NOT_IMPL before the device has produced its
   *         first sample -- which is a normal state for the first moments
   *         after boot, and for a board where the part is absent.
   */
  vayu_status_t (*read)(baro_reading_t *out);
} baro_ops_t;

/** The selected barometer's model, or NULL if this build has no backend. */
const baro_ops_t *baro_ops(void);

#endif // VAYU_SENSOR_BARO_H
