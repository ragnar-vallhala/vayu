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
/* The ride-along table (see sensor/ride_along.h).
 *
 * This used to be a list of devices that imported bme280.h and vl53l0x.h to
 * build it -- which meant a driver the build did not select still had to
 * exist, because this file referenced it. Its own header said it should be
 * deleted rather than edited once drivers could register themselves. They can
 * now, so it was.
 *
 * What remains walks a linker section and names nothing. */
#include "sensor/ride_along.h"

#include <stddef.h>

/* Weak for the same reason as the sensor table: a link with no ride-along
 * compiled in leaves these undefined, and an undefined weak symbol is NULL.
 * The bus owner then simply has no ride-alongs to schedule. */
extern const sensor_ride_t __start_vayu_rides[] __attribute__((weak));
extern const sensor_ride_t __stop_vayu_rides[] __attribute__((weak));

/** @noreq ride-along registry accessor. */
const sensor_ride_t *sensor_rides(uint8_t *count) {
  uint8_t n = 0;
  if (__start_vayu_rides != NULL && __stop_vayu_rides != NULL) {
    n = (uint8_t)(__stop_vayu_rides - __start_vayu_rides);
  }
  if (count != NULL) {
    *count = n;
  }
  return __start_vayu_rides;
}
