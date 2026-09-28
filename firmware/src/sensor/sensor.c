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
/* See sensor/sensor.h.
 *
 * This file names no chip and includes no driver header, and the layering gate
 * holds it at zero for exactly that reason. If it ever needs to know which
 * device it is talking to, the descriptor is missing a field. */
#include "sensor/sensor.h"

#include "storage/fs_owner.h" /* vayu_log */
#include "task.h"
#include <stddef.h>

/* Section bounds. The firmware's linker.ld PROVIDEs these; on a link where the
 * section does not exist at all they stay undefined, which is why they are
 * WEAK: an undefined weak symbol resolves to NULL, so the table is empty
 * rather than the link failing.
 *
 * That is not a theoretical case. SITL does not compile the IMU driver -- it
 * feeds samples straight into the hub -- so a host binary genuinely has no IMU
 * entry, and a test executable that pulls no driver object out of the static
 * library has none at all. Without weak, every such link breaks.
 *
 * The flip side is that an EMPTY table is now a link-time success, so
 * sensor_probe_all says so loudly at boot. */
extern const sensor_driver_t __start_vayu_sensors[] __attribute__((weak));
extern const sensor_driver_t __stop_vayu_sensors[] __attribute__((weak));

/** @noreq registry accessor. */
uint8_t sensor_count(void) {
  if (__start_vayu_sensors == NULL || __stop_vayu_sensors == NULL) {
    return 0;
  }
  return (uint8_t)(__stop_vayu_sensors - __start_vayu_sensors);
}

/** @noreq registry accessor. */
const sensor_driver_t *sensor_backend(sensor_kind_t kind) {
  if (sensor_count() == 0) {
    return NULL;
  }
  for (const sensor_driver_t *d = __start_vayu_sensors; d < __stop_vayu_sensors;
       d++) {
    if (d->kind == (uint8_t)kind) {
      return d;
    }
  }
  return NULL;
}

/* Count the entries claiming a kind. Exactly one is the contract: the build
 * selects a backend per kind, so two means the board's driver list is wrong,
 * and which one wins would then be the linker's choice. */
static uint8_t _claims(sensor_kind_t kind) {
  uint8_t n = 0;
  if (sensor_count() == 0) {
    return 0;
  }
  for (const sensor_driver_t *d = __start_vayu_sensors; d < __stop_vayu_sensors;
       d++) {
    if (d->kind == (uint8_t)kind) {
      n++;
    }
  }
  return n;
}

/** @implements SNS-BOOT-001 */
uint8_t sensor_probe_all(void) {
  uint8_t ok = 0;

  if (sensor_count() == 0) {
    /* Nothing registered. On the firmware this is a build fault wearing a
     * runtime disguise -- the board selected no drivers, or the registry got
     * garbage-collected -- and it means booting with no gyro. Say so; the
     * alternative is an aircraft that looks fine until it is armed. */
    vayu_log("[SNS] NO SENSOR DRIVERS REGISTERED");
    return 0;
  }

  for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
    uint8_t claims = _claims((sensor_kind_t)k);
    if (claims == 0) {
      continue; /* a board need not carry every kind */
    }
    if (claims > 1) {
      /* Nothing here can resolve this: the table order is the linker's, so
       * the "winner" is not reproducible between builds. Name it at boot
       * rather than let it look like it worked. */
      vayu_log("[SNS] kind %d has %u drivers; the build must select one", k,
               (unsigned)claims);
    }

    const sensor_driver_t *d = sensor_backend((sensor_kind_t)k);
    if (d->probe == NULL) {
      continue;
    }
    vayu_status_t st = d->probe();
    if (st == VAYU_OK) {
      ok++;
    } else {
      /* Not fatal, and deliberately so: a missing barometer or rangefinder
       * degrades the vertical estimate, it does not ground the aircraft.
       * Whether a kind is required is the flight code's call. */
      vayu_log("[SNS] %s absent (%d)", d->name, (int)st);
    }
  }
  return ok;
}

/** @implements SNS-BOOT-001 */
vayu_status_t sensor_start_task(sensor_kind_t kind) {
  const sensor_driver_t *d = sensor_backend(kind);
  if (d == NULL) {
    return VAYU_ERR_INVALID;
  }
  if (d->task == NULL) {
    return VAYU_OK; /* a ride-along has no task of its own */
  }
  (void)task_create_named(d->task, NULL, d->stack_words, d->priority,
                          d->task_name);
  return VAYU_OK;
}
