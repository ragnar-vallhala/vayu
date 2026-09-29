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

#include "sensor/baro.h"
#include "sensor/imu.h"

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
  /* cppcheck-suppress comparePointers  -- __start_/__stop_ are the
   * linker's bounds on ONE contiguous section, which cppcheck cannot see:
   * it reads them as two unrelated arrays. Walking between them is the
   * mechanism, not a mistake. */
  return (uint8_t)(__stop_vayu_sensors - __start_vayu_sensors);
}

/** @noreq registry accessor. */
const sensor_driver_t *sensor_backend(sensor_kind_t kind) {
  if (sensor_count() == 0) {
    return NULL;
  }
  /* cppcheck-suppress comparePointers  -- __start_/__stop_ are the
   * linker's bounds on ONE contiguous section, which cppcheck cannot see:
   * it reads them as two unrelated arrays. Walking between them is the
   * mechanism, not a mistake. */
  for (const sensor_driver_t *d = __start_vayu_sensors; d < __stop_vayu_sensors;
       d++) {
    if (d->kind == (uint8_t)kind) {
      return d;
    }
  }
  return NULL;
}

/* Kind names for the boot log. A table rather than a switch so that adding a
 * kind without naming it yields NULL -- which prints as "?" -- instead of
 * silently falling through to a wrong label. */
static const char *const _kind_name[SENSOR_KIND_COUNT] = {
    [SENSOR_IMU] = "imu",
    [SENSOR_BARO] = "baro",
    [SENSOR_RANGE] = "range",
};

static const char *_kind_str(uint8_t k) {
  if (k >= SENSOR_KIND_COUNT || _kind_name[k] == NULL) {
    return "?";
  }
  return _kind_name[k];
}

/* Count the entries claiming a kind. Exactly one is the contract: the build
 * selects a backend per kind, so two means the board's driver list is wrong,
 * and which one wins would then be the linker's choice. */
static uint8_t _claims(sensor_kind_t kind) {
  uint8_t n = 0;
  if (sensor_count() == 0) {
    return 0;
  }
  /* cppcheck-suppress comparePointers  -- __start_/__stop_ are the
   * linker's bounds on ONE contiguous section, which cppcheck cannot see:
   * it reads them as two unrelated arrays. Walking between them is the
   * mechanism, not a mistake. */
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
      vayu_log("[SNS] %s has %u drivers; the build must select one",
               _kind_str((uint8_t)k), (unsigned)claims);
    }

    const sensor_driver_t *d = sensor_backend((sensor_kind_t)k);
    if (d->probe == NULL) {
      continue;
    }
    vayu_status_t st = d->probe();
    if (st == VAYU_OK) {
      ok++;
      /* One line per sensor, same shape whatever the chip is. Drivers used to
       * each announce their own success in their own format and the IMU
       * announced nothing at all, so "did the IMU come up?" could only be
       * answered by watching for attitude to start streaming. */
      vayu_log("[SNS] %s %s ok", _kind_str(d->kind), d->name);
    } else {
      /* Not fatal, and deliberately so: a missing barometer or rangefinder
       * degrades the vertical estimate, it does not ground the aircraft.
       * Whether a kind is required is the flight code's call. */
      vayu_log("[SNS] %s %s absent (%d)", _kind_str(d->kind), d->name, (int)st);
    }
  }
  return ok;
}

/** @implements SNS-BOOT-002 */
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

/* The one place the descriptor's type-erased ops pointer is cast, and only
 * after sensor_backend has matched the kind -- so an entry filed as an IMU is
 * the only thing that can ever be read as imu_ops_t. */
/** @noreq registry accessor. */
const imu_ops_t *imu_ops(void) {
  const sensor_driver_t *d = sensor_backend(SENSOR_IMU);
  return d ? (const imu_ops_t *)d->ops : NULL;
}

/** @noreq registry accessor. */
const baro_ops_t *baro_ops(void) {
  const sensor_driver_t *d = sensor_backend(SENSOR_BARO);
  return d ? (const baro_ops_t *)d->ops : NULL;
}
