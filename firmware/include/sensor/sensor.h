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
 * @file sensor/sensor.h
 * @brief The sensor adapter: bring a sensor up without naming which one.
 *
 * @details
 * Swapping a BMX160 for an ICM should be a build-configuration change, not an
 * edit to `main.c`. It was an edit to `main.c`: the boot path called
 * `bmx160_init()`, `bme280_init()` and `vl53l0x_init()` by name and created
 * each driver's acquisition task by name, with that driver's stack size
 * written next to it.
 *
 * Here a driver DESCRIBES itself and the adapter reads the description. Each
 * driver defines one `const sensor_driver_t` through VAYU_SENSOR_DRIVER, which
 * places it in the `vayu_sensors` linker section; the adapter walks that
 * section between the `__start_`/`__stop_` symbols the linker provides. The
 * table is `const`, so it lives in flash and costs no RAM, and nothing has to
 * hold a registry or run a registration pass at boot.
 *
 * Nothing imports a driver header: not this file, not `main.c`. The only thing
 * that decides which backend exists is which driver file the build compiles --
 * see `BOARD_SENSOR_DRIVERS` in the board's `board.cmake`. An unselected
 * driver is never compiled, so it contributes no flash, no RAM and no entry.
 *
 * DATA does not come through here. A driver publishes its samples to `hub/`
 * (`imu_queue_*`, `baro_publish`, `range_publish`) and consumers read them
 * there. This file is the LIFECYCLE seam only: probe, and start the
 * acquisition task. Keeping the two apart is deliberate -- the data path is
 * hot and must stay a direct ring write.
 *
 * ORDER: the linker decides where entries land, NOT the source. Two drivers
 * registered in one file can come out in either order. So nothing here may
 * depend on table order -- the adapter is driven by `sensor_kind_t`, which is
 * ordered by this header and therefore stable.
 */
#ifndef VAYU_SENSOR_H
#define VAYU_SENSOR_H

#include "vayu_status.h"
#include <stdint.h>

/** What a sensor measures. Probe order follows this enum, so it is the one
 *  ordering the boot path can rely on. */
typedef enum {
  SENSOR_IMU = 0, /**< gyro + accel (+ mag), the flight-critical one */
  SENSOR_BARO,    /**< barometric pressure */
  SENSOR_RANGE,   /**< downward rangefinder */
  SENSOR_KIND_COUNT
} sensor_kind_t;

/**
 * One driver's self-description. All fields are compile-time constants; the
 * struct lives in flash.
 */
typedef struct {
  const char *name; /**< chip name, for logs: "bmx160". Never matched on. */
  uint8_t kind;     /**< a sensor_kind_t */
  uint8_t instance; /**< 0 unless a board carries two of a kind */

  /**
   * Bring the device up: probe it, configure it, leave it producing.
   *
   * VAYU_OK means present and configured. Anything else means the adapter
   * logs it and carries on -- a missing barometer must not stop a boot, and
   * on this airframe it does not (the vertical estimator falls back). Whether
   * a given sensor is REQUIRED is the flight code's call, not the adapter's.
   */
  vayu_status_t (*probe)(void);

  /** Acquisition task entry, or NULL if the driver needs no task of its own
   *  (a ride-along on another device's bus loop, for instance). */
  void (*task)(void *args);
  const char *task_name; /**< for the task list and stack reports */
  uint16_t stack_words;  /**< measured peak + margin; see the driver */
  uint8_t priority;

  /**
   * Paced acquisition callback and its cadence, for a device whose sampling
   * is driven from the high-frequency timer rather than a task loop. NULL if
   * the driver paces itself.
   *
   * The adapter does not register this: the timer comes up after the sensors
   * do, and wiring a service to a descriptor is the composition root's job
   * (main.c). Putting it here keeps the cadence with the driver that knows
   * why it is what it is.
   */
  void (*tick)(void);
  uint32_t tick_period_us;

  /**
   * This kind's operation table, or NULL if the driver offers none.
   *
   * Type-erased because the type depends on `kind` -- an IMU points this at a
   * `const imu_ops_t` (sensor/imu.h). Only the accessor for a kind casts it,
   * and only after checking `kind`, so the erasure never escapes this layer.
   * It is a pointer into flash; nothing owns or frees it.
   */
  const void *ops;
} sensor_driver_t;

/**
 * Register a driver. One per driver file, at file scope:
 *
 *     VAYU_SENSOR_DRIVER(bmx160_sensor) = {
 *         .name = "bmx160", .kind = SENSOR_IMU, .probe = ..., ...
 *     };
 *
 * `used` keeps it through optimisation (nothing references the symbol), and
 * the linker section is what the adapter walks.
 */
#define VAYU_SENSOR_DRIVER(sym)                                                \
  static const sensor_driver_t sym                                             \
      __attribute__((used, section("vayu_sensors"), aligned(4)))

/** The driver registered for a kind, or NULL if the build has none.
 *  With more than one, the first found wins and probe_all has logged it. */
const sensor_driver_t *sensor_backend(sensor_kind_t kind);

/** Probe every registered sensor, in sensor_kind_t order.
 *  @return how many reported VAYU_OK. */
uint8_t sensor_probe_all(void);

/** Create the acquisition task for one kind, if it has one.
 *  Called at the point in the boot sequence that kind belongs at, so task
 *  creation order is the boot path's to choose, not the linker's. */
vayu_status_t sensor_start_task(sensor_kind_t kind);

/** How many drivers are registered in this build. */
uint8_t sensor_count(void);

#endif // VAYU_SENSOR_H
