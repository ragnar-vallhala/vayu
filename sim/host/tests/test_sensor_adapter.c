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
/* The sensor adapter (firmware/src/sensor/sensor.c).
 *
 * Drivers register themselves into a linker section and the adapter walks it,
 * so the boot path can bring up an IMU without naming one. The failure mode
 * that matters is an EMPTY or MISREAD table: every sensor would report absent
 * at boot and the aircraft would come up with no gyro, with nothing in the
 * build having gone wrong. So the check is that the section really is walked
 * and really is found, on this toolchain, with the drivers this build
 * selected.
 *
 * It links vayu_sitl_core with --whole-archive on purpose. The descriptors are
 * `static` and nothing references them by name, so an ordinary archive link
 * pulls no driver object and the table comes out EMPTY -- which is the one way
 * this mechanism fails quietly, so the test is linked the way that shows it.
 *
 * The host does not compile the IMU driver: SITL feeds samples straight into
 * the hub instead. So the entries here are the barometer and the rangefinder,
 * and a NULL IMU backend is correct on this build rather than a fault. */
#include <stdio.h>
#include <string.h>

#include "driver/bme280.h" /* bme280_publish, to give the model data */
#include "hub/hub.h"
#include "sensor/baro.h"
#include "sensor/imu.h"
#include "sensor/ride_along.h"
#include "sensor/sensor.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

int main(void) {
  printf("== sensor adapter ==\n");

  printf("  [1] the linker section is non-empty\n");
  {
    /* If __start == __stop the walk finds nothing and every probe is skipped.
     * That is the silent-death case: it builds, it boots, it has no sensors. */
    CHECK(sensor_count() > 0, "at least one driver registered");
    printf("      %u drivers registered\n", (unsigned)sensor_count());
  }

  printf("  [2] a registered kind resolves to a usable backend\n");
  {
    /* Whichever kinds this build compiled drivers for must come back filed
     * under the kind asked for, with a name and something to probe. */
    int found = 0;
    for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
      const sensor_driver_t *d = sensor_backend((sensor_kind_t)k);
      if (d == NULL) {
        continue;
      }
      found++;
      CHECK(d->kind == (uint8_t)k, "filed under the kind asked for");
      CHECK(d->probe != NULL, "it can be probed");
      CHECK(d->name != NULL && d->name[0] != '\0', "it has a name");
    }
    CHECK(found > 0, "at least one kind resolves");
    /* Each kind that resolves implies at least one entry behind it, so the
     * count cannot be smaller than the number of kinds found. Pins the count
     * to something real instead of just "not zero". */
    CHECK(sensor_count() >= (uint8_t)found,
          "the count covers every kind that resolved");
    printf("      baro=%s range=%s (no IMU on host: SITL feeds the hub)\n",
           sensor_backend(SENSOR_BARO) ? sensor_backend(SENSOR_BARO)->name
                                       : "-",
           sensor_backend(SENSOR_RANGE) ? sensor_backend(SENSOR_RANGE)->name
                                        : "-");
  }

  printf("  [3] a kind nothing registers returns NULL, not garbage\n");
  {
    /* Out of range, and past the end of the enum: both must be misses rather
     * than a read off the end of the section. */
    CHECK(sensor_backend((sensor_kind_t)SENSOR_KIND_COUNT) == NULL,
          "past the last kind is a miss");
    CHECK(sensor_backend((sensor_kind_t)200) == NULL, "a wild kind is a miss");
  }

  printf("  [4] every entry is self-consistent\n");
  {
    /* Walk what the adapter walks. A descriptor with a task but no stack would
     * create a task with a zero-size stack, which is a crash at first use. */
    for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
      const sensor_driver_t *d = sensor_backend((sensor_kind_t)k);
      if (d == NULL) {
        continue;
      }
      if (d->task != NULL) {
        CHECK(d->stack_words > 0, "a task-owning driver states a stack size");
        CHECK(d->task_name != NULL, "and a task name");
      }
      if (d->tick != NULL) {
        CHECK(d->tick_period_us > 0, "a paced driver states a period");
      }
    }
  }

  printf("  [5] the ride-along section is walked the same way\n");
  {
    /* Same mechanism, second section. If the two sections were confused for
     * each other the counts would not be independent. */
    uint8_t n = 0;
    const sensor_ride_t *rides = sensor_rides(&n);
    CHECK(n > 0, "ride table is populated");
    for (uint8_t i = 0; i < n; i++) {
      CHECK(rides[i].ingest != NULL, "a ride has an ingest callback");
      CHECK(rides[i].present != NULL, "and a presence check");
      CHECK(rides[i].len > 0, "and reads a non-zero burst");
      CHECK(rides[i].every_n > 0, "and has a cadence");
    }
    printf("      %u ride-alongs registered\n", (unsigned)n);
  }

  printf("  [6] the IMU model resolves only through an IMU entry\n");
  {
    /* imu_ops() casts the descriptor's type-erased ops pointer, so the thing
     * that keeps it safe is that it goes through sensor_backend(SENSOR_IMU)
     * first. The host compiles no IMU driver, so there is no IMU entry and
     * the model must come back NULL rather than reinterpreting a barometer's
     * descriptor as an IMU's. */
    const sensor_driver_t *imu = sensor_backend(SENSOR_IMU);
    const imu_ops_t *ops = imu_ops();
    if (imu == NULL) {
      CHECK(ops == NULL, "no IMU backend means no model, not a bad cast");
    } else {
      CHECK(ops == (const imu_ops_t *)imu->ops, "the model is that entry's");
    }
    /* A baro entry carrying ops must never be reachable as an IMU. */
    const sensor_driver_t *baro = sensor_backend(SENSOR_BARO);
    if (baro != NULL && baro->ops != NULL) {
      CHECK((const void *)ops != baro->ops, "a baro's ops are not the IMU's");
    }
  }

  printf("  [7] the baro model carries humidity, and nothing the hub has\n");
  {
    /* The model used to hand back a whole reading -- pressure, temperature and
     * a timestamp that the hub already carries, plus an altitude the hub is
     * the one place allowed to derive. Now it answers for humidity only, and
     * says so when it cannot. */
    const baro_ops_t *bar = baro_ops();
    CHECK(bar != NULL, "a baro backend exists on this build");
    CHECK(bar && bar->humidity != NULL, "it can report humidity");
    if (bar && bar->humidity) {
      float rh = -1.0f;
      CHECK(bar->humidity(&rh) != VAYU_OK, "no sample yet is reported");
      CHECK(rh == -1.0f, "and the caller's value is untouched");

      bme280_publish(96000.0f, 21.5f, 43.25f);
      CHECK(bar->humidity(&rh) == VAYU_OK, "a fed device answers");
      CHECK(rh == 43.25f, "humidity carried through");
      CHECK(bar->humidity(NULL) != VAYU_OK, "a NULL destination is refused");
    }
    if (bar != NULL && imu_ops() != NULL) {
      CHECK((const void *)bar != (const void *)imu_ops(),
            "the baro and IMU models are different tables");
    }
  }

  printf(
      "  [8] pressure and temperature come from the hub, altitude derived\n");
  {
    /* What the BARO telemetry message is built from, in one place. The driver
     * used to hold its own copy of the ISA formula and its own sea-level
     * constant; they agreed with the hub's only by coincidence. */
    baro_sample_t s;
    CHECK(baro_latest(&s) && s.valid, "the hub has the measurement");
    CHECK(s.pressure_pa == 96000.0f, "pressure from the hub");
    CHECK(s.temp_c == 21.5f, "temperature from the hub");

    float alt = hub_altitude_m(s.pressure_pa, HUB_SEA_LEVEL_PA_DEFAULT);
    CHECK(alt > 0.0f, "below sea-level pressure derives a positive altitude");
    /* 96000 Pa is roughly 450 m on the ISA curve; a wrong exponent or datum
     * lands nowhere near it. */
    CHECK(alt > 300.0f && alt < 600.0f, "and a physically sane one");
    CHECK(hub_altitude_m(HUB_SEA_LEVEL_PA_DEFAULT, HUB_SEA_LEVEL_PA_DEFAULT) ==
              0.0f,
          "at the datum, altitude is zero");
    /* A zero or negative datum is a divide by zero, and the result would go
     * on the wire as the aircraft's altitude. It must fall back, not diverge. */
    CHECK(hub_altitude_m(s.pressure_pa, 0.0f) == alt,
          "a zero datum falls back to the default");
    CHECK(hub_altitude_m(s.pressure_pa, -5.0f) == alt,
          "so does a negative one");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
