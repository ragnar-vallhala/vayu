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
/* The sensor adapter with an EMPTY registry.
 *
 * Sibling of test_sensor_adapter.c, and the difference is the LINK: this one
 * takes vayu_sitl_core without --whole-archive, so the linker pulls no driver
 * object out of the archive and the vayu_sensors section never exists. The
 * section-bound symbols are weak, so they resolve to NULL rather than failing
 * the link -- and every accessor then has to cope with NULL bounds instead of
 * differencing two null pointers and walking from address zero.
 *
 * This is not a contrived case. A host test binary that touches the adapter
 * lands here by default, and so would any future build that put the drivers
 * in an archive. The registry being empty is survivable; reading off address
 * zero to discover that is not. */
#include <stdio.h>

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
  printf("== sensor adapter, empty registry ==\n");

  printf("  [1] an empty registry counts zero, it does not crash\n");
  CHECK(sensor_count() == 0, "count is zero");

  printf("  [2] every kind misses, cleanly\n");
  for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
    CHECK(sensor_backend((sensor_kind_t)k) == NULL, "kind returns NULL");
  }

  printf("  [3] starting a task on a missing backend is refused\n");
  CHECK(sensor_start_task(SENSOR_IMU) != VAYU_OK, "refused, not attempted");

  printf("  [4] probing an empty registry reports nothing brought up\n");
  /* And logs that it found no drivers -- on real hardware that is a build
   * fault in runtime clothing, and it means arming with no gyro. */
  CHECK(sensor_probe_all() == 0, "zero sensors probed OK");

  printf("  [5] the ride table is empty too, and safe to ask\n");
  {
    uint8_t n = 200;
    (void)sensor_rides(&n);
    CHECK(n == 0, "no ride-alongs");
  }

  printf("  [6] the IMU model is absent, not a null-deref waiting to happen\n");
  {
    /* The command layer calls imu_ops() and checks for NULL before every use;
     * this is the build where that branch is the live one. */
    CHECK(imu_ops() == NULL, "no registry means no IMU model");
    CHECK(baro_ops() == NULL, "nor a baro model");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
