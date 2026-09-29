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
/* Substituting one IMU backend for another.
 *
 * test_sensor_adapter.c proves the registry is walked, and removing the
 * rangefinder from a board proved a driver can be dropped. Neither proves the
 * thing the adapter exists for: that a DIFFERENT backend can take the same
 * slot and the rest of the tree neither knows nor changes.
 *
 * This file is compiled TWICE -- once with SWAP_IMU_B defined, once without --
 * and each build registers a different IMU descriptor into the vayu_sensors
 * section. Nothing else differs, and no test code selects a backend: both
 * binaries ask the adapter the same questions and each is told about the
 * driver its own link pulled in. That is the substitution, performed at the
 * only layer that performs it for real.
 *
 *   @verifies SNS-BOOT-001  the boot probe reaches a substituted backend
 *
 * The backends are fakes on purpose. A speculative driver for a chip nobody
 * has would be unverifiable against silicon and would still not exercise this
 * -- what is under test is the adapter's selection path, not a device.
 */
#include <stdio.h>
#include <string.h>

#include "sensor/baro.h"
#include "sensor/imu.h"
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

/* Which backend this build compiled in, and what it must resolve to. */
#ifdef SWAP_IMU_B
#define EXPECT_NAME "fakeimu-b"
#define EXPECT_MARK 0xB0u
#else
#define EXPECT_NAME "fakeimu-a"
#define EXPECT_MARK 0xA0u
#endif

/* Each fake records that IT ran, so "the right ops table" is checked by
 * calling through it rather than by comparing pointers alone. */
static unsigned g_mark = 0;
#ifndef SWAP_IMU_B
static unsigned g_cancelled = 0;
#endif
static unsigned g_probed = 0;

#ifndef SWAP_IMU_B
static vayu_status_t fake_a_cal_start(uint32_t imu_id, uint32_t type) {
  (void)imu_id;
  (void)type;
  g_mark = 0xA0u;
  return VAYU_OK;
}
static void fake_a_cal_cancel(void) { g_cancelled++; }
static bool fake_a_calibrating(void) { return false; }
static vayu_status_t fake_a_probe(void) {
  g_probed++;
  return VAYU_OK;
}

static const imu_ops_t fake_a_ops = {
    .calibrate_start = fake_a_cal_start,
    .calibrate_cancel = fake_a_cal_cancel,
    .calibrating = fake_a_calibrating,
};

VAYU_SENSOR_DRIVER(fake_imu_a) = {
    .name = "fakeimu-a",
    .kind = SENSOR_IMU,
    .probe = fake_a_probe,
    .ops = &fake_a_ops,
};
#else
/* Deliberately a MINIMAL backend: it offers calibrate_start and nothing else,
 * which imu.h says is a legal backend (a NULL entry is VAYU_ERR_NOT_IMPL at
 * the caller, not a link error). If swapping to a smaller backend broke the
 * command layer's assumptions, it would break here. */
static vayu_status_t fake_b_cal_start(uint32_t imu_id, uint32_t type) {
  (void)imu_id;
  (void)type;
  g_mark = 0xB0u;
  return VAYU_OK;
}
static vayu_status_t fake_b_probe(void) {
  g_probed++;
  return VAYU_OK;
}

static const imu_ops_t fake_b_ops = {
    .calibrate_start = fake_b_cal_start,
};

VAYU_SENSOR_DRIVER(fake_imu_b) = {
    .name = "fakeimu-b",
    .kind = SENSOR_IMU,
    .probe = fake_b_probe,
    .ops = &fake_b_ops,
};
#endif

int main(void) {
  printf("== sensor adapter, IMU substituted (%s) ==\n", EXPECT_NAME);

  printf("  [1] the IMU slot resolves to the backend THIS build registered\n");
  const sensor_driver_t *imu = sensor_backend(SENSOR_IMU);
  CHECK(imu != NULL, "an IMU backend is registered");
  CHECK(imu && imu->kind == SENSOR_IMU, "filed under SENSOR_IMU");
  CHECK(imu && strcmp(imu->name, EXPECT_NAME) == 0,
        "and it is the one this build compiled in");

  printf("  [2] the model comes from that descriptor, and calls into it\n");
  {
    /* Pointer identity is necessary but not sufficient -- a wrong cast can
     * still produce the right address. Call through and see who answers. */
    const imu_ops_t *ops = imu_ops();
    CHECK(ops != NULL, "the model resolves");
    CHECK(imu && ops == (const imu_ops_t *)imu->ops,
          "the model is that descriptor's ops table");
    CHECK(ops && ops->calibrate_start != NULL, "it can start a calibration");
    if (ops && ops->calibrate_start) {
      g_mark = 0;
      CHECK(ops->calibrate_start(1u, 0u) == VAYU_OK, "the call succeeds");
      CHECK(g_mark == EXPECT_MARK, "and THIS build's backend is what ran");
    }
  }

  printf("  [3] a backend that omits an op is legal, not a link error\n");
  {
    /* The whole reason the model is a table of pointers. Backend B leaves
     * cancel and calibrating NULL; the caller branches, nothing fails to
     * link, and the caller can tell the difference. */
    const imu_ops_t *ops = imu_ops();
#ifdef SWAP_IMU_B
    CHECK(ops && ops->calibrate_cancel == NULL, "a minimal backend omits it");
    CHECK(ops && ops->calibrating == NULL, "and omits this one too");
#else
    CHECK(ops && ops->calibrate_cancel != NULL, "a full backend offers it");
    CHECK(ops && ops->calibrating != NULL, "and this one");
    if (ops && ops->calibrate_cancel) {
      g_cancelled = 0;
      ops->calibrate_cancel();
      CHECK(g_cancelled == 1, "and cancel reaches the backend");
    }
#endif
  }

  printf("  [4] swapping the IMU disturbs no other kind\n");
  {
    /* The point of the adapter: the barometer and rangefinder entries are
     * whatever this build has, unchanged by which IMU sits beside them, and
     * still filed under their own kinds. */
    const sensor_driver_t *baro = sensor_backend(SENSOR_BARO);
    const sensor_driver_t *rng = sensor_backend(SENSOR_RANGE);
    CHECK(baro != NULL && baro->kind == SENSOR_BARO, "baro still resolves");
    CHECK(rng != NULL && rng->kind == SENSOR_RANGE, "range still resolves");
    CHECK(baro && strcmp(baro->name, "bme280") == 0, "and is the same baro");
    printf("      imu=%s baro=%s range=%s\n", imu ? imu->name : "-",
           baro ? baro->name : "-", rng ? rng->name : "-");
  }

  printf("  [5] the IMU and baro models stay distinct tables\n");
  {
    /* ops is type-erased; the guard against reinterpreting one kind's table
     * as another's is that the accessor resolves the kind first. */
    const baro_ops_t *bar = baro_ops();
    CHECK(bar != NULL, "a baro model exists");
    CHECK((const void *)bar != (const void *)imu_ops(),
          "a baro's ops are not the IMU's");
  }

  printf("  [6] the boot probe reaches the substituted backend\n");
  {
    /* sensor_probe_all() is what main() calls; until now no host build had an
     * IMU entry for it to walk into. The marker is set by the fake's own
     * probe, so this fails if the boot path skips the IMU kind, probes the
     * wrong descriptor, or never reaches this object file's section entry. */
    g_probed = 0;
    uint8_t ok = sensor_probe_all();
    CHECK(g_probed == 1u, "the registered IMU was probed exactly once");
    CHECK(ok >= 1u, "and counted among those that came up");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
