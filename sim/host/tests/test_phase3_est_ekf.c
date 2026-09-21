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
 * @file sim/host/tests/test_phase3_est_ekf.c
 * @brief SITL verification suite for the attitude EKF (SF_EKF / SF_EKF_ACCEL_BIAS).
 *
 * Thin host adapter over the shared, reporter-agnostic branch-coverage core in
 * src/est/ekf_selftest.c (compiled here with -DEKF_SELFTEST). The same core
 * runs on the ARM target via the -DEKF_SELFTEST firmware build (src/main.c),
 * so host and firmware exercise identical checks.
 *
 *   @verifies EST-EKF-001  EKF attitude estimator converges (SITL)
 *   @verifies EST-EKF-002  9-state accel-bias variant (SITL)
 *   @verifies EST-EKF-101  init / reset, 6-state vs 9-state selection
 *   @verifies EST-EKF-102  accel-direction tilt update (6-state convergence)
 *   @verifies EST-EKF-103  gyro-bias observability
 *   @verifies EST-EKF-104  specific-force update, accel-bias convergence (9-state)
 *   @verifies EST-EKF-105  accel-trust gate + mag-invalid skip + degenerate guard
 *   @verifies EST-EKF-106  quaternion finite + unit-norm under long soak
 */
#include <stdbool.h>
#include <stdio.h>

#include "est/ekf_selftest.h"

static int g_checks = 0;
static int g_fails = 0;

static void report(void *ctx, bool pass, const char *name) {
  (void)ctx;
  g_checks++;
  if (pass) {
    printf("    ok   %s\n", name);
  } else {
    g_fails++;
    printf("    FAIL %s\n", name);
  }
}

int main(void) {
  printf("== Phase-3 EST EKF SITL verification ==\n");
  printf("  ekf_selftest (EST-EKF-101..106)\n");

  ekf_selftest_run(report, NULL);

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
