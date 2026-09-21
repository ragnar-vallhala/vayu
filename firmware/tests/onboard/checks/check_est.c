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
/* Bench check: estimator self-test. Reuses the shared EKF branch-coverage
 * scenarios (ekf_selftest_run) as one bench item — same code the host and the
 * -DEKF_SELFTEST boot path run. */
#include "hwtest_runner.h"
#include "est/ekf_selftest.h"
#include <stdbool.h>

static void noop_report(void *ctx, bool pass, const char *name) {
  (void)ctx;
  (void)pass;
  (void)name;
}

/* @verifies EST-EKF-001, EST-EKF-101, EST-EKF-102, EST-EKF-103, EST-EKF-106 */
hw_result_t check_ekf_selftest(void) {
  int fails = ekf_selftest_run(noop_report, 0);
  return fails == 0 ? hw_pass(0.0f, "fails") : hw_fail((float)fails, "fails");
}
