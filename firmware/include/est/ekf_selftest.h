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
 * @file ekf_selftest.h
 * @brief Reporter-agnostic correctness self-test for the attitude EKF.
 *
 * One set of branch-coverage scenarios driven through the real EKF API, with
 * results delivered via a caller-supplied callback. This lets the SAME checks
 * run two ways:
 *   - HOST: sim/host/tests/test_phase3_est_ekf.c adapts the callback to
 *     the CHECK()/ctest harness (native gcc).
 *   - FIRMWARE: src/main.c (built with -DEKF_SELFTEST) adapts it to vayu_log,
 *     so the suite runs on target and reports over UART.
 *
 * The implementation body is compiled only when EKF_SELFTEST is defined, so a
 * normal firmware build carries none of it.
 */
#ifndef VAYU_EST_EKF_SELFTEST_H
#define VAYU_EST_EKF_SELFTEST_H

#include <stdbool.h>

/**
 * @brief Per-check result sink.
 * @param ctx   opaque caller context (counters, etc.).
 * @param pass  whether this check passed.
 * @param name  short check description.
 */
typedef void (*ekf_selftest_report_fn)(void *ctx, bool pass, const char *name);

/**
 * @brief Run every EKF branch-correctness scenario.
 * @return number of failed checks (0 = all pass).
 *
 * When built without EKF_SELFTEST this reports a single "not built" failure so
 * a misconfigured test build is loud rather than silently green.
 */
int ekf_selftest_run(ekf_selftest_report_fn report, void *ctx);

#endif /* VAYU_EST_EKF_SELFTEST_H */
