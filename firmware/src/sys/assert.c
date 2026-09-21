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
 * @file src/sys/assert.c
 * @brief vayu_assert_fail() — invariant-violation handler.
 *
 * @implements CONV-02
 *
 * Two-mode behaviour per R9.2 — see vayu_assert.h for the contract.
 */
#include "vayu_assert.h"

#include "sys/state.h"
#include "utils.h"            /* vaios v_panic (extern/vaios/include/utils.h) */
#include "storage/fs_owner.h" /* vayu_log */

void vayu_assert_fail(const char *file, int line, const char *expr) {
  /* Log first so the failure site is captured regardless of which
     * branch handles the halt. vayu_log enqueues to an OVERWRITE
     * lock-free ring — safe from any context including ISRs. */
  vayu_log("[VAYU_ASSERT] %s:%d  cond: %s  state=0x%x", file, line, expr,
           (unsigned)system_state_get());

#ifdef NDEBUG
  /* R9.2 release branch: request FAILSAFE, then trap the calling
     * task. Other tasks (motor task, telemetry) continue to run and
     * observe the FAILSAFE transition. */
  /* FAILSAFE always allowed by system_state_set; cast suppresses
     * the warn_unused_result attribute since there is no caller to
     * propagate to from inside an assertion handler. */
  VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  for (;;) {
    __asm__ volatile("nop");
  }
#else
  /* Debug branch: hand off to vaios v_panic which prints the site
     * over the serial console and halts the system. */
  v_panic(file, line, "VAYU_ASSERT failed: %s", expr);
  /* Unreachable; satisfy noreturn. */
  for (;;) {
    __asm__ volatile("nop");
  }
#endif
}
