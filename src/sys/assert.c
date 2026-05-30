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
#include "utils.h"          /* vaios v_panic (extern/vaios/include/utils.h) */
#include "logger/logger.h"    /* vayu_log */

void vayu_assert_fail(const char *file, int line, const char *expr) {
    /* Log first so the failure site is captured regardless of which
     * branch handles the halt. vayu_log enqueues to an OVERWRITE
     * lock-free ring — safe from any context including ISRs. */
    vayu_log("[VAYU_ASSERT] %s:%d  cond: %s  state=0x%x",
             file, line, expr, (unsigned)system_state_get());

#ifdef NDEBUG
    /* R9.2 release branch: request FAILSAFE, then trap the calling
     * task. Other tasks (motor task, telemetry) continue to run and
     * observe the FAILSAFE transition. */
    /* FAILSAFE always allowed by system_state_set; cast suppresses
     * the warn_unused_result attribute since there is no caller to
     * propagate to from inside an assertion handler. */
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
    for (;;) {
        __asm__ volatile ("nop");
    }
#else
    /* Debug branch: hand off to vaios v_panic which prints the site
     * over the serial console and halts the system. */
    v_panic(file, line, "VAYU_ASSERT failed: %s", expr);
    /* Unreachable; satisfy noreturn. */
    for (;;) {
        __asm__ volatile ("nop");
    }
#endif
}
