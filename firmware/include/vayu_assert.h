/**
 * @file vayu_assert.h
 * @brief VAYU_ASSERT — invariant assertion macro.
 *
 * @implements CONV-02
 *
 * Use for invariants that must hold by construction (programming
 * errors). Recoverable runtime conditions return vayu_status_t — see
 * vayu_status.h (CONV-01).
 *
 * Behaviour on failure (R9.2):
 *   - Debug build (NDEBUG not defined): log the site, then call
 *     v_panic() which traps + halts the system.
 *   - Release build (NDEBUG defined): log the site, request
 *     SYSTEM_STATE_FAILSAFE, and halt the calling task. FAILSAFE-aware
 *     tasks (motor task, etc.) continue executing and command zero
 *     motor output.
 *
 * ISR-safe: vayu_assert_fail() does not block — vayu_log uses an
 * OVERWRITE-policy lock-free queue and state_set is a single store.
 *
 * Distinct from TEST_ASSERT (test-only assertion in navtest/framework).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Defined in src/sys/assert.c. */
void vayu_assert_fail(const char *file, int line, const char *expr)
    __attribute__((noreturn));

#define VAYU_ASSERT(cond)                                                      \
    do {                                                                       \
        if (!(cond)) {                                                         \
            vayu_assert_fail(__FILE__, __LINE__, #cond);                       \
        }                                                                      \
    } while (0)

#ifdef __cplusplus
}
#endif
