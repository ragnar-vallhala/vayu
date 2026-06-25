/**
 * @file vayu_status.h
 * @brief Canonical vayu_status_t — return-code vocabulary for vayu.
 *
 * @implements CONV-01
 *
 * Single status type returned by any vayu function whose failure is
 * a recoverable runtime condition (queue empty, transient I/O fault,
 * timeout, parameter out of range, …). Programming-error invariants
 * use VAYU_ASSERT instead — see vayu_assert.h (CONV-02).
 *
 * Constraints (per docs/firmware/coding-guidelines.md):
 *   - R7.5 / R7.6: shared status vocabulary across the codebase.
 *   - R9.4: error codes must not be redefined; values are stable.
 *   - R4.4: enumeration has an explicit underlying width.
 *   - No transitive HAL / RTOS includes — safe from any TU including
 *     ISRs and from cross-cutting headers.
 */
#pragma once

#include <stdint.h>

typedef enum vayu_status {
    VAYU_OK            =  0, /**< Success. */
    VAYU_ERR_INVALID   = -1, /**< Invalid argument or invalid state for op. */
    VAYU_ERR_TIMEOUT   = -2, /**< Operation timed out. */
    VAYU_ERR_BUSY      = -3, /**< Resource currently in use; try later. */
    VAYU_ERR_RANGE     = -4, /**< Value outside accepted range. */
    VAYU_ERR_FAULT     = -5, /**< Hardware, transport, or peripheral fault. */
    VAYU_ERR_NOT_IMPL  = -6, /**< Not implemented on this build target. */
} vayu_status_t;

/* R4.4: width is bounded — guard against accidental drift if the enum
 * grows large literals later. ABI clients depend on int32_t-fit. */
_Static_assert(sizeof(vayu_status_t) <= sizeof(int32_t),
               "vayu_status_t must fit in int32_t");

/**
 * @brief Explicitly discard a vayu_status_t return value.
 *
 * Use only at call sites where ignoring failure is intentional and
 * safe (e.g. a FAILSAFE state-set that the validator always allows,
 * or a best-effort transition that another loop iteration will retry).
 *
 * Plain `(void)` casts do not suppress GCC's `warn_unused_result`
 * attribute on C functions; assignment to a local does.
 */
#define VAYU_DISCARD(expr)                                                     \
    do {                                                                       \
        vayu_status_t _vayu_unused = (expr);                                   \
        (void)_vayu_unused;                                                    \
    } while (0)
