/**
 * @file sys.h
 * @brief Public umbrella header for the system module (SYS).
 *
 * @implements R2.1
 *
 * Single public entry point for the system subsystem: the state machine,
 * boot/assert vocabulary, system utilities (timestamp/device/CRC), and
 * the low-level helpers (math helpers, timer callbacks, shared scalar types).
 */
#ifndef VAYU_SYS_H
#define VAYU_SYS_H

#include "sys/math_utils.h"
#include "sys/state.h"
#include "sys/sys_utils.h"
#include "sys/timer_callbacks.h"
#include "sys/types.h"

#endif // VAYU_SYS_H
