/**
 * @file control.h
 * @brief Public umbrella header for the control module (CTRL).
 *
 * @implements R2.1
 *
 * Single public entry point for the control subsystem: the angle and
 * angle-rate controllers, live PID-gain configuration, the PID core, and
 * the control-telemetry buffer. External modules include only this header.
 *
 * The PID core (`pid.h`) and control-telemetry buffer (`control_buffer.h`)
 * were moved here from `maths/` per Phase 4 R2.6.
 */
#ifndef VAYU_CONTROL_H
#define VAYU_CONTROL_H

#include "control/angle_controller.h"
#include "control/angle_rate_controller.h"
#include "control/control_buffer.h"
#include "control/pid.h"
#include "control/pid_config.h"

#endif // VAYU_CONTROL_H
