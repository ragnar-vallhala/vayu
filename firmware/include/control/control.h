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
 * @file control.h
 * @brief Public umbrella header for the control module (CTRL).
 *
 * @implements R2.1
 *
 * Single public entry point for the control subsystem: the angle and
 * angle-rate controllers, live PID-gain configuration, the PID core, and
 * the control-telemetry buffer. External modules include only this header.
 */
#ifndef VAYU_CONTROL_H
#define VAYU_CONTROL_H

#include "control/angle_controller.h"
#include "control/angle_rate_controller.h"
#include "control/control_buffer.h"
#include "control/pid.h"
#include "control/pid_config.h"

#endif // VAYU_CONTROL_H
