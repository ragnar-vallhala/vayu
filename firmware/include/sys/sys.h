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
