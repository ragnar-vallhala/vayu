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
#ifndef VAYU_MATH_UTILS_H
#define VAYU_MATH_UTILS_H

#include <stdint.h>

/**
 * @brief Converts a 32-bit float to a 16-bit half-precision float (IEEE 754).
 *
 * @param f The 32-bit float to convert.
 * @return uint16_t The 16-bit float representation.
 *
 * @noreq numeric conversion helper (telemetry payload compression).
 */
uint16_t float32_to_float16(float f);

#endif // VAYU_MATH_UTILS_H
