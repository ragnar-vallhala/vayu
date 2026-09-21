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
#include "sys/math_utils.h"

uint16_t float32_to_float16(float f) {
  union {
    float f;
    uint32_t i;
  } u;
  u.f = f;

  uint16_t sign = (u.i >> 16) & 0x8000;
  int16_t exp = ((u.i >> 23) & 0xff);
  uint32_t mant = (u.i & 0x7fffff);

  if (exp == 0) { // Zero or subnormal
    return sign;
  } else if (exp == 0xff) { // Inf or NaN
    return sign | 0x7c00 | (mant ? 0x200 : 0);
  } else {
    int new_exp = (int)exp - 127 + 15;
    if (new_exp >= 31) { // Overflow
      return sign | 0x7c00;
    } else if (new_exp <= 0) { // Underflow
      return sign;
    } else {
      return sign | (uint16_t)(new_exp << 10) | (uint16_t)(mant >> 13);
    }
  }
}
