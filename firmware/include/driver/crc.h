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
/* The CRC32 peripheral.
 *
 * NOT thread-safe and deliberately so: the unit has one shared accumulator, so
 * two concurrent users corrupt each other. Serialisation is the caller's --
 * sys/sys_utils.c holds the mutex and is the only caller.
 *
 * @implements HAL-CRC-001 */
#ifndef VAYU_DRIVER_CRC_H
#define VAYU_DRIVER_CRC_H

#include <stdint.h>

/** CRC32 over `len` bytes, standard polynomial, init 0xFFFFFFFF. */
uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len);

#endif // VAYU_DRIVER_CRC_H
