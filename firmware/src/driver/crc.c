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
/* See driver/crc.h. Re-initialised per call because the unit keeps a running
 * accumulator across computes; without the reset the second CRC in a process
 * continues the first. */
#include "driver/crc.h"

#include "navhal.h"

/** @implements HAL-CRC-001 */
uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len) {
  hal_crc_config_t cfg = {.polynomial = HAL_CRC_POLY_CRC32,
                          .init_value = 0xFFFFFFFF};
  hal_crc_init(&cfg);
  return hal_crc_compute(data, len);
}
