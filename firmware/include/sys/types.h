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
#ifndef VAYU_TYPES_H
#define VAYU_TYPES_H

typedef enum {
  NONE = 0,    // Invalid state recieved retry
  INVALID = 1, // Invalid state recieved retry
  ERROR = 2,   // Reinitialize and retry
  FAULT = 3,   // Kernel Panic (can't recover, reboot!)
  USAGE = 4,   // Incorrect usage don't retry
  EMPTY = 5    // No data to process
} err_t;

#endif //! VAYU_TYPES_H