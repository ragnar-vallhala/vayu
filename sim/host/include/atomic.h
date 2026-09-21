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
#ifndef VAIOS_ATOMIC_H
#define VAIOS_ATOMIC_H
// Host-native atomic_t for the SITL build. vaios keeps atomic_t in the port
// layer (portable/<arch>/atomic.h), so the host port supplies its own — same
// plain-ops shape the host test stub uses. The SITL scheduler drives the
// firmware tasks cooperatively, so a plain volatile counter is sufficient
// (no LDREX/STREX, which is Cortex-M only).
#include <stdint.h>

typedef struct atomic {
  volatile int32_t counter;
} atomic_t;

static inline void atomic_set(atomic_t *v, int32_t i) { v->counter = i; }
static inline int32_t atomic_get(atomic_t *v) { return v->counter; }
static inline void atomic_inc(atomic_t *v) { v->counter++; }
static inline void atomic_dec(atomic_t *v) { v->counter--; }

#endif // VAIOS_ATOMIC_H
