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
#ifndef VAYU_HOVER_STORE_H
#define VAYU_HOVER_STORE_H

#include <stdbool.h>

/*
 * Persistence for the measured hover collective.
 *
 * Hover is the number the whole vertical stack keys off, and it is now measured
 * in flight (est/hover_estimate.h) rather than assumed. Without persistence that
 * measurement dies at every power cycle and the next lift-off opens at the
 * compiled guess again -- which is exactly the failure that put the airframe
 * into the ceiling. Storing it means the FIRST takeoff after a reboot already
 * uses what the last flight learned.
 *
 * Its own file, deliberately NOT a field in pid.bin: adding one there means
 * bumping PID_CONFIG_MAGIC, which resets every persisted tune on the card. A
 * hover estimate is not worth invalidating a tune for.
 *
 * Read directly via vfs_* at task start (the pattern pid_config.c uses -- the
 * kernel vfs ops take the global mutex, so it serialises against fs_owner's
 * writes). Written through fs_owner, the sole runtime SD writer, and only on
 * disarm: the flight's learned value is final by then, it is a naturally rare
 * event, and it keeps SD wear and queue pressure to one write per flight.
 */

/* 8.3 name -- FF_USE_LFN is 0, so a longer name fails vfs_open with -6. */
#define HOVER_STORE_PATH "0:hover.bin"
#define HOVER_STORE_MAGIC 0x52564F48u /* 'H''O''V''R' */

/* Load the persisted hover, or return `fallback` when there is no file, the
 * magic is wrong, or the value is outside the sane band. A corrupt or absent
 * store must degrade to the compiled guess, never to a wild value. */
float hover_store_load(float fallback);

/* Queue a save through fs_owner. Returns false if the queue was full (the value
 * is simply not persisted this time -- it is an optimisation, not state the
 * aircraft depends on). */
bool hover_store_save(float hover);

#endif /* VAYU_HOVER_STORE_H */
