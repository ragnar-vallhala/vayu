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
#ifndef VAYU_NAVLINK_ROUTER_H
#define VAYU_NAVLINK_ROUTER_H

/* The single home for the firmware's NavLink v2 receive handler table.
 *
 * Every decoded leaf the GCS sends lands in the one handler struct owned by
 * navlink_router.c; the generated codec is included only there, so the rest of
 * the comm layer never sees a navlink_* type. Leaves with no specific handler
 * hit the default handler — a 1 s, 10 Hz blue-LED blink (non-reentrant: a second
 * trigger while a blink is running is ignored). Real handlers (e.g. CMD_SET_PID)
 * are registered in navlink_router_init(); switching one is a one-line edit
 * there. See navlink/INTEGRATION.md. */
void navlink_router_init(
    void); /* build the parser + handler table; call once */
void navlink_router_poll(
    void); /* drain RX bytes -> dispatch; service the blink */

#endif /* VAYU_NAVLINK_ROUTER_H */
