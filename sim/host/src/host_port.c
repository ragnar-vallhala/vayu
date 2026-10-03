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
/*
 * host_port.c -- definitions for the vaios host port globals declared in
 * sim/host/include/port.h.
 */
#include "port.h"

pthread_mutex_t host_critical_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile uint32_t critical_nesting = 0;

/* Pend a context switch.
 *
 * kernel/bus.c calls this after a publish wakes a reader that was blocked on the
 * topic, so the woken task runs promptly instead of at the next tick. The legacy
 * SITL has no vaios scheduler to switch -- it runs the firmware's logic on host
 * threads -- so there is nothing to pend and nothing is lost by saying so.
 *
 * Deliberately NOT the cooperative swapcontext that host_rtos_port.c does: that
 * build runs the real scheduler and genuinely has to switch here. This one does
 * not, and the two must not be linked together (host_rtos_port.c defines
 * critical_nesting too). */
void v_port_trigger_pendsv(void) {}
