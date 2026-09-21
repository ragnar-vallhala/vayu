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
 * sim/host/include/port.h
 *
 * Host port for vaios primitives. Shadows
 * extern/vaios/portable/cortex-m4/port.h on the SITL build. Replaces
 * the cortex-m4 BASEPRI/CPSID critical-section primitives with a
 * single pthread mutex. Good enough because the host build does not
 * need real-time interrupt masking - it only needs serialized access
 * to kernel data shared between pthreads.
 */
#ifndef VAIOS_HOST_PORT_H
#define VAIOS_HOST_PORT_H

#include <stdint.h>

extern volatile uint32_t critical_nesting;

#ifdef VAYU_SITL_RTOS
/* Phase 4 RTOS path: the real vaios scheduler runs cooperatively in ONE OS
 * thread (ucontext), so critical sections need no interrupt masking — a nesting
 * counter satisfies the kernel's enter/exit balance, and the FromISR form uses
 * the same save/restore signature the kernel (ipc.c) expects. */
static inline void v_enter_critical(void) { critical_nesting++; }
static inline void v_exit_critical(void) {
  if (critical_nesting)
    critical_nesting--;
}
static inline uint32_t v_enter_critical_from_isr(void) { return 0; }
static inline void v_exit_critical_from_isr(uint32_t saved) { (void)saved; }

#define ENTER_CRITICAL() v_enter_critical()
#define EXIT_CRITICAL() v_exit_critical()
#define ENTER_CRITICAL_FROM_ISR() v_enter_critical_from_isr()
#define EXIT_CRITICAL_FROM_ISR(saved) v_exit_critical_from_isr(saved)

#else /* legacy pthread SITL */
#include <pthread.h>
extern pthread_mutex_t host_critical_mutex;

static inline void v_enter_critical(void) {
  pthread_mutex_lock(&host_critical_mutex);
  critical_nesting++;
}

static inline void v_exit_critical(void) {
  critical_nesting--;
  pthread_mutex_unlock(&host_critical_mutex);
}

#define ENTER_CRITICAL() v_enter_critical()
#define EXIT_CRITICAL() v_exit_critical()
#define ENTER_CRITICAL_FROM_ISR() v_enter_critical()
#define EXIT_CRITICAL_FROM_ISR() v_exit_critical()
#endif

/* Memory barrier macro used by extern/vaios/kernel/structure.c. On host
 * pthreads the compiler fence + atomic ordering implied by __sync_synchronize
 * gives ordering equivalent to ARM's `dmb`. */
#define V_PORT_MB() __sync_synchronize()

#endif /* VAIOS_HOST_PORT_H */
