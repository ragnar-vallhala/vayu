/*
 * tools/sim_host/include/port.h
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

#include <pthread.h>
#include <stdint.h>

extern pthread_mutex_t host_critical_mutex;
extern volatile uint32_t critical_nesting;

static inline void v_enter_critical(void) {
    pthread_mutex_lock(&host_critical_mutex);
    critical_nesting++;
}

static inline void v_exit_critical(void) {
    critical_nesting--;
    pthread_mutex_unlock(&host_critical_mutex);
}

#define ENTER_CRITICAL() v_enter_critical()
#define EXIT_CRITICAL()  v_exit_critical()
#define ENTER_CRITICAL_FROM_ISR() v_enter_critical()
#define EXIT_CRITICAL_FROM_ISR()  v_exit_critical()

/* Memory barrier macro used by extern/vaios/kernel/structure.c. On host
 * pthreads the compiler fence + atomic ordering implied by __sync_synchronize
 * gives ordering equivalent to ARM's `dmb`. */
#define V_PORT_MB() __sync_synchronize()

#endif /* VAIOS_HOST_PORT_H */
