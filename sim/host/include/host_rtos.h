/*
 * host_rtos.h -- API for the real-vaios-scheduler SITL (Phase 4).
 * Implemented in host_rtos_port.c; only used by the vayu_sitl_rtos target.
 */
#ifndef HOST_RTOS_H
#define HOST_RTOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Resume the cooperative scheduler and run until it goes idle (all tasks
 * parked) — the idle task's cpu_relax swaps back here. On return the tick has
 * fully settled (PWM written). */
void host_rtos_run_until_idle(void);

/* Advance the SysTick clock `ms` ticks, waking due delayed/timed-out tasks.
 * Call before host_rtos_run_until_idle() to step sim time forward. */
void host_rtos_tick(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HOST_RTOS_H */
