/*
 * host_rtos_main.c -- entry point for the experimental vayu_sitl_rtos binary
 * (Phase 4, docs/plans/sitl-lockstep-sim.md). Boots the REAL vaios scheduler on
 * the host via the ucontext port (host_rtos_port.c), creates the firmware tasks,
 * and steps the cooperative scheduler a few sim ticks to prove it runs.
 *
 * This is the step-1b milestone: link the real kernel + boot. The single-
 * threaded sensor stepper (feeding IMU/baro/RC and reading PWM per tick) is
 * step 2; here there is no sensor input, so tasks simply park on the scheduler
 * (attitude on its IMU semaphore, the loops on their tick delays) — which is
 * exactly what proves the scheduler + context switches work end to end.
 */
#define _GNU_SOURCE
#include <stdio.h>

#include "host_rtos.h"
#include "sys/state.h"
#include "vaios.h"

extern int vayu_sitl_start(void *iface);   /* host_lifecycle.c */
extern uint32_t get_context_switch_count(void);  /* kernel task.c */

int main(void) {
  fprintf(stderr, "vayu_sitl_rtos: booting the REAL vaios scheduler on host\n");

  /* Kernel bring-up: heap + scheduler (idle task) init, all behind the host
   * port's v_port_hw_* stubs. Must precede any task_create / semaphore_create. */
  vaios_init_config_t cfg = {0};
  v_system_init(&cfg);

  /* Create the firmware tasks on the real scheduler (no feeders in RTOS mode). */
  if (vayu_sitl_start(NULL) != 0) {
    fprintf(stderr, "vayu_sitl_rtos: vayu_sitl_start failed\n");
    return 1;
  }

  fprintf(stderr, "vayu_sitl_rtos: tasks created, starting scheduler\n");
  scheduler_start();                 /* enter first task; returns when idle */

  /* Step the sim clock; each tick wakes due delay/timeout tasks, then we run the
   * scheduler to quiescence. With no sensors the loops just tick and re-park. */
  for (int i = 0; i < 50; i++) {
    host_rtos_tick(1);               /* +1 ms SysTick */
    host_rtos_run_until_idle();
  }

  fprintf(stderr,
          "vayu_sitl_rtos: booted OK. state=0x%x  context_switches=%u\n",
          (unsigned)system_state_get(), get_context_switch_count());
  return 0;
}
