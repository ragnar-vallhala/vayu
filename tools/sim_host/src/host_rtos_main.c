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
#include <string.h>
#include <time.h>

#include "comm/comm.h"        /* ibus_data_t, rc_queue_control_push */
#include "host_imu_feeder.h"  /* host_imu_feeder_open / _pump */
#include "host_rtos.h"
#include "sys/state.h"
#include "vaios.h"

extern int vayu_sitl_start(void *iface);          /* host_lifecycle.c */
extern uint32_t get_context_switch_count(void);   /* kernel task.c */
extern void increment_high_freq_timer(void);      /* firmware HF timestamp */

#define HF_PER_SAMPLE 10   /* HIGH_FREQ_TIMER_FREQ(10k) / SITL_IMU_FEED_HZ(1k) */

/* Constant neutral RC (centred sticks, idle throttle, disarmed) pushed each
 * tick so the control loops have an input. Step-2 milestone keeps it constant;
 * harness-driven RC (arming, doublets) is a follow-on. */
static void inject_neutral_rc(void) {
  ibus_data_t rc;
  memset(&rc, 0, sizeof rc);
  rc.channels[0] = 1500;  /* roll  */
  rc.channels[1] = 1500;  /* pitch */
  rc.channels[2] = 1000;  /* throttle (idle) */
  rc.channels[3] = 1500;  /* yaw   */
  rc.channels[4] = 1000;  /* arm switch low (disarmed) */
  for (int i = 5; i < 14; i++)
    rc.channels[i] = 1500;
  rc.is_failsafe = false;
  rc_queue_control_push(&rc);
}

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
  scheduler_start();                 /* run boot tasks to idle (all parked) */

  /* ---- single-threaded sensor stepper (Phase 4 step 2) ----------------
   * Lockstep with vsim_d: read one IMU sample, inject it (wakes the estimator),
   * advance the SysTick one tick (wakes the control loops), run the scheduler to
   * idle (the firmware fully processes the sample and writes PWM), repeat. vsim
   * reads the PWM, steps physics, emits the next IMU — a deterministic 1:1
   * handshake driven entirely from this one thread (no pthread interleaving). */
  fprintf(stderr, "vayu_sitl_rtos: stepper waiting for vsim IMU producer...\n");
  if (host_imu_feeder_open() < 0) {
    fprintf(stderr, "vayu_sitl_rtos: IMU FIFO open failed\n");
    return 1;
  }

  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  const int N = 5000;                /* 5 s of sim time at 1 kHz */
  int n = 0;
  for (; n < N; n++) {
    inject_neutral_rc();
    if (!host_imu_feeder_pump())      /* blocks for one vsim IMU frame */
      break;                          /* producer closed */
    host_rtos_tick(1);               /* +1 ms SysTick → wake delayed loops */
    for (int h = 0; h < HF_PER_SAMPLE; h++)
      increment_high_freq_timer();   /* HF timestamp tracks sim time */
    host_rtos_run_until_idle();      /* run firmware to quiescence (PWM written) */
    if ((n + 1) % 1000 == 0)
      fprintf(stderr, "vayu_sitl_rtos: stepped %d samples (t=%u ms)\n",
              n + 1, v_get_ticks());
  }
  struct timespec t1;
  clock_gettime(CLOCK_MONOTONIC, &t1);
  double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
  double sims = n / 1000.0;          /* 1 sample = 1 ms sim */

  fprintf(stderr,
          "vayu_sitl_rtos: stepped %d samples (%.1f s sim) in %.2f s wall "
          "= %.1fx realtime. state=0x%x ctx_switches=%u t=%u ms\n",
          n, sims, wall, wall > 0 ? sims / wall : 0.0,
          (unsigned)system_state_get(), get_context_switch_count(),
          v_get_ticks());
  return 0;
}
