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
#include <stdint.h>   /* before <stdlib.h>: glibc stdlib.h uses int32_t */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "comm/comm.h"        /* ibus_data_t, rc_queue_control_push */
#include "est/est.h"          /* attitude_t */
#include "host_rtos.h"
#include "sensor/imu_buffer.h" /* attitude_queue_telemetry_peek */
#include "sensor/sensor.h"    /* bmx160_all_reading_t, imu_queue_*_push */
#include "sys/state.h"
#include "vaios.h"
#include "variables.h"        /* SYS_CLOCK_FREQ */

extern int vayu_sitl_start(void *iface);          /* host_lifecycle.c */
extern uint32_t get_context_switch_count(void);   /* kernel task.c */
extern void increment_high_freq_timer(void);      /* firmware HF timestamp */

/* in-process vsim physics (vsim_inproc.cpp) + PWM read-back (host_navhal.c) */
extern void vsim_inproc_reset(uint32_t seed);
extern void vsim_inproc_step(const float duty[4], float dt, uint8_t out_imu[76]);
extern void host_pwm_get_latest(float out[4]);

#define HF_PER_SAMPLE 10   /* HIGH_FREQ_TIMER_FREQ(10k) / SITL_IMU_FEED_HZ(1k) */
#define STEP_SEED     12345u

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

  /* ---- single-threaded, in-process stepper (Phase 4 step 2a) ----------
   * Physics is linked in (vsim_inproc.cpp), so each sample is pure CPU with no
   * FIFO round-trip: step physics under the last PWM → sample IMU → inject →
   * SysTick +1 → run the cooperative scheduler to idle (firmware writes PWM) →
   * read that PWM back inline → repeat. One process, one thread, seeded — fast,
   * faithful (PWM never stale), and deterministic. */
  /* Seed + sample count are env-overridable so the determinism validator can
   * vary them (same seed -> identical; different seed -> different). */
  const char *seed_env = getenv("VAYU_RTOS_SEED");
  const char *nenv = getenv("VAYU_RTOS_SAMPLES");
  const uint32_t seed = seed_env && *seed_env ? (uint32_t)strtoul(seed_env, 0, 10)
                                              : STEP_SEED;
  const int N = nenv && *nenv ? atoi(nenv) : 5000;   /* default 5 s at 1 kHz */

  vsim_inproc_reset(seed);
  float duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bmx160_all_reading_t sample;
  memset(&sample, 0, sizeof sample);   /* rule out uninitialised fields */
  uint32_t cyc = 0;
  double imu_fp = 0.0;                  /* fingerprint of the vsim IMU input */

  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  int n = 0;
  double fp = 0.0;                   /* determinism fingerprint (estimator output) */
  attitude_t att = {0};
  for (; n < N; n++) {
    inject_neutral_rc();
    vsim_inproc_step(duty, 0.001f, (uint8_t *)&sample.converted);  /* 76B payload */
    cyc += (uint32_t)(SYS_CLOCK_FREQ / 1000);
    sample.converted.timestamp = cyc;
    imu_fp += (double)sample.converted.acc[0] + sample.converted.gyr[0] +
              sample.converted.mag[0];
    imu_queue_control_push(&sample);
    imu_queue_telemetry_push(&sample);
    imu_queue_attitude_push(&sample);
    host_rtos_tick(1);               /* +1 ms SysTick → wake delayed loops */
    for (int h = 0; h < HF_PER_SAMPLE; h++)
      increment_high_freq_timer();   /* HF timestamp tracks sim time */
    host_rtos_run_until_idle();      /* run firmware to quiescence (PWM written) */
    host_pwm_get_latest(duty);       /* read back the motor output for next step */
    if (attitude_queue_telemetry_peek(&att))
      fp += (double)att.roll + (double)att.pitch + (double)att.yaw;
  }
  fprintf(stderr, "vayu_sitl_rtos: IMU input fp = %.10g | attitude fp = %.10g "
          "(final r/p/y = %.5f/%.5f/%.5f)\n", imu_fp, fp, att.roll, att.pitch,
          att.yaw);
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

  /* Machine-parseable result for the determinism validator. The two
   * fingerprints are exact functions of (seed, sample count): identical across
   * runs of the same seed, different across seeds. */
  printf("#RTOS-RESULT seed=%u samples=%d imu_fp=%.17g att_fp=%.17g "
         "wall=%.6f speedup=%.2f\n",
         seed, n, imu_fp, fp, wall, wall > 0 ? sims / wall : 0.0);
  fflush(stdout);
  return 0;
}
