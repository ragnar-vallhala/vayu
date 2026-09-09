/*
 * host_rtos_engine.c -- the reusable in-process RTOS SITL step engine.
 * See host_rtos_engine.h. Factored out of host_rtos_main.c (Phase 5,
 * firmware/docs/plans/sitl-rtos-consolidation.md) — pure refactor, no behaviour change.
 */
#define _GNU_SOURCE
#include "host_rtos_engine.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm/comm.h" /* ibus_data_t, rc_queue_control_push, rc_arm_engaged */
#include "control/angle_controller.h" /* angle_controller_set_gains */
#include "control/angle_rate_controller.h" /* angle_rate_controller_set_gains, _set_motor_geometry */
#include "host_rc_feeder.h" /* host_rc_feeder_start (serial RC for the GCS) */
#include "host_rtos.h"      /* host_rtos_tick, host_rtos_run_until_idle */
#include "sensor/bme280.h"  /* bme280_publish (in-process baro injection) */
#include "sys/state.h"      /* system_state_get/_set, SYSTEM_STATE_* */
#include "sys/sys_utils.h"  /* VAYU_DISCARD */
#include "vaios.h"     /* v_system_init, scheduler_start, vaios_init_config_t */
#include "variables.h" /* SYS_CLOCK_FREQ */

extern int vayu_sitl_start(void *iface);     /* host_lifecycle.c */
extern void increment_high_freq_timer(void); /* firmware HF timestamp */

/* in-process vsim physics (vsim_inproc.cpp) + PWM read-back (host_navhal.c) */
extern void vsim_inproc_step(const float duty[4], float dt,
                             uint8_t out_imu[88]);
extern int vsim_inproc_load_geometry(const char *path, float out_x[4],
                                     float out_y[4], int out_spin[4]);
extern void vsim_inproc_apply_actuator_env_default(void);
extern void vsim_inproc_get_baro(float *pressure_pa, float *temperature_c,
                                 float *humidity_rh);
extern void host_pwm_get_latest(float out[4]);

#define HF_PER_SAMPLE 10 /* HIGH_FREQ_TIMER_FREQ(10k) / SITL_IMU_FEED_HZ(1k) */
#define BARO_DECIM 20    /* 1 kHz step / 20 -> ~50 Hz baro (BME280-realistic) */

double env_f(const char *k, double dflt) {
  const char *v = getenv(k);
  return v && *v ? strtod(v, NULL) : dflt;
}

/* If VAYU_RTOS_GEOMETRY points at a serialized vsim_ctl_geometry_t, drive BOTH
 * the in-process physics and the firmware mix from it (one geometry source, as
 * on the realtime stack) — so the fast tuner flies the loaded airframe, not the
 * compiled reference quad. Returns 1 if a geometry file was loaded (which also
 * carried the actuator-imperfection envs), 0 otherwise — so the caller can apply
 * those envs against the reference quad when no geometry is present. */
static int apply_geometry_from_env(void) {
  const char *path = getenv("VAYU_RTOS_GEOMETRY");
  if (!path || !*path)
    return 0;
  float gx[4], gy[4];
  int gs[4];
  if (vsim_inproc_load_geometry(path, gx, gy, gs)) {
    angle_rate_controller_set_motor_geometry(gx, gy, gs);
    fprintf(
        stderr,
        "vayu_sitl_rtos: geometry from %s applied (physics + firmware mix)\n",
        path);
    return 1;
  }
  fprintf(stderr,
          "vayu_sitl_rtos: WARN could not read geometry %s — using reference "
          "quad\n",
          path);
  return 0;
}

/* Apply roll/pitch (axes 0,1) + optional yaw rate/angle gains from env, so an
 * external tuner can evaluate a gain set. Unset env -> firmware default / pid.bin. */
void apply_gains_from_env(void) {
  double rkp = env_f("VAYU_RATE_KP", -1), rki = env_f("VAYU_RATE_KI", -1),
         rkd = env_f("VAYU_RATE_KD", -1), akp = env_f("VAYU_ANGLE_KP", -1),
         ykp = env_f("VAYU_YAW_RATE_KP", -1);
  for (int ax = 0; ax <= 1; ax++) {
    if (rkp >= 0 || rki >= 0 || rkd >= 0) {
      angle_rate_controller_set_gains((uint8_t)ax,
                                      rkp >= 0 ? (float)rkp : 5e-4f,
                                      rki >= 0 ? (float)rki : 0.0033333f,
                                      rkd >= 0 ? (float)rkd : 0.0f, 0.0f);
    }
    if (akp >= 0)
      angle_controller_set_gains((uint8_t)ax, (float)akp, 0.0f, 0.0f, 0.0f);
  }
  if (ykp >= 0)
    angle_rate_controller_set_gains(2, (float)ykp, 0.0f, 0.0f, 0.0f);
}

void stepper_init(stepper_t *s) {
  memset(s, 0, sizeof *s); /* zero the IMU struct: garbage in unfilled fields
                              * (the parts packImu doesn't write) breaks
                              * determinism — found the hard way. */
}

/* Push one RC frame and apply the arm/disarm state machine the (absent) rc_task
 * would — same logic as host_rc_feeder. Lets the doublet scenario actually arm. */
void set_rc(int roll, int pitch, int thr, int yaw, int arm) {
  ibus_data_t rc;
  memset(&rc, 0, sizeof rc);
  rc.channels[0] = (uint16_t)roll;
  rc.channels[1] = (uint16_t)pitch;
  rc.channels[2] = (uint16_t)thr;
  rc.channels[3] = (uint16_t)yaw;
  rc.channels[4] = (uint16_t)arm;
  for (int i = 5; i < 14; i++)
    rc.channels[i] = 1500;
  rc.is_failsafe = false;
  sys_state_t cur = system_state_get();
  if (rc_arm_engaged(&rc)) {
    if (cur == SYSTEM_STATE_STANDBY && thr < 1100)
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
    else if (cur == SYSTEM_STATE_STANDBY && thr > 1100)
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  } else if (cur == SYSTEM_STATE_ARMED || cur == SYSTEM_STATE_FAILSAFE) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
  }
  rc_queue_control_push(&rc);
}

/* One deterministic sim step: physics under last PWM -> sample IMU -> inject ->
 * SysTick+1 -> run the real scheduler to idle (firmware writes PWM) -> read PWM
 * back. Drains the control trace to its latest into *ct (got=1 if any). */
void step_once(stepper_t *s, control_telemetry_t *ct, int *got) {
  vsim_inproc_step(s->duty, 0.001f, (uint8_t *)&s->sample.converted);
  s->cyc += (uint32_t)(SYS_CLOCK_FREQ / 1000);
  s->sample.converted.timestamp = s->cyc;
  imu_queue_control_push(&s->sample);
  imu_queue_telemetry_push(&s->sample);
  imu_queue_attitude_push(&s->sample);
  /* Feed the modelled barometer at ~50 Hz (mirrors the old host_baro FIFO
   * feeder, which is compiled out in this in-process build) so the firmware's
   * vertical estimator has altitude — otherwise VERT/climb telemetry is dead.
   * The attitude/rate loops don't read baro, so this leaves their determinism
   * fingerprints untouched. */
  static uint32_t baro_ctr = 0;
  if (++baro_ctr >= BARO_DECIM) {
    baro_ctr = 0;
    float pressure_pa, temp_c, hum_rh;
    vsim_inproc_get_baro(&pressure_pa, &temp_c, &hum_rh);
    bme280_publish(pressure_pa, temp_c, hum_rh);
  }
  host_rtos_tick(1);
  for (int h = 0; h < HF_PER_SAMPLE; h++)
    increment_high_freq_timer();
  host_rtos_run_until_idle();
  host_pwm_get_latest(s->duty);
  if (got)
    *got = 0;
  control_telemetry_t t;
  while (control_telemetry_queue_pop(&t)) {
    if (ct)
      *ct = t;
    if (got)
      *got = 1;
  }
}

/* ---- wall-clock pacer (#11b) ----------------------------------------- */
void rtos_pacer_init(rtos_pacer_t *p) {
  clock_gettime(CLOCK_MONOTONIC, &p->next);
  p->behind = 0;
}

void rtos_pacer_wait(rtos_pacer_t *p, double dt_s) {
  long ns = (long)(dt_s * 1e9 + 0.5);
  p->next.tv_nsec += ns;
  while (p->next.tv_nsec >= 1000000000L) {
    p->next.tv_nsec -= 1000000000L;
    p->next.tv_sec++;
  }
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  if (now.tv_sec > p->next.tv_sec ||
      (now.tv_sec == p->next.tv_sec && now.tv_nsec > p->next.tv_nsec)) {
    p->behind++; /* fell behind — reset baseline so we don't busy-spiral */
    p->next = now;
    return;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &p->next, NULL);
}

int rtos_engine_boot(void *iface) {
  static int booted = 0;
  if (booted)
    return 0; /* idempotent: the firmware/scheduler boot once per
                           * process (GCS Stop/Re-Start resumes the loop, doesn't
                           * re-init the kernel). */
  booted = 1;
  fprintf(stderr, "vayu_sitl_rtos: booting the REAL vaios scheduler on host\n");
  vaios_init_config_t cfg = {0};
  v_system_init(&cfg); /* heap + scheduler init */
  if (vayu_sitl_start(iface) !=
      0) { /* iface != NULL: telemetry via its UART2 cb */
    fprintf(stderr, "vayu_sitl_rtos: vayu_sitl_start failed\n");
    return 1;
  }
  scheduler_start(); /* run boot tasks to idle */
  /* after boot so the mix isn't re-init'd. With no geometry file, still honour
   * the actuator-imperfection envs against the reference quad (otherwise they'd
   * only apply on the geometry path) — so `disturb` can fly the identified plant. */
  if (!apply_geometry_from_env())
    vsim_inproc_apply_actuator_env_default();
  return 0;
}

void rtos_engine_enable_serial_rc(void) {
  host_rc_feeder_start(); /* reads VAYU_UART_RC_PATH; pushes RC + arm SM */
}

/* ---- type-free interactive run facade --------------------------------
 * Lets the GCS worker thread drive the engine without ever seeing the firmware
 * types (stepper_t / control_telemetry_t live only here). RC arrives via the
 * serial feeder thread (rtos_engine_enable_serial_rc), so the loop is just
 * step + pace; the worker reads pose via vsim_inproc_get_pose and pushes config
 * via the vsim_inproc_set_* surface between steps. */
static stepper_t g_run_stepper;
static rtos_pacer_t g_run_pacer;

void rtos_engine_run_begin(void) {
  stepper_init(&g_run_stepper);
  rtos_pacer_init(&g_run_pacer);
}

void rtos_engine_run_step(void) {
  control_telemetry_t ct;
  int got;
  step_once(&g_run_stepper, &ct, &got);
  rtos_pacer_wait(&g_run_pacer, 0.001); /* wall-clock pace to 1 ms/step */
}
