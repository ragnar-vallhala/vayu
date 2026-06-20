/*
 * host_rtos_main.c -- entry point for vayu_sitl_rtos (Phase 4,
 * docs/plans/sitl-lockstep-sim.md). Runs the REAL vaios scheduler on the host
 * via the ucontext port, with vsim physics linked in-process, stepped from one
 * thread — so SITL is deterministic and ~60x realtime.
 *
 * Two scenarios (env VAYU_RTOS_SCENARIO):
 *   hold     (default) constant neutral RC; emits determinism fingerprints
 *                      (see tools/sim_host/validate_rtos_determinism.py).
 *   doublet            arm + a roll/pitch/yaw step-doublet with env-set PID
 *                      gains; scores rate-loop tracking corr(rate_sp,rate_curr)
 *                      — a fast, deterministic autotune-style eval. Sim-time
 *                      paced (the stepper owns the clock; no wall-clock sleeps).
 */
#define _GNU_SOURCE
#include <math.h>
#include <stdint.h>   /* before <stdlib.h>: glibc stdlib.h uses int32_t */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "comm/comm.h"        /* ibus_data_t, rc_queue_control_push, rc_arm_engaged */
#include "control/angle_controller.h"       /* angle_controller_set_gains */
#include "control/angle_rate_controller.h"  /* angle_rate_controller_set_gains */
#include "control/control_buffer.h"          /* control_telemetry_t, _queue_pop */
#include "est/est.h"          /* attitude_t */
#include "host_rtos.h"
#include "sensor/imu_buffer.h" /* attitude_queue_telemetry_peek */
#include "sensor/sensor.h"    /* bmx160_all_reading_t, imu_queue_*_push */
#include "sys/state.h"
#include "sys/sys_utils.h"    /* VAYU_DISCARD */
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

/* ---- shared per-sample state for the stepper ------------------------- */
typedef struct {
  float duty[4];
  bmx160_all_reading_t sample;
  uint32_t cyc;
} stepper_t;

static void stepper_init(stepper_t *s) {
  memset(s, 0, sizeof *s);   /* zero the IMU struct: garbage in unfilled fields
                              * (the parts packImu doesn't write) breaks
                              * determinism — found the hard way. */
}

static double env_f(const char *k, double dflt) {
  const char *v = getenv(k);
  return v && *v ? atof(v) : dflt;
}

/* Push one RC frame and apply the arm/disarm state machine the (absent) rc_task
 * would — same logic as host_rc_feeder. Lets the doublet scenario actually arm. */
static void set_rc(int roll, int pitch, int thr, int yaw, int arm) {
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
static void step_once(stepper_t *s, control_telemetry_t *ct, int *got) {
  vsim_inproc_step(s->duty, 0.001f, (uint8_t *)&s->sample.converted);
  s->cyc += (uint32_t)(SYS_CLOCK_FREQ / 1000);
  s->sample.converted.timestamp = s->cyc;
  imu_queue_control_push(&s->sample);
  imu_queue_telemetry_push(&s->sample);
  imu_queue_attitude_push(&s->sample);
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

/* ---- streaming Pearson correlation ----------------------------------- */
typedef struct { double n, sx, sy, sxx, syy, sxy; } corr_t;
static void corr_add(corr_t *c, double x, double y) {
  c->n++; c->sx += x; c->sy += y; c->sxx += x * x; c->syy += y * y; c->sxy += x * y;
}
static double corr_val(const corr_t *c) {
  if (c->n < 2) return 0.0;
  double n = c->n;
  double cov = c->sxy - c->sx * c->sy / n;
  double vx = c->sxx - c->sx * c->sx / n, vy = c->syy - c->sy * c->sy / n;
  double d = sqrt(vx * vy);
  return d > 1e-12 ? cov / d : 0.0;
}

/* Apply roll/pitch (axes 0,1) + optional yaw rate/angle gains from env, so an
 * external tuner can evaluate a gain set. Unset env -> firmware default / pid.bin. */
static void apply_gains_from_env(void) {
  double rkp = env_f("VAYU_RATE_KP", -1), rki = env_f("VAYU_RATE_KI", -1),
         rkd = env_f("VAYU_RATE_KD", -1), akp = env_f("VAYU_ANGLE_KP", -1),
         ykp = env_f("VAYU_YAW_RATE_KP", -1);
  for (int ax = 0; ax <= 1; ax++) {
    if (rkp >= 0 || rki >= 0 || rkd >= 0) {
      angle_rate_controller_set_gains(
          (uint8_t)ax, rkp >= 0 ? (float)rkp : 5e-4f,
          rki >= 0 ? (float)rki : 0.0033333f, rkd >= 0 ? (float)rkd : 0.0f, 0.0f);
    }
    if (akp >= 0)
      angle_controller_set_gains((uint8_t)ax, (float)akp, 0.0f, 0.0f, 0.0f);
  }
  if (ykp >= 0)
    angle_rate_controller_set_gains(2, (float)ykp, 0.0f, 0.0f, 0.0f);
}

/* ---- doublet rollout: arm, hover, step each axis, score rate tracking - */
static int run_doublet(uint32_t seed) {
  vsim_inproc_reset(seed);
  apply_gains_from_env();
  stepper_t s;
  stepper_init(&s);
  control_telemetry_t ct;
  int got;
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  const int settle = 600, hold = 250, ret = 350;
  const int hover = 1500, step = 1800;   /* ~21 deg angle command */

  /* Arm (throttle low + arm high), then settle at hover. */
  for (int i = 0; i < 200; i++) set_rc(1500, 1500, 1000, 1500, 2000), step_once(&s, &ct, &got);
  for (int i = 0; i < settle; i++) set_rc(1500, 1500, hover, 1500, 2000), step_once(&s, &ct, &got);

  corr_t cr = {0}, cp = {0}, cy = {0};
  /* roll doublet */
  for (int i = 0; i < hold; i++) { set_rc(step, 1500, hover, 1500, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cr, ct.roll_rate_sp, ct.roll_rate_curr); }
  for (int i = 0; i < ret; i++)  { set_rc(1500, 1500, hover, 1500, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cr, ct.roll_rate_sp, ct.roll_rate_curr); }
  /* pitch doublet */
  for (int i = 0; i < hold; i++) { set_rc(1500, step, hover, 1500, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cp, ct.pitch_rate_sp, ct.pitch_rate_curr); }
  for (int i = 0; i < ret; i++)  { set_rc(1500, 1500, hover, 1500, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cp, ct.pitch_rate_sp, ct.pitch_rate_curr); }
  /* yaw doublet (yaw is rate-commanded) */
  for (int i = 0; i < hold; i++) { set_rc(1500, 1500, hover, step, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cy, ct.yaw_rate_sp, ct.yaw_rate_curr); }
  for (int i = 0; i < ret; i++)  { set_rc(1500, 1500, hover, 1500, 2000); step_once(&s, &ct, &got);
    if (got) corr_add(&cy, ct.yaw_rate_sp, ct.yaw_rate_curr); }

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
  double sims = (200 + settle + (hold + ret) * 3) / 1000.0;
  double x = wall > 0 ? sims / wall : 0.0;
  double rr = corr_val(&cr), pr = corr_val(&cp), yr = corr_val(&cy);
  fprintf(stderr,
          "vayu_sitl_rtos: doublet — rate-loop corr(sp,curr) roll %.2f pitch %.2f "
          "yaw %.2f  (state=0x%x, %.1fs sim in %.3fs = %.0fx)\n",
          rr, pr, yr, (unsigned)system_state_get(), sims, wall, x);
  printf("#RTOS-TUNE seed=%u roll_corr=%.4f pitch_corr=%.4f yaw_corr=%.4f "
         "rate_kp=%.6g angle_kp=%.4g wall=%.4f speedup=%.1f\n",
         seed, rr, pr, yr, env_f("VAYU_RATE_KP", 5e-4), env_f("VAYU_ANGLE_KP", 4.0),
         wall, x);
  fflush(stdout);
  return 0;
}

/* ---- hold rollout: determinism fingerprints (default) ---------------- */
static int run_hold(uint32_t seed, int N, double *out_wall, double *out_x) {
  vsim_inproc_reset(seed);
  stepper_t s;
  stepper_init(&s);
  double imu_fp = 0.0, fp = 0.0;
  attitude_t att = {0};
  control_telemetry_t ct;
  int got;
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  for (int n = 0; n < N; n++) {
    set_rc(1500, 1500, 1000, 1500, 1000);   /* neutral, disarmed */
    imu_fp += (double)s.sample.converted.acc[0]; /* pre-step value of last sample */
    step_once(&s, &ct, &got);
    imu_fp += (double)s.sample.converted.gyr[0] + s.sample.converted.mag[0];
    if (attitude_queue_telemetry_peek(&att))
      fp += (double)att.roll + att.pitch + att.yaw;
  }
  clock_gettime(CLOCK_MONOTONIC, &t1);
  double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
  double sims = N / 1000.0, x = wall > 0 ? sims / wall : 0.0;
  fprintf(stderr, "vayu_sitl_rtos: hold — %d samples in %.2fs = %.1fx, "
          "state=0x%x ctx=%u\n", N, wall, x, (unsigned)system_state_get(),
          get_context_switch_count());
  printf("#RTOS-RESULT seed=%u samples=%d imu_fp=%.17g att_fp=%.17g wall=%.6f "
         "speedup=%.2f\n", seed, N, imu_fp, fp, wall, x);
  fflush(stdout);
  if (out_wall) *out_wall = wall;
  if (out_x) *out_x = x;
  return 0;
}

int main(void) {
  fprintf(stderr, "vayu_sitl_rtos: booting the REAL vaios scheduler on host\n");
  vaios_init_config_t cfg = {0};
  v_system_init(&cfg);                /* heap + scheduler init */
  if (vayu_sitl_start(NULL) != 0) {
    fprintf(stderr, "vayu_sitl_rtos: vayu_sitl_start failed\n");
    return 1;
  }
  scheduler_start();                  /* run boot tasks to idle */

  const char *seed_env = getenv("VAYU_RTOS_SEED");
  const char *nenv = getenv("VAYU_RTOS_SAMPLES");
  const char *scen = getenv("VAYU_RTOS_SCENARIO");
  const uint32_t seed =
      seed_env && *seed_env ? (uint32_t)strtoul(seed_env, 0, 10) : STEP_SEED;
  const int N = nenv && *nenv ? atoi(nenv) : 5000;

  if (scen && strcmp(scen, "doublet") == 0)
    return run_doublet(seed);
  return run_hold(seed, N, 0, 0);
}
