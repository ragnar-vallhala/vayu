/*
 * host_rtos_main.c -- entry point for vayu_sitl_rtos (Phase 4,
 * firmware/docs/plans/sitl-lockstep-sim.md). Runs the REAL vaios scheduler on the host
 * via the ucontext port, with vsim physics linked in-process, stepped from one
 * thread — so SITL is deterministic and ~60x realtime.
 *
 * Two scenarios (env VAYU_RTOS_SCENARIO):
 *   hold     (default) constant neutral RC; emits determinism fingerprints
 *                      (see sim/host/validate_rtos_determinism.py).
 *   doublet            arm + a roll/pitch/yaw step-doublet with env-set PID
 *                      gains; scores rate-loop tracking corr(rate_sp,rate_curr)
 *                      — a fast, deterministic autotune-style eval. Sim-time
 *                      paced (the stepper owns the clock; no wall-clock sleeps).
 *   disturb            arm + hover + kick one axis, then capture the rate
 *                      ring-down at full sim cadence (1 ms) to a CSV — the
 *                      limit-cycle / robustness study the FIFO path aliased.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h> /* before <stdlib.h>: glibc stdlib.h uses int32_t */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "control/control_buffer.h" /* control_telemetry_t */
#include "est/est.h"                /* attitude_t */
#include "host_rtos_engine.h" /* the reusable step engine (boot/set_rc/step_once/...) */
#include "sensor/imu_buffer.h" /* attitude_queue_telemetry_peek */
#include "sys/state.h"         /* system_state_get, SYSTEM_STATE_* */

extern uint32_t get_context_switch_count(void); /* kernel task.c */

#define STEP_SEED 12345u

/* ---- streaming Pearson correlation ----------------------------------- */
typedef struct {
  double n, sx, sy, sxx, syy, sxy;
} corr_t;
static void corr_add(corr_t *c, double x, double y) {
  c->n++;
  c->sx += x;
  c->sy += y;
  c->sxx += x * x;
  c->syy += y * y;
  c->sxy += x * y;
}
static double corr_val(const corr_t *c) {
  if (c->n < 2)
    return 0.0;
  double n = c->n;
  double cov = c->sxy - c->sx * c->sy / n;
  double vx = c->sxx - c->sx * c->sx / n, vy = c->syy - c->sy * c->sy / n;
  double d = sqrt(vx * vy);
  return d > 1e-12 ? cov / d : 0.0;
}

/* ---- per-axis Sample windows for the EXACT GCS cost --------------------
 * Each captured sample mirrors control_telemetry_t onto the fields of
 * autotune::Sample, so the Navigator can score the doublet with the SAME
 * axisCost()/yawRateCost() it runs on the realtime SITL telemetry — including
 * its |angle|>80 deg -> kBig divergence guard — instead of the rate-tracking
 * correlation. Written to VAYU_RTOS_TUNE_OUT as a small binary blob the GCS
 * reads back (one window per axis: roll, pitch, yaw). */
#define TUNE_WIN_MAX 2000 /* hold+ret steps/axis (1 ms/step), with headroom */
/* 13 floats/sample: f0..8 mirror autotune::Sample (angle loop for roll/pitch,
 * rate loop for yaw, + the per-axis output); f9..12 add the roll/pitch RATE
 * setpoint+measured so the GCS can also score inner-loop tracking (the optional
 * "angle + rate" cost). The per-axis output (roll_out/pitch_out) IS the rate
 * controller output, so f2/f5 double as the rate-loop output. */
#define TUNE_SAMPLE_FLOATS 13
typedef struct {
  float f[TUNE_SAMPLE_FLOATS];
} tune_sample_t;
static tune_sample_t g_tune_win[3][TUNE_WIN_MAX];
static int g_tune_n[3];

static void tune_win_add(int ax, const control_telemetry_t *ct) {
  if (ax < 0 || ax > 2 || g_tune_n[ax] >= TUNE_WIN_MAX)
    return;
  tune_sample_t *s = &g_tune_win[ax][g_tune_n[ax]++];
  /* f0..8 field order MUST match autotune::Sample (Cost.h). */
  s->f[0] = ct->roll_angle_sp;
  s->f[1] = ct->roll_angle_curr;
  s->f[2] = ct->roll_out;
  s->f[3] = ct->pitch_angle_sp;
  s->f[4] = ct->pitch_angle_curr;
  s->f[5] = ct->pitch_out;
  s->f[6] = ct->yaw_rate_sp;
  s->f[7] = ct->yaw_rate_curr;
  s->f[8] = ct->yaw_out;
  s->f[9] = ct->roll_rate_sp;
  s->f[10] = ct->roll_rate_curr;
  s->f[11] = ct->pitch_rate_sp;
  s->f[12] = ct->pitch_rate_curr;
}

/* [magic u32][n_axes=3 u32], then per axis [count u32][count * tune_sample_t].
 * magic 'TNT2' (v2 = 13-float record); the GCS rejects an older 9-float file. */
static void tune_win_write(const char *path) {
  FILE *f = fopen(path, "wb");
  if (!f)
    return;
  uint32_t magic = 0x32544e54u /* 'TNT2' */, na = 3u;
  fwrite(&magic, 4, 1, f);
  fwrite(&na, 4, 1, f);
  for (int ax = 0; ax < 3; ax++) {
    uint32_t cnt = (uint32_t)g_tune_n[ax];
    fwrite(&cnt, 4, 1, f);
    fwrite(g_tune_win[ax], sizeof(tune_sample_t), (size_t)g_tune_n[ax], f);
  }
  fclose(f);
}

/* Excitation stick µs for step `i` (0-based) within a `hold`-ms maneuver.
 * waveform 0 = step (constant deflection at step_us); 1 = linear chirp: a swept
 * sine f0→f1 Hz over the hold, peak amplitude (step_us-1500) about centre — the
 * instantaneous frequency ramps linearly so the phase is its integral. Mirrors
 * Rollout.cpp exciteAxis so the fast backend probes the same band as realtime. */
static int excite_us(int i, int hold, int step_us, int waveform, double f0,
                     double f1) {
  if (waveform != 1)
    return step_us;
  const double amp = (double)step_us - 1500.0;
  const double T = hold > 0 ? (double)hold / 1000.0 : 1.0; /* s */
  const double t = (double)i / 1000.0;                     /* s (1 ms/step) */
  const double phase = 2.0 * M_PI * (f0 * t + (f1 - f0) * t * t / (2.0 * T));
  return (int)(1500.0 + amp * sin(phase));
}

/* ---- doublet rollout: arm, hover, step each axis, score rate tracking - */
static int run_doublet(uint32_t seed) {
  vsim_inproc_reset(seed);
  /* Soft rig (matches the realtime autotune RolloutParams.tetherK, default 30):
   * a hard pin makes the angle-tracking cost reward a motionless craft. */
  vsim_inproc_set_tether((float)env_f("VAYU_RTOS_TETHER", 30.0));
  apply_gains_from_env();
  stepper_t s;
  stepper_init(&s);
  control_telemetry_t ct;
  int got;
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  /* Match the realtime autotune doublet (RolloutParams): hold the step long
   * enough that the craft reaches steady state — at 1 ms/step, hold=1000 ⇒ 1.0 s,
   * ret=700 ⇒ 0.7 s, settle=600 ⇒ 0.6 s. A short hold (the craft never arrives)
   * makes the tracking IAE reward a motionless craft. Env-overridable so the GCS
   * can pass the same excitation it uses on the realtime path. */
  const int settle = (int)env_f("VAYU_RTOS_SETTLE_MS", 600);
  const int hold = (int)env_f("VAYU_RTOS_HOLD_MS", 1000);
  const int ret = (int)env_f("VAYU_RTOS_RET_MS", 700);
  const int hover = (int)env_f("VAYU_RTOS_HOVER_US", 1500);
  const int step =
      (int)env_f("VAYU_RTOS_STEP_US", 1800); /* ~21 deg angle cmd */
  /* Waveform: 0 = step doublet, 1 = chirp (swept sine f0→f1 over the hold). */
  const int wf = (int)env_f("VAYU_RTOS_WAVEFORM", 0);
  const double cf0 = env_f("VAYU_RTOS_CHIRP_F0", 1.0);
  const double cf1 = env_f("VAYU_RTOS_CHIRP_F1", 12.0);

  /* Arm (throttle low + arm high), then settle at hover. */
  for (int i = 0; i < 200; i++)
    set_rc(1500, 1500, 1000, 1500, 2000), step_once(&s, &ct, &got);
  for (int i = 0; i < settle; i++)
    set_rc(1500, 1500, hover, 1500, 2000), step_once(&s, &ct, &got);

  corr_t cr = {0}, cp = {0}, cy = {0};
  /* Per axis: fire the waveform (step or chirp) over `hold`, then return to
   * centre + settle over `ret`; capture the angle-loop window for the GCS cost. */
  /* roll doublet (axis 0) */
  for (int i = 0; i < hold; i++) {
    set_rc(excite_us(i, hold, step, wf, cf0, cf1), 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cr, ct.roll_rate_sp, ct.roll_rate_curr);
      tune_win_add(0, &ct);
    }
  }
  for (int i = 0; i < ret; i++) {
    set_rc(1500, 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cr, ct.roll_rate_sp, ct.roll_rate_curr);
      tune_win_add(0, &ct);
    }
  }
  /* pitch doublet (axis 1) */
  for (int i = 0; i < hold; i++) {
    set_rc(1500, excite_us(i, hold, step, wf, cf0, cf1), hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cp, ct.pitch_rate_sp, ct.pitch_rate_curr);
      tune_win_add(1, &ct);
    }
  }
  for (int i = 0; i < ret; i++) {
    set_rc(1500, 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cp, ct.pitch_rate_sp, ct.pitch_rate_curr);
      tune_win_add(1, &ct);
    }
  }
  /* yaw doublet (axis 2, rate-commanded) */
  for (int i = 0; i < hold; i++) {
    set_rc(1500, 1500, hover, excite_us(i, hold, step, wf, cf0, cf1), 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cy, ct.yaw_rate_sp, ct.yaw_rate_curr);
      tune_win_add(2, &ct);
    }
  }
  for (int i = 0; i < ret; i++) {
    set_rc(1500, 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      corr_add(&cy, ct.yaw_rate_sp, ct.yaw_rate_curr);
      tune_win_add(2, &ct);
    }
  }

  /* Emit the per-axis Sample windows so the GCS can run the exact realtime cost. */
  {
    const char *tout = getenv("VAYU_RTOS_TUNE_OUT");
    if (tout && *tout)
      tune_win_write(tout);
  }

  clock_gettime(CLOCK_MONOTONIC, &t1);
  double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
  double sims = (200 + settle + (hold + ret) * 3) / 1000.0;
  double x = wall > 0 ? sims / wall : 0.0;
  double rr = corr_val(&cr), pr = corr_val(&cp), yr = corr_val(&cy);
  fprintf(
      stderr,
      "vayu_sitl_rtos: doublet — rate-loop corr(sp,curr) roll %.2f pitch %.2f "
      "yaw %.2f  (state=0x%x, %.1fs sim in %.3fs = %.0fx)\n",
      rr, pr, yr, (unsigned)system_state_get(), sims, wall, x);
  printf("#RTOS-TUNE seed=%u roll_corr=%.4f pitch_corr=%.4f yaw_corr=%.4f "
         "rate_kp=%.6g angle_kp=%.4g wall=%.4f speedup=%.1f\n",
         seed, rr, pr, yr, env_f("VAYU_RATE_KP", 5e-4),
         env_f("VAYU_ANGLE_KP", 4.0), wall, x);
  fflush(stdout);
  return 0;
}

/* ---- disturbance rollout: arm, hover, kick one axis, capture the rate
 * ring-down at FULL sim cadence (1 ms/step) -----------------------------
 * The limit-cycle / robustness study the FIFO path could never do cleanly:
 * wall-clock polling aliased the fast lockstep sim, so a sustained ~2 Hz cycle
 * was indistinguishable from a decaying one. In-process there is no FIFO and
 * the stepper owns the clock, so EVERY control sample is captured. Commands an
 * angle step on VAYU_RTOS_DIST_AXIS (0 roll, 1 pitch, 2 yaw) of VAYU_RTOS_DIST_US
 * about centre for VAYU_RTOS_DIST_MS, returns to centre, and traces
 * VAYU_RTOS_CAPTURE_MS of ring-down to VAYU_RTOS_TRACE_CSV. Reports a decay ratio
 * (late-RMS/peak): >~0.5 ⇒ the rate never settled = a sustained limit cycle. To
 * reproduce the REAL plant, run with VAYU_RTOS_GEOMETRY + the VSIM_MOTOR_DELAY_MS
 * / VSIM_STALL_DUTY actuator-imperfection envs (carried by load_geometry). */
static int run_disturb(uint32_t seed) {
  vsim_inproc_reset(seed);
  vsim_inproc_set_tether((float)env_f("VAYU_RTOS_TETHER", 30.0));
  apply_gains_from_env();
  stepper_t s;
  stepper_init(&s);
  control_telemetry_t ct;
  int got;

  const int settle = (int)env_f("VAYU_RTOS_SETTLE_MS", 600);
  const int hover = (int)env_f("VAYU_RTOS_HOVER_US", 1500);
  const int dist_ms = (int)env_f("VAYU_RTOS_DIST_MS", 60);
  const int dist_us = (int)env_f("VAYU_RTOS_DIST_US", 1800); /* ~21 deg cmd */
  const int cap_ms = (int)env_f("VAYU_RTOS_CAPTURE_MS", 3000);
  const int axis = (int)env_f("VAYU_RTOS_DIST_AXIS", 1); /* pitch */

  FILE *csv = NULL;
  const char *cp = getenv("VAYU_RTOS_TRACE_CSV");
  if (cp && *cp) {
    csv = fopen(cp, "w");
    if (csv)
      fprintf(csv, "t_ms,phase,roll_rate_sp,roll_rate_curr,pitch_rate_sp,"
                   "pitch_rate_curr,yaw_rate_sp,yaw_rate_curr,roll_ang,"
                   "pitch_ang,d0,d1,d2,d3\n");
  }
#define DIST_TRACE(ph)                                                         \
  if (got && csv)                                                              \
  fprintf(csv, "%d,%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n", t, ph,           \
          ct.roll_rate_sp, ct.roll_rate_curr, ct.pitch_rate_sp,                \
          ct.pitch_rate_curr, ct.yaw_rate_sp, ct.yaw_rate_curr,                \
          ct.roll_angle_curr, ct.pitch_angle_curr, s.duty[0], s.duty[1],       \
          s.duty[2], s.duty[3])
#define DIST_RATE(c)                                                           \
  (axis == 0   ? (c).roll_rate_curr                                            \
   : axis == 1 ? (c).pitch_rate_curr                                           \
               : (c).yaw_rate_curr)

  /* arm (throttle low + arm high), then settle at hover */
  for (int i = 0; i < 200; i++) {
    set_rc(1500, 1500, 1000, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  for (int i = 0; i < settle; i++) {
    set_rc(1500, 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
  }

  /* Envelope decay: compare the oscillation amplitude just after the kick
   * transient (early window 10–30% of the capture) to the tail (late window,
   * final 40%). late/early ≈ 1 (or rising) ⇒ a sustained limit cycle; ≪1 ⇒ it
   * rang down. Using an early RMS — not the single peak spike — as the baseline
   * makes the ratio a true decay measure. (FFT the CSV for the cycle frequency.) */
  double peak = 0.0, late_sxx = 0.0, early_sxx = 0.0;
  long late_n = 0, early_n = 0;
  const int early_lo = (int)(cap_ms * 0.1), early_hi = (int)(cap_ms * 0.3);
  const int late_start = (int)(cap_ms * 0.6); /* RMS over the final 40% */
  int t = 0;

  /* kick: hold the angle step on the chosen axis */
  for (int i = 0; i < dist_ms; i++, t++) {
    int r = 1500, p = 1500, y = 1500;
    if (axis == 0)
      r = dist_us;
    else if (axis == 1)
      p = dist_us;
    else
      y = dist_us;
    set_rc(r, p, hover, y, 2000);
    step_once(&s, &ct, &got);
    DIST_TRACE("kick");
  }
  /* release: back to centre, capture the ring-down */
  for (int i = 0; i < cap_ms; i++, t++) {
    set_rc(1500, 1500, hover, 1500, 2000);
    step_once(&s, &ct, &got);
    if (got) {
      double rr = DIST_RATE(ct);
      if (fabs(rr) > peak)
        peak = fabs(rr);
      if (i >= early_lo && i < early_hi) {
        early_sxx += rr * rr;
        early_n++;
      }
      if (i >= late_start) {
        late_sxx += rr * rr;
        late_n++;
      }
    }
    DIST_TRACE("ring");
  }
  if (csv)
    fclose(csv);

  /* disarm: exercise the armed→standby transition before we exit */
  for (int i = 0; i < 50; i++) {
    set_rc(1500, 1500, 1000, 1500, 1000);
    step_once(&s, &ct, &got);
  }

  double late_rms = late_n ? sqrt(late_sxx / (double)late_n) : 0.0;
  double early_rms = early_n ? sqrt(early_sxx / (double)early_n) : 0.0;
  double decay = early_rms > 1e-6 ? late_rms / early_rms : 0.0;
  const char *axn = axis == 0 ? "roll" : axis == 1 ? "pitch" : "yaw";
  fprintf(
      stderr,
      "vayu_sitl_rtos: disturb axis=%s peak=%.1f early_rms=%.1f late_rms=%.1f "
      "deg/s decay=%.3f -> %s (end state=0x%x)\n",
      axn, peak, early_rms, late_rms, decay,
      decay > 0.7 ? "SUSTAINED (limit cycle)" : "decays",
      (unsigned)system_state_get());
  printf("#RTOS-DISTURB seed=%u axis=%s peak=%.4f early_rms=%.4f late_rms=%.4f "
         "decay=%.4f rate_kp=%.6g angle_kp=%.4g\n",
         seed, axn, peak, early_rms, late_rms, decay,
         env_f("VAYU_RATE_KP", 5e-4), env_f("VAYU_ANGLE_KP", 4.0));
  fflush(stdout);
#undef DIST_TRACE
#undef DIST_RATE
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
  /* #11b smoke test: VAYU_RTOS_REALTIME paces the loop to wall-clock (1 ms/step)
   * so a run takes ~N ms instead of ~N/40 ms. Pacing only sleeps — the
   * fingerprints below must stay bit-identical to the free-run values. */
  const int realtime = (int)env_f("VAYU_RTOS_REALTIME", 0);
  rtos_pacer_t pacer;
  if (realtime)
    rtos_pacer_init(&pacer);
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  for (int n = 0; n < N; n++) {
    set_rc(1500, 1500, 1000, 1500, 1000); /* neutral, disarmed */
    imu_fp +=
        (double)s.sample.converted.acc[0]; /* pre-step value of last sample */
    step_once(&s, &ct, &got);
    imu_fp += (double)s.sample.converted.gyr[0] + s.sample.converted.mag[0];
    if (attitude_queue_telemetry_peek(&att))
      fp += (double)att.roll + att.pitch + att.yaw;
    if (realtime)
      rtos_pacer_wait(&pacer, 0.001);
  }
  clock_gettime(CLOCK_MONOTONIC, &t1);
  double wall = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
  double sims = N / 1000.0, x = wall > 0 ? sims / wall : 0.0;
  fprintf(stderr,
          "vayu_sitl_rtos: hold — %d samples in %.2fs = %.1fx, "
          "state=0x%x ctx=%u\n",
          N, wall, x, (unsigned)system_state_get(), get_context_switch_count());
  printf("#RTOS-RESULT seed=%u samples=%d imu_fp=%.17g att_fp=%.17g wall=%.6f "
         "speedup=%.2f\n",
         seed, N, imu_fp, fp, wall, x);
  fflush(stdout);
  if (out_wall)
    *out_wall = wall;
  if (out_x)
    *out_x = x;
  return 0;
}

/* ---- pose smoke test (#11c): arm, kick pitch, dump the pose snapshot ---- */
static int run_pose(uint32_t seed) {
  vsim_inproc_reset(seed);
  vsim_inproc_set_tether((float)env_f("VAYU_RTOS_TETHER", 30.0));
  stepper_t s;
  stepper_init(&s);
  control_telemetry_t ct;
  int got;
  vsim_pose_frame_t p;
  for (int i = 0; i < 200; i++) {
    set_rc(1500, 1500, 1000, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  for (int i = 0; i < 400; i++) {
    set_rc(1500, 1500, 1500, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  vsim_inproc_get_pose(&p);
  printf("#RTOS-POSE pre-kick  tick=%u quat=[%.4f %.4f %.4f %.4f] "
         "omega_b=[%.2f %.2f %.2f] pos=[%.3f %.3f %.3f]\n",
         p.hdr.seq_no, p.quat_wxyz[0], p.quat_wxyz[1], p.quat_wxyz[2],
         p.quat_wxyz[3], p.omega_b[0], p.omega_b[1], p.omega_b[2], p.pos_w[0],
         p.pos_w[1], p.pos_w[2]);
  for (int i = 0; i < 120; i++) {
    set_rc(1500, 1850, 1500, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  vsim_inproc_get_pose(&p);
  printf("#RTOS-POSE post-kick tick=%u quat=[%.4f %.4f %.4f %.4f] "
         "omega_b=[%.2f %.2f %.2f] pos=[%.3f %.3f %.3f]\n",
         p.hdr.seq_no, p.quat_wxyz[0], p.quat_wxyz[1], p.quat_wxyz[2],
         p.quat_wxyz[3], p.omega_b[0], p.omega_b[1], p.omega_b[2], p.pos_w[0],
         p.pos_w[1], p.pos_w[2]);
  fflush(stdout);
  fprintf(stderr,
          "vayu_sitl_rtos: pose — magic=0x%x ver=%u type=%u payload=%u (expect "
          "0x%x/%u/%u/%zu)\n",
          p.hdr.magic, p.hdr.version, p.hdr.type, p.hdr.payload_bytes,
          VSIM_MAGIC, VSIM_PROTO_VERSION, (unsigned)VSIM_FRAME_POSE,
          sizeof(vsim_pose_frame_t) - sizeof(vsim_hdr_t));
  return 0;
}

/* ---- config-surface smoke test (#11d): arm, spin up, kill motors ------- */
static int run_cfg(uint32_t seed) {
  vsim_inproc_reset(seed);
  vsim_inproc_set_tether((float)env_f("VAYU_RTOS_TETHER", 30.0));
  stepper_t s;
  stepper_init(&s);
  control_telemetry_t ct;
  int got;
  vsim_pose_frame_t p;
  for (int i = 0; i < 200; i++) {
    set_rc(1500, 1500, 1000, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  for (int i = 0; i < 400; i++) {
    set_rc(1500, 1500, 1500, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  vsim_inproc_get_pose(&p);
  float before = 0;
  for (int i = 0; i < 4; i++)
    before += fabsf(p.motor_omega[i]);
  before /= 4;
  /* kill all four ESCs via the config surface */
  vsim_ctl_faults_t f;
  memset(&f, 0, sizeof f);
  for (int i = 0; i < 4; i++)
    f.motor_kill[i] = 1;
  vsim_inproc_set_faults(&f);
  for (int i = 0; i < 300; i++) {
    set_rc(1500, 1500, 1500, 1500, 2000);
    step_once(&s, &ct, &got);
  }
  vsim_inproc_get_pose(&p);
  float after = 0;
  for (int i = 0; i < 4; i++)
    after += fabsf(p.motor_omega[i]);
  after /= 4;
  int ok = after < 0.05f * before;
  printf("#RTOS-CFG motor_kill mean_omega before=%.1f after=%.1f -> %s\n",
         before, after, ok ? "KILLED ok" : "FAIL");
  fflush(stdout);
  return ok ? 0 : 1;
}

/* ---- interactive headless driver (VAYU_RTOS_SCENARIO=driver) -------------
 * Turns the standalone binary into a drop-in replacement for the old
 * vsim_d + vayu_sitl FIFO pair: an external (Python) driver streams RC over the
 * VAYU_UART_RC_PATH pty and config over the ctl FIFO, and reads FC telemetry off
 * the UART2 pty advert + ground-truth pose off the pose FIFO. One process now
 * runs BOTH firmware and physics; the pwm/imu/baro FIFOs (the old firmware<->
 * physics boundary) are gone — that hop is in-process.
 *
 * Wire contract unchanged: same /tmp/vsim_{pose,ctl}<suffix> FIFOs, same
 * vsim_pose_frame_t / vsim_ctl_frame_t framing (sim/vsim/include/vsim_proto.h),
 * same UART2 advert (/tmp/vayu_uart2_pty<suffix>) and RC CSV pty as before. */

static volatile sig_atomic_t g_driver_stop = 0;
static void driver_on_signal(int sig) {
  (void)sig;
  g_driver_stop = 1;
}

/* Append $VSIM_FIFO_SUFFIX to a /tmp base path (mirrors host_navhal + the
 * suffix the external driver publishes). Unset/empty == legacy shared path. */
static void driver_suffixed(char *out, size_t n, const char *base) {
  const char *s = getenv("VSIM_FIFO_SUFFIX");
  snprintf(out, n, "%s%s", base, (s && *s) ? s : "");
}

/* mkfifo (tolerating EEXIST) then open. O_RDWR keeps the FIFO open across a
 * peer's connect/disconnect and makes writes non-blocking-lossy rather than
 * ENXIO when no reader is attached yet. */
static int driver_open_fifo(const char *path, int flags) {
  if (mkfifo(path, 0666) != 0 && errno != EEXIST)
    return -1;
  return open(path, flags);
}

/* One ctl frame -> the matching in-process config call (faithful port of
 * vsim_d's dispatch; the vsim_inproc_* surface reuses the same wire structs). */
static void driver_dispatch_ctl(const vsim_ctl_frame_t *f) {
  switch (f->subtype) {
  case VSIM_CTL_RESET: {
    vsim_ctl_reset_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_reset_to(&b);
    break;
  }
  case VSIM_CTL_PAUSE: {
    int32_t p;
    memcpy(&p, f->body, sizeof p);
    vsim_inproc_set_pause(p != 0);
    break;
  }
  case VSIM_CTL_SET_TESTRIG: {
    vsim_ctl_testrig_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_testrig(&b);
    break;
  }
  case VSIM_CTL_SET_GEOMETRY: {
    vsim_ctl_geometry_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_geometry(&b);
    break;
  }
  case VSIM_CTL_SET_WORLD: {
    vsim_ctl_world_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_world(&b);
    break;
  }
  case VSIM_CTL_CLEAR_OBSTACLES:
    vsim_inproc_clear_obstacles();
    break;
  case VSIM_CTL_ADD_OBSTACLE: {
    vsim_ctl_obstacle_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_add_obstacle(&b);
    break;
  }
  case VSIM_CTL_SET_WORLD_MESH: {
    vsim_ctl_world_mesh_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_world_mesh(&b);
    break;
  }
  case VSIM_CTL_CLEAR_WORLD_MESH:
    vsim_inproc_clear_world_mesh();
    break;
  case VSIM_CTL_SET_RATES: {
    vsim_ctl_rates_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_rates(&b);
    break;
  }
  case VSIM_CTL_SET_NOISE: {
    vsim_ctl_noise_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_noise(&b);
    break;
  }
  case VSIM_CTL_SET_FAULTS: {
    vsim_ctl_faults_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_faults(&b);
    break;
  }
  case VSIM_CTL_SET_WIND: {
    vsim_ctl_wind_t b;
    memcpy(&b, f->body, sizeof b);
    vsim_inproc_set_wind(&b);
    break;
  }
  case VSIM_CTL_PING:
  default:
    break;
  }
}

static int run_driver(void) {
  signal(SIGINT, driver_on_signal);
  signal(SIGTERM, driver_on_signal);
  signal(SIGPIPE,
         SIG_IGN); /* a closed pose reader must not kill us mid-write */

  char pose_path[256], ctl_path[256];
  driver_suffixed(pose_path, sizeof pose_path, VSIM_FIFO_POSE);
  driver_suffixed(ctl_path, sizeof ctl_path, VSIM_FIFO_CTL);
  int pose_fd = driver_open_fifo(pose_path, O_RDWR | O_NONBLOCK);
  int ctl_fd = driver_open_fifo(ctl_path, O_RDWR | O_NONBLOCK);
  if (pose_fd < 0 || ctl_fd < 0) {
    fprintf(stderr, "rtos driver: FIFO setup failed (pose=%s ctl=%s)\n",
            pose_path, ctl_path);
    if (pose_fd >= 0)
      close(pose_fd);
    if (ctl_fd >= 0)
      close(ctl_fd);
    return 1;
  }

  /* RC streams in over VAYU_UART_RC_PATH (the external driver's pty); telemetry
   * already flows out the UART2 pty advert (rtos_engine_boot(NULL) wired it). */
  rtos_engine_enable_serial_rc();
  fprintf(stderr,
          "rtos driver: online — pose=%s ctl=%s, telemetry on the UART2 pty "
          "advert, "
          "RC on VAYU_UART_RC_PATH\n",
          pose_path, ctl_path);

  /* run_step advances 1 ms (wall-clock paced); publish pose every ~16 ms → 60 Hz. */
  const int pose_div = 16;
  int pose_ctr = 0;
  uint8_t buf[4096];
  size_t buflen = 0;

  rtos_engine_run_begin();
  while (!g_driver_stop) {
    /* 1) Drain queued ctl frames (a startup burst — rates/geometry/world/mesh/
     * wind — must ALL apply, so parse every complete frame, not just the last). */
    ssize_t r = read(ctl_fd, buf + buflen, sizeof buf - buflen);
    if (r > 0) {
      buflen += (size_t)r;
      size_t pos = 0;
      while (pos + sizeof(vsim_hdr_t) <= buflen) {
        const vsim_hdr_t *h = (const vsim_hdr_t *)(buf + pos);
        if (h->magic != VSIM_MAGIC) {
          pos++;
          continue;
        } /* resync after a torn write */
        const size_t need = sizeof(vsim_hdr_t) + h->payload_bytes;
        if (pos + need > buflen)
          break; /* incomplete tail — wait for more */
        if (h->type == VSIM_FRAME_CTL && need == sizeof(vsim_ctl_frame_t))
          driver_dispatch_ctl((const vsim_ctl_frame_t *)(buf + pos));
        pos += need;
      }
      if (pos > 0) {
        memmove(buf, buf + pos, buflen - pos);
        buflen -= pos;
      }
      if (buflen == sizeof buf)
        buflen = 0; /* no frame boundary in a full buffer → resync */
    }

    /* 2) Advance one paced step (firmware + physics in lockstep, in-process). */
    rtos_engine_run_step();

    /* 3) Publish the latest ground-truth pose at ~60 Hz. Lossy write: no reader
     * yet or a full pipe just drops the frame, exactly like the old link. */
    if (++pose_ctr >= pose_div) {
      pose_ctr = 0;
      vsim_pose_frame_t p;
      vsim_inproc_get_pose(&p);
      ssize_t w = write(pose_fd, &p, sizeof p);
      (void)w;
    }
  }

  close(pose_fd);
  close(ctl_fd);
  fprintf(stderr, "rtos driver: stopping\n");
  return 0;
}

int main(void) {
  if (rtos_engine_boot(NULL) !=
      0) /* headless: telemetry via the pty, RC via set_rc */
    return 1;

  const char *seed_env = getenv("VAYU_RTOS_SEED");
  const char *nenv = getenv("VAYU_RTOS_SAMPLES");
  const char *scen = getenv("VAYU_RTOS_SCENARIO");
  const uint32_t seed =
      seed_env && *seed_env ? (uint32_t)strtoul(seed_env, 0, 10) : STEP_SEED;
  const int N = nenv && *nenv ? atoi(nenv) : 5000;

  if (scen && strcmp(scen, "driver") == 0)
    return run_driver();
  if (scen && strcmp(scen, "doublet") == 0)
    return run_doublet(seed);
  if (scen && strcmp(scen, "disturb") == 0)
    return run_disturb(seed);
  if (scen && strcmp(scen, "pose") == 0)
    return run_pose(seed);
  if (scen && strcmp(scen, "cfg") == 0)
    return run_cfg(seed);
  return run_hold(seed, N, 0, 0);
}
