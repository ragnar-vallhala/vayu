/*
 * host_rtos_engine.h -- the reusable in-process RTOS SITL step engine.
 *
 * Factored out of host_rtos_main.c (Phase 5, docs/plans/sitl-rtos-consolidation.md)
 * so BOTH the standalone scenarios (host_rtos_main.c) and the GCS worker thread
 * drive the SAME loop: advance in-process vsim physics under the last PWM,
 * sample the IMU, inject it, tick the real vaios scheduler to idle, read PWM
 * back inline. Single-threaded, deterministic, no FIFO.
 *
 * #11a is a pure refactor: no behaviour change (determinism fingerprints stay
 * bit-identical). Realtime pacing (#11b), the pose getter (#11c) and the config
 * API (#11d) build on top of these primitives.
 */
#ifndef VAYU_HOST_RTOS_ENGINE_H
#define VAYU_HOST_RTOS_ENGINE_H

#include <stdint.h>

#include "control/control_buffer.h" /* control_telemetry_t */
#include "sensor/sensor.h"          /* bmx160_all_reading_t, imu_queue_*_push */
#include "vsim_proto.h"             /* vsim_pose_frame_t (the GCS wire layout) */

#ifdef __cplusplus
extern "C" {
#endif

/* Per-sample stepper state: the duty held across steps, the injected IMU
 * sample, and the running cycle counter used for the sample timestamp. */
typedef struct {
  float duty[4];
  bmx160_all_reading_t sample;
  uint32_t cyc;
} stepper_t;

/* Boot the REAL vaios scheduler + in-process vsim physics, then apply the
 * VAYU_RTOS_GEOMETRY airframe (and its actuator-imperfection envs), or — with
 * no geometry file — the actuator-imperfection envs against the reference quad.
 * `iface` is the vsim_iface_t* whose UART2 callback receives firmware telemetry
 * in-process (the GCS passes its iface so telemetry reaches the GUI exactly as
 * the legacy in-process firmware did); NULL for the headless scenarios (telemetry
 * falls back to the pty). Call once before stepping. Returns 0 on success, 1 on
 * vayu_sitl_start failure. */
int rtos_engine_boot(void *iface);

/* Start the serial RC feeder thread (host_rc_feeder): it reads RC µs frames from
 * VAYU_UART_RC_PATH (a pty/serial — RcBridge in the GCS, or a remote transmitter)
 * and pushes them + the arm/disarm state machine into the firmware. For the
 * INTERACTIVE GCS only — call after boot. The headless scenarios DON'T call this;
 * they inject RC deterministically via set_rc, so determinism is preserved. */
void rtos_engine_enable_serial_rc(void);

/* Type-free interactive run facade: the GCS worker drives the engine
 * without seeing the firmware types. run_begin resets the internal stepper +
 * pacer (call on (re)start); run_step advances one 1 ms step and paces to
 * wall-clock. RC arrives via the serial feeder; pose via vsim_inproc_get_pose. */
void rtos_engine_run_begin(void);
void rtos_engine_run_step(void);

/* Zero a stepper. The IMU struct MUST start zeroed: garbage in the fields
 * packImu doesn't write breaks determinism (found the hard way). */
void stepper_init(stepper_t *s);

/* Push one RC frame + run the arm/disarm state machine the (absent) rc_task
 * would (same logic as host_rc_feeder). roll/pitch/thr/yaw/arm are raw stick µs. */
void set_rc(int roll, int pitch, int thr, int yaw, int arm);

/* One deterministic sim step: physics under last PWM -> sample IMU -> inject ->
 * SysTick+1 -> run the scheduler to idle (PWM written) -> read PWM back. Drains
 * the control trace to its latest into *ct (got=1 if any sample was produced). */
void step_once(stepper_t *s, control_telemetry_t *ct, int *got);

/* getenv-as-double with a default (shared by the CLI scenarios). */
double env_f(const char *k, double dflt);

/* ---- wall-clock pacer (#11b) -----------------------------------------
 * FREE_RUN (the default — headless/CI/autotune) just calls step_once in a tight
 * loop; the stepper owns the sim clock, so it runs ~40-60x realtime and stays
 * deterministic. REALTIME paces that loop to wall time for the interactive GCS:
 * the caller drives one step_once per pacer_wait(dt). Pacing only SLEEPS — it
 * never touches the sim math, so determinism (the fingerprints) is unchanged;
 * only how fast wall-time advances differs. clock_nanosleep(TIMER_ABSTIME) on a
 * monotonic target, with catch-up-skip when behind (no busy spiral). */
#include <time.h>
typedef struct {
  struct timespec next;  /* absolute monotonic target for the next step */
  long behind;           /* count of steps the loop fell behind (diagnostic) */
} rtos_pacer_t;

/* Anchor the pacer at "now". Call once before the paced loop. */
void rtos_pacer_init(rtos_pacer_t *p);

/* Advance the target by dt_s and sleep until it. If already behind, reset the
 * baseline to now (skip, don't spiral) and bump p->behind. */
void rtos_pacer_wait(rtos_pacer_t *p, double dt_s);

/* Apply roll/pitch/yaw PID gains from the VAYU_*_KP/KI/KD envs (no-op if unset). */
void apply_gains_from_env(void);

/* In-process vsim physics controls (defined in vsim_inproc.cpp) used directly
 * by the scenarios to set up the rig before a rollout. */
void vsim_inproc_reset(uint32_t seed);
void vsim_inproc_set_tether(float tether_k);

/* Latest pose snapshot for the GCS renderer (#11c) — lock-free seqlock read,
 * safe from a thread other than the stepper. Same vsim_pose_frame_t the
 * decoupled vsim_d publishes, so SimWorker's consumer is unchanged. */
void vsim_inproc_get_pose(vsim_pose_frame_t *out);

/* ---- in-process config surface (#11d) --------------------------------
 * One call per VSIM_CTL_* message (faithful port of vsim_d's dispatch), reusing
 * the vsim_proto.h wire structs — the GCS configures the in-process physics by
 * direct call instead of writing the ctl FIFO. */
void vsim_inproc_reset_to(const vsim_ctl_reset_t *b);
void vsim_inproc_set_testrig(const vsim_ctl_testrig_t *t);
void vsim_inproc_set_geometry(const vsim_ctl_geometry_t *g);
void vsim_inproc_set_world(const vsim_ctl_world_t *w);
void vsim_inproc_clear_obstacles(void);
void vsim_inproc_add_obstacle(const vsim_ctl_obstacle_t *b);
int  vsim_inproc_set_world_mesh(const vsim_ctl_world_mesh_t *m);  /* 1 ok, 0 fail */
void vsim_inproc_clear_world_mesh(void);
void vsim_inproc_set_rates(const vsim_ctl_rates_t *r);
void vsim_inproc_set_noise(const vsim_ctl_noise_t *n);
void vsim_inproc_set_faults(const vsim_ctl_faults_t *f);
void vsim_inproc_set_wind(const vsim_ctl_wind_t *w);
void vsim_inproc_set_pause(int paused);

#ifdef __cplusplus
}
#endif

#endif /* VAYU_HOST_RTOS_ENGINE_H */
