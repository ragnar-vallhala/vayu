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
 * Call once before stepping. Returns 0 on success, 1 on vayu_sitl_start failure. */
int rtos_engine_boot(void);

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

/* Apply roll/pitch/yaw PID gains from the VAYU_*_KP/KI/KD envs (no-op if unset). */
void apply_gains_from_env(void);

/* In-process vsim physics controls (defined in vsim_inproc.cpp) used directly
 * by the scenarios to set up the rig before a rollout. */
void vsim_inproc_reset(uint32_t seed);
void vsim_inproc_set_tether(float tether_k);

#ifdef __cplusplus
}
#endif

#endif /* VAYU_HOST_RTOS_ENGINE_H */
