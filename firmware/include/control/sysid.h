#pragma once
#include <stdint.h>

/* On-hardware system identification.
 *
 * Injects a tapered linear chirp so a host can fit the rate plant from the
 * measured (control effort u, gyro) response. Two injection points are offered:
 *
 *   SYSID_INJECT_RATE_SP (0, default): perturb the rate-loop SETPOINT (deg/s).
 *     The chirp is ADDED to target_rates[] *before* the rate PID, so the CLOSED
 *     loop tracks it. Safe and self-stabilising, but the captured u is the PID's
 *     response (shaped by the loop) — closed-loop ID, where u is correlated with
 *     gyro noise. Good enough for the loop-shaped tune; biased near the loop BW.
 *
 *   SYSID_INJECT_U (1): perturb the rate-PID OUTPUT u (control effort) directly.
 *     The chirp is ADDED to outputs[] *after* the rate PID, i.e. straight onto
 *     the plant input, and the captured u is that total command. Above the rate
 *     loop's bandwidth the PID barely reacts, so u ~= the chirp — a far cleaner,
 *     near-open-loop plant excitation. More aggressive: it bypasses the loop's
 *     smoothing, so amp is in CONTROL-EFFORT units (~O(1) full scale), not deg/s,
 *     and is clamped to a tighter hard cap. Prefer this when the airframe can
 *     hold attitude through the perturbation (rig or stable hover).
 *
 * SAFETY MODEL (both modes):
 *  - Motors move solely via the controller's existing ARMED-gated
 *    motor_set_outputs() downstream of the capture, so the whole path (command,
 *    state machine, chirp, self-abort, capture) is exercisable fully DISARMED
 *    with props off — the chirp appears in the capture while the motors stay
 *    still (a real plant fit still needs an ARMED run so the gyro responds).
 *  - Self-aborts on |angle| or |rate| limit on the excited axis, and on timeout.
 *  - Commanded amplitude / frequency / duration are clamped to hard caps; the
 *    amplitude cap is mode-dependent (deg/s for RATE_SP, control-effort for U).
 */

enum {
  SYSID_INJECT_RATE_SP = 0, /* perturb the rate SETPOINT (deg/s); closed-loop */
  SYSID_INJECT_U = 1,       /* perturb the rate-PID OUTPUT u (control effort) */
};

typedef struct {
  uint8_t axis;  // 0 roll, 1 pitch, 2 yaw
  uint8_t mode;  // SYSID_INJECT_RATE_SP (0) or SYSID_INJECT_U (1)
  float f0_hz;   // chirp start frequency
  float f1_hz;   // chirp end frequency
  float amp_dps; // amplitude: deg/s for RATE_SP, control-effort u for U
  float duration_s;
} sysid_request_t;

// Start an excitation (replaces any in progress). Params are validated/clamped.
void sysid_start(const sysid_request_t *req);
// Abort immediately; the injection returns to zero on the next tick.
void sysid_abort(void);
// Non-zero while an excitation is active.
int sysid_active(void);

/* Per control tick. dt seconds; current angles (deg) and body rates (deg/s) are
 * used for the self-abort check. Writes the per-axis chirp perturbation to
 * inject_out[3] (deg/s in RATE_SP mode, control-effort u in U mode) — all zero
 * when idle or aborted. The caller routes inject_out to target_rates[] (RATE_SP)
 * or to outputs[] (U) per sysid_inject_mode(). Advances chirp phase, auto-stops
 * at duration, and aborts if a limit is exceeded. */
void sysid_step(float dt, const float angles_deg[3], const float rates_dps[3],
                float inject_out[3]);

/* Injection point of the active/last run: SYSID_INJECT_RATE_SP or SYSID_INJECT_U.
 * The controller uses this to decide whether to add the chirp to the rate
 * setpoint (before the PID) or to the control effort u (after the PID). */
int sysid_inject_mode(void);

/* High-rate capture. Call once per control tick AFTER the rate PID:
 * while a run is active it records the excited axis's (u, gyro) — u is the
 * rate-PID OUTPUT (the control effort the plant fit needs), gyro the measured
 * body rate — decimated to ~500 Hz into a bounded RAM buffer. No-op when idle.
 * Reset at each sysid_start(). */
void sysid_capture(const float u[3], const float gyro[3]);

/* No-op: capture is RAM-only, so there is nothing to flush. Present so the
 * telemetry-task call site is stable. */
void sysid_flush_poll(void);

int sysid_capture_count(void); // total samples captured to RAM for the last run
int sysid_capture_hz(void);    // capture rate (Hz)
int sysid_capture_axis(void);  // excited axis of the last run

/* Post-run dump: stream the captured RAM buffer back over the link. Driven by
 * CMD_SYSID_DUMP; the telemetry task pulls chunks via sysid_dump_next(). */
void sysid_dump_request(void); // (re)start the dump from sample 0
int sysid_dump_active(void);   // non-zero while a dump is in progress
/* Read the next chunk of up to `cap` samples from the RAM capture buffer into
 * u/gyro (u = rate-PID output x1000, gyro = 0.1 deg/s; both i16); returns the
 * count read (0 when complete / on error) and sets *start to this chunk's first
 * sample index. Called from the telemetry task. */
int sysid_dump_next(uint16_t *start, int16_t *u, int16_t *gyro, int cap);
