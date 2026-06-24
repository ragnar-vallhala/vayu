#pragma once
#include <stdint.h>

/* On-hardware system identification — Phase 0 skeleton.
 *
 * Injects a tapered linear chirp into ONE rate-loop setpoint (deg/s) so a host
 * can fit the rate plant from the measured gyro response. The excitation is
 * produced here and ADDED to target_rates[] by angle_rate_controller; the
 * existing CONTROL_TRACE telemetry (rate_sp vs rate_curr) is the capture channel.
 *
 * SAFETY MODEL:
 *  - Only the SETPOINT of the closed rate loop is perturbed; the loop keeps the
 *    craft bounded. Motors move solely via the controller's existing
 *    ARMED-gated motor_set_outputs(), so the entire path (command, state machine,
 *    chirp, self-abort, telemetry capture) is exercisable fully DISARMED with
 *    props off — the chirp appears in telemetry while the motors stay still.
 *  - Self-aborts on |angle| or |rate| limit on the excited axis, and on timeout.
 *  - Commanded amplitude / frequency / duration are clamped to hard caps.
 */

typedef struct {
  uint8_t axis;  // 0 roll, 1 pitch, 2 yaw
  float f0_hz;   // chirp start frequency
  float f1_hz;   // chirp end frequency
  float amp_dps; // amplitude (deg/s) of the rate-setpoint perturbation
  float duration_s;
} sysid_request_t;

// Start an excitation (replaces any in progress). Params are validated/clamped.
void sysid_start(const sysid_request_t *req);
// Abort immediately; the injection returns to zero on the next tick.
void sysid_abort(void);
// Non-zero while an excitation is active.
int sysid_active(void);

/* Per control tick. dt seconds; current angles (deg) and body rates (deg/s) are
 * used for the self-abort check. Writes the per-axis rate-setpoint perturbation
 * (deg/s) to inject_out[3] — all zero when idle or aborted. Advances chirp phase,
 * auto-stops at duration, and aborts if a limit is exceeded. */
void sysid_step(float dt, const float angles_deg[3], const float rates_dps[3],
                float inject_out[3]);

/* High-rate capture (Phase 1). Call once per control tick AFTER the rate PID:
 * while a run is active it records the excited axis's (u, gyro) — u is the
 * rate-PID OUTPUT (the control effort the plant fit needs), gyro the measured
 * body rate — decimated to ~500 Hz into a bounded RAM buffer. No-op when idle.
 * Reset at each sysid_start(). */
void sysid_capture(const float u[3], const float gyro[3]);

/* No-op (kept so the telemetry-task call site is unchanged). Capture is RAM-only;
 * SD streaming was abandoned as too fragile on this stack. */
void sysid_flush_poll(void);

int sysid_capture_count(void); // total samples written to SD for the last run
int sysid_capture_hz(void);    // capture rate (Hz)
int sysid_capture_axis(void);  // excited axis of the last run

/* Post-run dump: stream the captured "0:sysid.bin" back over the link. Driven by
 * CMD_SYSID_DUMP; the telemetry task pulls chunks via sysid_dump_next(). */
void sysid_dump_request(void); // (re)start the dump from sample 0
int sysid_dump_active(void);   // non-zero while a dump is in progress
/* Read the next chunk of up to `cap` samples (0.1 deg/s, i16) FROM the SD file
 * into sp/gyro; returns the count read (0 when complete / on error) and sets
 * *start to this chunk's first sample index. Called from the telemetry task. */
int sysid_dump_next(uint16_t *start, int16_t *sp, int16_t *gyro, int cap);
