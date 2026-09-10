#include "control/sysid.h"

#include <stddef.h> /* size_t (was reached transitively) */

#include "maths/maths_interface.h"

/* Hard safety caps (the physical rig should also have soft-stops). These bound
 * what any CMD_SYSID_EXCITE can do regardless of the requested values. */
#define SYSID_MAX_AMP_DPS                                                      \
  120.0f // clamp commanded amplitude, RATE_SP mode (deg/s)
#define SYSID_MAX_AMP_U                                                        \
  0.30f // clamp commanded amplitude, U mode (control effort,
        // u ~O(1) full scale; this is ~30% authority — start lower)
#define SYSID_MAX_F_HZ 40.0f        // clamp chirp frequencies
#define SYSID_MAX_DURATION_S 12.0f  // clamp excitation length
#define SYSID_ABORT_ANGLE_DEG 35.0f // |angle| on the excited axis -> abort
#define SYSID_ABORT_RATE_DPS 300.0f // |rate|  on the excited axis -> abort
#define SYSID_TAPER_S 0.3f          // cosine-free ramp in/out (avoid step kick)

/* Capture is RAM-only (SD streaming proved too fragile on this stack: FF_FS_LOCK=0
 * lets a racing open corrupt the file). A bounded buffer keeps it simple and
 * reliable; 2 s at 500 Hz = 1000 samples * 2 int16 = 4 KB, well under the RAM that
 * overflowed a task stack at 16 KB. Plenty for a rate-loop plant fit. */
#define SYSID_DECIM 2
#define SYSID_CAP_HZ (1000 / SYSID_DECIM)
#define SYSID_CAP_N 1000 // 2 s at 500 Hz

static volatile int s_active = 0;
static uint8_t s_axis = 0;
static uint8_t s_mode = SYSID_INJECT_RATE_SP;
static float s_f0, s_f1, s_amp, s_dur;
static float s_elapsed;
static float s_phase; // chirp phase (rad)

/* Interleaved (u, gyro) i16: slot 0 = rate-PID output u (x1000, SYSID_U_SCALE),
 * slot 1 = gyro (0.1 deg/s, SYSID_W_SCALE). Written by the 1 kHz control loop,
 * read back by the telemetry task only AFTER the run ends (no concurrent access). */
static int16_t s_cap[SYSID_CAP_N * 2];
static int s_cap_n = 0; // samples captured this run
static int s_decim = 0;

static int s_dump_active = 0;
static int s_dump_pos = 0; // next sample index to stream

/** @noreq scalar clamp helper. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** @noreq i16 quantise helper. */
static int16_t to_i16(float v, float scale) { // v*scale, clamped to i16
  float s = v * scale;
  s = clampf(s, -32768.0f, 32767.0f);
  return (int16_t)s;
}
/* Slot scales: u (rate-PID output, ~O(1)) at x1000 for resolution; gyro (deg/s)
 * at x10 (0.1 dps). The host divides by these to recover native units, so the
 * fitted K and designed gains stay consistent with the firmware PID. */
#define SYSID_U_SCALE 1000.0f
#define SYSID_W_SCALE 10.0f

/* @implements CTRL-SID-001, CTRL-SID-101 */
void sysid_start(const sysid_request_t *req) {
  if (req == 0 || req->axis > 2)
    return;
  s_axis = req->axis;
  s_mode =
      (req->mode == SYSID_INJECT_U) ? SYSID_INJECT_U : SYSID_INJECT_RATE_SP;
  s_f0 = clampf(req->f0_hz, 0.1f, SYSID_MAX_F_HZ);
  s_f1 = clampf(req->f1_hz, s_f0, SYSID_MAX_F_HZ);
  // Amplitude cap is mode-dependent: deg/s for the rate setpoint, control-effort
  // for direct u-injection (which drives the motors far harder for the same number).
  s_amp =
      clampf(req->amp_dps, 0.0f,
             s_mode == SYSID_INJECT_U ? SYSID_MAX_AMP_U : SYSID_MAX_AMP_DPS);
  s_dur = clampf(req->duration_s, 0.1f, SYSID_MAX_DURATION_S);
  s_elapsed = 0.0f;
  s_phase = 0.0f;
  s_cap_n = 0; // fresh RAM capture for this run
  s_decim = 0;
  s_dump_active = 0;
  s_active = 1;
}

/** @noreq abort-flag setter. */
void sysid_abort(void) { s_active = 0; }

/** @noreq active-flag accessor. */
int sysid_active(void) { return s_active; }

/** @noreq inject-mode accessor. */
int sysid_inject_mode(void) { return s_mode; }

/* @implements CTRL-SID-001, CTRL-SID-101 */
void sysid_step(float dt, const float angles_deg[3], const float rates_dps[3],
                float inject_out[3]) {
  inject_out[0] = inject_out[1] = inject_out[2] = 0.0f;
  if (!s_active)
    return;
  // Reject a pathological dt so a stalled loop can't integrate a huge phase step.
  if (dt <= 0.0f || dt > 0.1f)
    return;

  // Self-abort: a limit exceedance on the excited axis ends the run immediately
  // (injection goes to zero, the rate loop recovers the craft to its setpoint).
  float a = angles_deg[s_axis];
  float r = rates_dps[s_axis];
  if (a < 0.0f)
    a = -a;
  if (r < 0.0f)
    r = -r;
  if (a > SYSID_ABORT_ANGLE_DEG || r > SYSID_ABORT_RATE_DPS) {
    s_active = 0;
    return;
  }

  s_elapsed += dt;
  if (s_elapsed >= s_dur) {
    s_active = 0;
    return;
  }

  // Linear chirp: instantaneous frequency sweeps f0 -> f1 across the duration;
  // integrate it into a phase so the waveform is continuous.
  float frac = s_elapsed / s_dur; // 0..1
  float f = s_f0 + (s_f1 - s_f0) * frac;
  s_phase += 2.0f * 3.14159265f * f * dt;

  // Ramp the amplitude in and out over SYSID_TAPER_S so the motors never get a
  // step kick at the start/end of the excitation.
  float env = 1.0f;
  if (s_elapsed < SYSID_TAPER_S)
    env = s_elapsed / SYSID_TAPER_S;
  else if (s_elapsed > s_dur - SYSID_TAPER_S)
    env = (s_dur - s_elapsed) / SYSID_TAPER_S;
  env = clampf(env, 0.0f, 1.0f);

  inject_out[s_axis] = s_amp * env * m_sin(s_phase);
}

/* --- capture: RAM ring, called from the 1 kHz control loop. `u` is the rate-PID
 * output (control effort); `gyro` the measured body rate (deg/s). ---
 * @implements CTRL-SID-102 */
void sysid_capture(const float u[3], const float gyro[3]) {
  if (!s_active)
    return;
  if (++s_decim < SYSID_DECIM)
    return;
  s_decim = 0;
  if (s_cap_n >= SYSID_CAP_N)
    return; // buffer full (run capped at 2 s of capture)
  s_cap[(size_t)s_cap_n * 2] = to_i16(u[s_axis], SYSID_U_SCALE);
  s_cap[s_cap_n * 2 + 1] = to_i16(gyro[s_axis], SYSID_W_SCALE);
  s_cap_n++;
}

/** @noreq capture-count accessor. */
int sysid_capture_count(void) { return s_cap_n; }
/** @noreq capture-rate accessor. */
int sysid_capture_hz(void) { return SYSID_CAP_HZ; }
/** @noreq excited-axis accessor. */
int sysid_capture_axis(void) { return s_axis; }

/* No-op: capture is RAM-only, so there is nothing to flush. Present so the
 * telemetry task has a stable call site.
 * @noreq no-op stub (stable call site). */
void sysid_flush_poll(void) {}

/* --- dump: stream the RAM buffer back, called from the telemetry task --- */
/** @noreq dump-cursor init (glue for the proposed CTRL-SID-102 readout). */
void sysid_dump_request(void) {
  s_dump_pos = 0;
  s_dump_active = (s_cap_n > 0);
}

/** @noreq dump-in-progress accessor. */
int sysid_dump_active(void) { return s_dump_active; }

/* @implements CTRL-SID-102 */
int sysid_dump_next(uint16_t *start, int16_t *u, int16_t *gyro, int cap) {
  if (!s_dump_active || s_dump_pos >= s_cap_n) {
    s_dump_active = 0;
    return 0;
  }
  if (cap > 10)
    cap = 10;
  int n = s_cap_n - s_dump_pos;
  if (n > cap)
    n = cap;
  *start = (uint16_t)s_dump_pos;
  for (int i = 0; i < n; i++) {
    /* Both terms are non-negative by construction (s_dump_pos is a cursor, i a
     * loop counter), so widen once and index from that -- the pair is two slots
     * of one record, not two independent offsets. */
    const size_t k = ((size_t)s_dump_pos + (size_t)i) * 2u;
    u[i] = s_cap[k];         // slot 0 = rate-PID output u (x1000)
    gyro[i] = s_cap[k + 1u]; // slot 1 = gyro (0.1 deg/s)
  }
  s_dump_pos += n;
  if (s_dump_pos >= s_cap_n)
    s_dump_active = 0;
  return n;
}
