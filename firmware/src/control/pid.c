#include "control/pid.h"
#include "maths/maths_interface.h"
#include <stdbool.h>

/** @noreq PID struct field initialiser (infrastructure). */
void v_pid_init(struct PID *pid, float Kp, float Ki, float Kd, float Kff,
                float i_max, float d_max, float d_lpf_rc, float out_min,
                float out_max) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Kff = Kff;
  pid->i_max = i_max;
  pid->d_max = d_max;
  pid->d_lpf_rc = d_lpf_rc;
  pid->out_min = out_min;
  pid->out_max = out_max;
  pid->integral = 0;
  pid->prev_meas = 0;
  pid->d_filtered = 0;
  pid->sat_excess = 0.0f; /* no limit reported until a caller says otherwise */
  pid->initialized = false;
}

/**
 * Parallel-form P+I+D+FF step: integrator clamped to ±i_max during accumulation
 * and frozen when P+I saturates (anti-windup), derivative-on-measurement with a
 * first-order LPF (α = dt/(dt+d_lpf_rc)), output clamped to [out_min, out_max].
 *
 * @implements CTRL-PID-101
 */
float v_pid_update(struct PID *pid, float sp, float meas, float sp_dot,
                   float dt) {
  if (dt <= 1e-6f) {
    return 0;
  }
  if (!pid->initialized) {
    pid->prev_meas = meas;
    pid->d_filtered = 0;
    pid->initialized = true;
  }
  float error = sp - meas;

  // Proportional
  float P = pid->Kp * error;

  // Integral with clamped anti-windup
  float I =
      m_clamp(pid->integral + pid->Ki * error * dt, -pid->i_max, pid->i_max);

  // Derivative-on-measurement (avoids derivative kick on setpoint change)
  float D_raw = -pid->Kd * (meas - pid->prev_meas) / dt;

  // LPF
  float D;
  if (pid->d_lpf_rc > 1e-6f) {
    float alpha = dt / (dt + pid->d_lpf_rc);
    pid->d_filtered += alpha * (D_raw - pid->d_filtered);
    D = m_clamp(pid->d_filtered, -pid->d_max, pid->d_max);
  } else {
    D = m_clamp(D_raw, -pid->d_max, pid->d_max);
  }
  pid->prev_meas = meas;

  // Feedforward (setpoint velocity)
  float FF = pid->Kff * sp_dot;
  float output_pi = P + I;
  float output_full = output_pi + D + FF;

  /* Conditional integration, against BOTH limits that exist.
   *
   * The PID's own out_min/out_max is the weaker of the two and on a rate axis
   * it essentially never fires: authority is spent at the mixer, which sums
   * three axes onto the collective and saturates long before any single axis
   * reaches +-1. Measured on the aircraft 2026-09-16 while it was held tilted
   * by hand: all three integrators wound to their 0.2 clamps and pinned one
   * motor at 1.00 and another at the idle floor for eight seconds, while each
   * PID sat near 0.25 and its own freeze never triggered once.
   *
   * So the caller that owns the allocation reports what the actuator could not
   * deliver, and integration stops in THAT direction only -- winding out of the
   * limit stays allowed, which is what lets it recover as soon as the error
   * reverses. INDI has always done this with the realised differential; the PID
   * path was the gap. */
  const float SAT_EPS = 1e-4f;
  bool into_limit = (pid->sat_excess > SAT_EPS && error > 0.0f) ||
                    (pid->sat_excess < -SAT_EPS && error < 0.0f);
  if (!into_limit && output_pi > pid->out_min && output_pi < pid->out_max) {
    pid->integral = I;
  }
  return m_clamp(output_full, pid->out_min, pid->out_max);
}

/** @noreq actuator-saturation feedback (see the header). */
void v_pid_set_sat_excess(struct PID *pid, float excess) {
  if (pid != 0)
    pid->sat_excess = excess;
}

/** @noreq PID state-reset primitive (zeroes integrator + derivative state). */
void v_pid_reset(struct PID *pid) {
  pid->integral = 0;
  pid->sat_excess = 0.0f;
  pid->prev_meas = 0;
  pid->d_filtered = 0;
  pid->initialized = false;
}

/** @noreq trivial gain setter. */
void v_pid_set_gains(struct PID *pid, float Kp, float Ki, float Kd, float Kff) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Kff = Kff;
}

/** @noreq trivial output-limit setter. */
void v_pid_set_limits(struct PID *pid, float out_min, float out_max) {
  pid->out_min = out_min < out_max ? out_min : out_max;
  pid->out_max = out_min < out_max ? out_max : out_min;
}

/** @noreq trivial i_max setter. */
void v_pid_set_i_max(struct PID *pid, float i_max) { pid->i_max = i_max; }

/** @noreq trivial D-term LPF time-constant setter. */
void v_pid_set_d_lpf_rc(struct PID *pid, float d_lpf_rc) {
  pid->d_lpf_rc = (d_lpf_rc > 0.0f) ? d_lpf_rc : 0.0f;
  pid->d_filtered = 0; /* reset the filter state so the new RC starts clean */
}

/** @noreq trivial prev-measurement setter. */
void v_pid_set_prev_meas(struct PID *pid, float prev_meas) {
  pid->prev_meas = prev_meas;
  pid->initialized = false;
}

/** @noreq integral setter (clamped to ±i_max). */
void v_pid_set_integral(struct PID *pid, float integral) {
  pid->integral = m_clamp(integral, -pid->i_max, pid->i_max);
}
