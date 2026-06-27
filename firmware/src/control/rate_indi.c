/**
 * @file rate_indi.c
 * @brief INDI rate loop (see rate_indi.h for the theory). Per-axis, allocation-
 *        free, no locks — safe in the rate-loop hot path like the PID.
 */
#include "control/rate_indi.h"
#include "maths/maths_interface.h"

void rate_indi_init(rate_indi_t *c, float b, float k, float lpf_rc,
                    float out_min, float out_max) {
  /* b must be non-zero (we divide by it); guard a misconfig so the loop
   * degrades to "no command" rather than producing NaN that poisons the mix. */
  c->b      = (m_fabsf(b) > 1e-6f) ? b : 1e-6f;
  c->k      = k;
  c->lpf_rc = (lpf_rc > 0.0f) ? lpf_rc : 0.0f;
  c->out_min = (out_min < out_max) ? out_min : out_max;
  c->out_max = (out_min < out_max) ? out_max : out_min;
  rate_indi_reset(c);
}

void rate_indi_reset(rate_indi_t *c) {
  c->gyr_f       = 0.0f;
  c->wdot_f      = 0.0f;
  c->u_f         = 0.0f;
  c->initialized = false;
}

float rate_indi_update(rate_indi_t *c, float rate_sp, float rate_meas,
                       float dt) {
  /* First valid sample after a reset: seed the filters from the current state
   * and emit nothing. Derivative needs a previous sample; without this the
   * first wdot would be a huge (gyr - 0)/dt spike. */
  if (!c->initialized || dt <= 0.0f) {
    c->gyr_f       = rate_meas;
    c->wdot_f      = 0.0f;
    /* u_f left as-is (0 after reset) — the first command is just (wdot_des)/b */
    c->initialized = true;
    return 0.0f;
  }

  /* alpha for a first-order LPF with time constant lpf_rc (rc<=0 ⇒ passthrough).
   * The SAME alpha filters the gyro, the derivative, and the applied command so
   * all three share one delay (synchronized filtering). */
  const float alpha = (c->lpf_rc > 1e-6f) ? dt / (dt + c->lpf_rc) : 1.0f;

  /* 1. filter the gyro, then take its derivative -> measured angular accel. */
  const float gyr_prev = c->gyr_f;
  c->gyr_f += alpha * (rate_meas - c->gyr_f);
  const float wdot_meas = (c->gyr_f - gyr_prev) / dt;
  c->wdot_f += alpha * (wdot_meas - c->wdot_f);

  /* 2. desired angular accel from the single outer bandwidth gain. */
  const float wdot_des = c->k * (rate_sp - rate_meas);

  /* 3. incremental inversion: add the increment that closes the accel gap
   *    through the known effectiveness, on top of the (filtered) last command. */
  float u = c->u_f + (wdot_des - c->wdot_f) / c->b;

  /* 4. clamp to the mixer's authority. */
  if (u < c->out_min) u = c->out_min;
  if (u > c->out_max) u = c->out_max;

  /* 5. synchronized actuator feedback. Default to the clamped command; a caller
   *    that knows the post-mix realized differential can override via
   *    rate_indi_set_applied() AFTER the mix, which makes (3) saturation-aware
   *    (no integrating against thrust the motors never delivered). */
  c->u_f += alpha * (u - c->u_f);
  return u;
}

void rate_indi_set_applied(rate_indi_t *c, float u_applied) {
  /* Replace the synchronized feedback with the actually-applied command. Use
   * the same alpha-blend semantics as update()'s step 5 would have, but anchor
   * on the realized value. Kept simple: snap u_f toward the applied command. */
  if (!m_isfinite(u_applied)) return;
  const float a = (c->lpf_rc > 1e-6f) ? 0.5f : 1.0f; /* light blend if filtering */
  c->u_f += a * (u_applied - c->u_f);
}
