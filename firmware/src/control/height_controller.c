#include "control/height_controller.h"

/** @noreq Trivial clamp helper. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** @noreq Trivial state reset. */
void height_ctrl_reset(height_ctrl_t *h) {
  h->engaged = false;
  h->src_is_tof = false;
  h->landed = false;
  h->failed = false;
  h->runaway_t = 0.0f;
  h->handback = false;
  h->handback_thr = 0.0f;
  h->last_out = 0.0f;
  h->alt_sp = 0.0f;
  h->base = 0.0f;
  h->i = 0.0f;
}

/** @noreq Arm the hand-back at a SAFE collective: the last commanded value, but
 *  never above the hover baseline. See the CRITICAL note in the header — handing
 *  back a saturated runaway collective latches the runaway. */
static void handback_arm(height_ctrl_t *h) {
  float v = h->last_out;
  if (v > h->base) {
    v = h->base;
  }
  h->handback = true;
  h->handback_thr = v;
}

/** @noreq Hand collective back to the pilot without dropping it: hold the value
 *  armed above until their stick rises to meet it (see height_controller.h). */
static float handback_step(height_ctrl_t *h, float stick, bool in_air) {
  if (!h->handback) {
    return stick;
  }
  if (!in_air || stick >= h->handback_thr) {
    h->handback = false; /* on the ground, or the pilot has caught up */
    return stick;
  }
  return h->handback_thr;
}

/** @noreq Inner climb-rate PI + output clamp; shared by HOLD and LAND. */
static float climb_rate_pi(height_ctrl_t *h, float climb_sp, float climb_rate,
                           float dt) {
  float err = climb_sp - climb_rate;
  if (dt > 0.0f) {
    h->i = clampf(h->i + HEIGHT_KI_VZ * err * dt, -HEIGHT_I_MAX, HEIGHT_I_MAX);
  }
  float out = h->base + HEIGHT_KP_VZ * err + h->i;

  /* Clamp + back-calculation anti-windup: on a limit the integrator must not
   * keep charging, or it takes seconds to unwind and the craft overshoots the
   * other way. (Saturation is detected by the comparisons themselves, not by
   * testing lim != out — the build is -Werror=float-equal.) */
  if (out > HEIGHT_THR_MAX) {
    h->i = clampf(h->i - (out - HEIGHT_THR_MAX), -HEIGHT_I_MAX, HEIGHT_I_MAX);
    return HEIGHT_THR_MAX;
  }
  if (out < HEIGHT_THR_MIN) {
    h->i = clampf(h->i - (out - HEIGHT_THR_MIN), -HEIGHT_I_MAX, HEIGHT_I_MAX);
    return HEIGHT_THR_MIN;
  }
  return out;
}

/** @noreq Height-mode step; see height_controller.h. */
float height_ctrl_update(height_ctrl_t *h, height_mode_t mode, bool in_air,
                         float stick, float alt_m, bool alt_is_tof,
                         float climb_rate, float dt) {
  if (mode == HEIGHT_MODE_OFF) {
    /* State is dropped so the next engage re-captures a fresh baseline rather
     * than resuming a stale integrator, and centring the switch is what clears
     * the landed/failed latches. But if we were flying the aircraft a moment
     * ago, hand the collective back gently rather than dumping it on a stick
     * that is probably at the bottom. */
    bool was_flying_it = h->engaged && in_air;
    /* Work out the safe hand-back value BEFORE clearing the state — the clamp
     * reads h->base, and zeroing it first would hand back 0 (i.e. chop the
     * collective), which is the opposite of the point. */
    float safe_handback = h->last_out;
    if (safe_handback > h->base) {
      safe_handback = h->base;
    }
    /* Clear the MODE state but deliberately NOT the hand-back: OFF is evaluated
     * every loop, and a full reset here would wipe the re-sync on the very next
     * call and dump the collective after all. */
    h->engaged = false;
    h->src_is_tof = false;
    h->landed = false;
    h->failed = false;
    h->alt_sp = 0.0f;
    h->base = 0.0f;
    h->i = 0.0f;
    h->runaway_t = 0.0f;
    if (was_flying_it) {
      h->handback = true;
      h->handback_thr = safe_handback;
    }
    return handback_step(h, stick, in_air);
  }

  if (h->failed) {
    /* Gave up: the pilot has the collective back (via the same re-sync) until
     * they centre the switch, which is the only thing that clears this. */
    return handback_step(h, stick, in_air);
  }

  if (!h->engaged) {
    h->engaged = true;
    h->src_is_tof = alt_is_tof;
    h->i = 0.0f;
    h->landed = false;
    h->runaway_t = 0.0f;
    if (in_air) {
      /* Engaging in flight: hold the height we are at, and trust the pilot's
       * stick as the hover reference — they were flying it level. */
      h->alt_sp = alt_m;
      h->base = clampf(stick, HEIGHT_BASE_MIN, HEIGHT_BASE_MAX);
    } else {
      /* Lifting off: no stick reference to learn from (it is at the bottom),
       * so start from the airframe's nominal hover and let the PI trim. */
      h->alt_sp = HEIGHT_TARGET_M;
      h->base = HEIGHT_HOVER_GUESS;
    }
  }

  /* Source handoff (ToF dropping in/out of range). The two references have
   * different zeros, so carrying the old setpoint across would step the
   * throttle. Re-anchor to where we are now. Skipped while lifting off, where
   * the setpoint is a fixed target rather than a captured height. */
  if (alt_is_tof != h->src_is_tof) {
    h->src_is_tof = alt_is_tof;
    if (mode == HEIGHT_MODE_HOLD && in_air) {
      h->alt_sp = alt_m;
    }
  }

  /* Runaway guard — see HEIGHT_RUNAWAY_M. Checked after the handoff re-anchor
   * so a source change cannot false-trip it, and only once airborne (on the
   * ground the setpoint is a target we have not reached yet, not a height we
   * are failing to hold). */
  if (in_air && alt_m > h->alt_sp + HEIGHT_RUNAWAY_M) {
    h->runaway_t += (dt > 0.0f) ? dt : 0.0f;
    if (h->runaway_t >= HEIGHT_RUNAWAY_S) {
      h->failed = true;
      h->engaged = false;
      h->i = 0.0f;
      /* Don't drop the collective on the way out, and — just as important —
       * don't hand back the runaway thrust that got us here. */
      handback_arm(h);
      return handback_step(h, stick, in_air);
    }
  } else {
    h->runaway_t = 0.0f;
  }

  if (mode == HEIGHT_MODE_LAND) {
    /* Settle at a fixed sink rate until touchdown, then idle and latch — the
     * craft must not lift off again on its own, and the latch only clears when
     * the pilot centres the switch. `!in_air` covers the FC's own landing
     * detector firing before we reach the height threshold. */
    if (h->landed || alt_m <= HEIGHT_TOUCHDOWN_M || !in_air) {
      h->landed = true;
      h->i = 0.0f;
      h->last_out = HEIGHT_IDLE_THROTTLE;
      return HEIGHT_IDLE_THROTTLE;
    }
    h->last_out = climb_rate_pi(h, -HEIGHT_LAND_RATE_MS, climb_rate, dt);
    return h->last_out;
  }

  /* HOLD: climb toward the setpoint, then sit on it. */
  float climb_sp = clampf(HEIGHT_KP_ALT * (h->alt_sp - alt_m),
                          -HEIGHT_MAX_CLIMB_MS, HEIGHT_MAX_CLIMB_MS);
  h->last_out = climb_rate_pi(h, climb_sp, climb_rate, dt);
  return h->last_out;
}
