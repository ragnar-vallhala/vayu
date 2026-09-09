#ifndef VAYU_HEIGHT_CONTROLLER_H
#define VAYU_HEIGHT_CONTROLLER_H

#include <stdbool.h>

/*
 * Height mode (HGT) — a 3-position switch drives auto lift-off, height hold and
 * auto land. Pure math, no HAL/RTOS/globals: the caller
 * (angle_controller_task) owns the instance and wires the I/O, the way
 * vertical_task.c owns the vertical estimator.
 *
 * THE STICKS ARE UNTOUCHED. Roll, pitch and yaw are always the pilot's, and the
 * collective stick is the pilot's too whenever the mode switch is centred. This
 * module only takes the collective while a mode is actually selected:
 *
 *   centre -> OFF   pilot flies it, stick passes straight through
 *   up     -> HOLD  from the ground: lift to HEIGHT_TARGET_M and hold there
 *                   already flying: hold the height you are at right now
 *   down   -> LAND  descend at HEIGHT_LAND_RATE_MS until touchdown, then idle
 *
 * Cascade underneath: height error -> climb-rate setpoint -> PI -> collective,
 * around a hover baseline.
 *
 * The hover baseline. Engaging IN FLIGHT captures the pilot's own stick — they
 * were holding height, so their stick IS hover, and that is the calibration knob
 * (no per-airframe constant, and it re-captures every engage so battery sag is
 * absorbed). Lifting off from the GROUND has no such reference, so it starts
 * from HEIGHT_HOVER_GUESS and lets the integrator find the rest.
 *
 * SAFETY — why the output clamp exists. A pilot chopping the collective on
 * 2026-09-04 dropped it under PID_FULL_AUTHORITY_THROTTLE, where the rate loop
 * fades out its own authority ("PID authority ramp") — the stabiliser switched
 * OFF, the airframe tipped, tripped the 70 deg bank failsafe, and had its motors
 * cut. The PRIMARY fix for that is in angle_rate_controller.c, which now floors
 * the ramp input while IN_AIR, so authority survives any commanded collective.
 * HEIGHT_THR_MIN is the second line of defence: an automatic controller should
 * not be commanding a collective that low in the first place. (It is a plain
 * constant rather than being tied to PID_FULL_AUTHORITY_THROTTLE because that
 * threshold differs per build — 0.20 on hardware, 0.45 under VAYU_SIM — and
 * this module stays free of variables.h so it can be unit-tested standalone.)
 * The one exception is after touchdown, where idling the motors is the point.
 *
 * HAND-BACK / throttle re-sync. When the mode stops flying the aircraft while it
 * is still airborne — the switch is centred, or the runaway guard gives up — the
 * pilot's collective stick is very likely at the BOTTOM, because during an auto
 * lift-off they were never holding it. Returning that stick verbatim would chop
 * the collective to idle in mid-air, which is the exact accident this whole
 * feature exists to prevent. So on hand-back the controller HOLDS its last
 * commanded collective until the pilot's stick rises to meet it, then gets out
 * of the way. Standard throttle re-sync: the stick has to pass through the held
 * value to take control. Once on the ground it hands back immediately.
 *
 * CRITICAL: the held value is NEVER above the hover baseline. Handing back
 * `last_out` verbatim is what turned the runaway guard into a latched
 * full-throttle climb on 2026-09-04 — the guard correctly gave up, then held the
 * saturated collective it had been commanding until the pilot's idle stick rose
 * to meet it, and the craft climbed into the ceiling. Whatever went wrong, the
 * safe thing to hand a pilot is roughly hover, never more.
 *
 * Altitude source is the caller's choice: ToF AGL below ~1.5 m, baro AGL above.
 * `alt_is_tof` says which, so the setpoint can be re-anchored on a handoff
 * instead of stepping the throttle when the two references disagree (they
 * always do — different zero, different drift).
 */

/* Height held by HOLD when lifting off from the ground (m AGL). Chosen to sit
 * inside the rangefinder's band, so the hold runs on the ToF, not the baro. */
#define HEIGHT_TARGET_M 1.0f

/* Climb rate ceiling while lifting to the target (m/s). Deliberately gentle —
 * this is a 1 m hop, not a launch. */
#define HEIGHT_MAX_CLIMB_MS 0.6f

/* Runaway guard. If the craft ends up this far ABOVE the setpoint, the mode is
 * not in control of the aircraft — hand the collective straight back to the
 * pilot and latch off until the switch is centred.
 *
 * This exists because a hover-referenced controller silently assumes the
 * airframe's hover throttle is near HEIGHT_HOVER_GUESS and above
 * HEIGHT_THR_MIN. SITL 2026-09-04 flew a vehicle with roughly 3x the real
 * thrust-to-weight (hover ~0.17): its hover sat BELOW the output floor, so the
 * controller's own minimum was a climb command and it flew away at 200 m/s with
 * every loop behaving "correctly". Nothing else in the design would have caught
 * that, and on a real airframe the same thing happens after a prop/battery/
 * weight change big enough to move hover. An automatic mode needs to be able to
 * notice it is not working and give up. */
#define HEIGHT_RUNAWAY_M 2.0f
/* ...but only after the excursion has PERSISTED this long. A single sample over
 * the line means nothing — baro AGL carries metres of noise, and tripping on one
 * bad reading is precisely the mistake the old bank-angle failsafe made (see
 * MAX_ANGLE_RECOVER in variables.h). A real runaway stays out of bounds. */
#define HEIGHT_RUNAWAY_S 1.0f

/* Descent rate commanded in LAND (m/s), and the height below which we call it
 * touchdown and idle the motors. */
#define HEIGHT_LAND_RATE_MS 0.30f
#define HEIGHT_TOUCHDOWN_M 0.12f

/* Height -> climb-rate outer gain (1/s). */
#define HEIGHT_KP_ALT 1.0f

/* Climb-rate -> collective inner PI (throttle units per m/s). */
#define HEIGHT_KP_VZ 0.12f
#define HEIGHT_KI_VZ 0.10f
#define HEIGHT_I_MAX 0.25f

/* Output clamp. MIN must stay >= PID_FULL_AUTHORITY_THROTTLE (0.20 on hw) —
 * see the safety note above. MAX leaves the mixer room for attitude. */
#define HEIGHT_THR_MIN 0.25f
#define HEIGHT_THR_MAX 0.85f

/* Collective commanded once touched down: below the authority floor on purpose,
 * because by then we are on the ground and no longer IN_AIR. */
#define HEIGHT_IDLE_THROTTLE 0.0f

/* Compiled-in FALLBACK hover for a lift-off. Callers pass the live measured
 * hover to height_ctrl_update() instead (est/hover_estimate.h, persisted across
 * reboots by storage/hover_store.h); this is only what the estimator itself is
 * seeded with the very first time an airframe flies.
 *
 * The integrator trims a wrong value, but only within +/-HEIGHT_I_MAX -- so a
 * badly wrong one is not recoverable, it just flies.
 *
 * 0.38 for the 5in racer (650 g AUW, 220 mm wheelbase, 2400KV on 3S, 5x4.3x3):
 * ~1.10 kg static thrust per motor gives TWR ~6.8, and since thrust goes as
 * duty^2, hover duty = sqrt(1/6.8) = 0.384.
 *
 * This was 0.50, carried over from the 10in S500 airframe, and that is the
 * likeliest single cause of the 2026-09-04 ceiling climb: (0.50/0.384)^2 = 1.69x
 * hover thrust, i.e. ~6.8 m/s^2 of net climb from the guess alone, before any
 * mixer or saturation effect. RE-DERIVE THIS WHENEVER THE AIRFRAME CHANGES.
 * Measured hover throttle beats the estimate -- thrust per motor is the least
 * certain input (0.9-1.3 kg/motor moves hover to 0.43-0.35). */
#define HEIGHT_HOVER_GUESS 0.38f

/* In-flight engage captures the stick, clamped to this sane band so engaging
 * with the stick parked somewhere absurd cannot inject a step. */
#define HEIGHT_BASE_MIN 0.30f
#define HEIGHT_BASE_MAX 0.75f

typedef enum {
  HEIGHT_MODE_OFF = 0, /**< switch centred — pilot has the collective. */
  HEIGHT_MODE_HOLD,    /**< switch up — lift to target / hold current height. */
  HEIGHT_MODE_LAND,    /**< switch down — descend and settle. */
} height_mode_t;

typedef struct {
  bool engaged;    /**< controller currently driving the collective. */
  bool src_is_tof; /**< which reference alt_sp is expressed in. */
  bool landed; /**< LAND has touched down; latched until the switch centres. */
  bool
      lifting_off; /**< engaged from the ground and still climbing to the target.
                    *   While set, a source handoff must NOT re-anchor alt_sp:
                    *   the setpoint is a fixed target, not a height being held.
                    *   `in_air` cannot serve here — flight_phase declares IN_AIR
                    *   at 0.15 m, so most of the climb to 1.0 m is airborne. */
  bool failed;     /**< runaway detected; latched until the switch centres. */
  float
      runaway_t; /**< seconds the height has persisted above the guard band. */
  bool handback; /**< holding collective until the pilot's stick catches up. */
  float handback_thr; /**< the collective being held during hand-back. */
  float last_out;     /**< last collective this controller commanded. */
  float alt_sp;       /**< held height setpoint (m, in the current source). */
  float base;         /**< hover-throttle baseline. */
  float i;            /**< climb-rate integrator (throttle units). */
} height_ctrl_t;

/* Drop to disengaged. Safe to call every loop. */
void height_ctrl_reset(height_ctrl_t *h);

/* One control step. Returns the collective to command.
 *
 *   mode        selected height mode (already interlocked by the caller)
 *   in_air      FC believes it is flying (drives lift-off vs in-flight engage)
 *   stick       pilot collective [0,1] — returned unchanged in OFF
 *   alt_m       current height (m); ToF AGL or baro AGL, caller's choice
 *   alt_is_tof  true when alt_m came from the rangefinder
 *   climb_rate  fused climb rate (m/s, up-positive)
 *   dt          step (s)
 *   hover_ref   collective that hovers this airframe RIGHT NOW -- the measured
 *               estimate when there is one, else HEIGHT_HOVER_GUESS. Used as
 *               the lift-off baseline, where there is no pilot stick to learn
 *               hover from. Out-of-band values fall back to the constant.
 */
float height_ctrl_update(height_ctrl_t *h, height_mode_t mode, bool in_air,
                         float stick, float alt_m, bool alt_is_tof,
                         float climb_rate, float dt, float hover_ref);

#endif // VAYU_HEIGHT_CONTROLLER_H
