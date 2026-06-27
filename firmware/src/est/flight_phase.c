/**
 * @file flight_phase.c
 * @brief Takeoff / landing detector + FC-owned AGL ground reference — pure core.
 *
 * See include/est/flight_phase.h for the model, the gate rationale, and the
 * conventions. No HAL, no queues, no globals; the VERT task (src/est/
 * vertical_task.c) owns an instance, feeds it the fused estimate + throttle +
 * armed/in-air flags, and applies the returned event via system_state_set().
 */
#include "est/flight_phase.h"

#include "maths/maths_interface.h"

/* @noreq trivial flight-phase detector struct init. */
void flight_phase_init(flight_phase_t *fp) {
  fp->ground_ref = 0.0f;
  fp->have_ref = false;
  fp->agl = 0.0f;
  fp->powered = false;
  fp->takeoff_timer = 0.0f;
  fp->land_timer = 0.0f;
}

flight_phase_event_t flight_phase_update(flight_phase_t *fp, bool armed,
                                         bool in_air, float fused_alt,
                                         float baro_alt, float climb_rate,
                                         float throttle, float dt) {
  /* Ground reference: recapture continuously while disarmed (absorbs slow baro
   * drift), freeze the moment the craft commits to arm. Anchored to the
   * raw baro — instantaneous, so an arming/teleport transient in the fused
   * estimate can't poison it. On the ground AGL is 0 by construction, the
   * timers stay cleared, and the throttle latch resets. */
  if (!armed) {
    /* Seed on the first disarmed sample, then only REFINE while settled
     * (near-zero vertical motion) — the ground reference is "baro altitude when
     * the craft is at rest on the ground". This rejects a pre-arm handling bump
     * or a fall transient (e.g. the SITL respawn) freezing a bogus reference;
     * the last settled value is kept instead. */
    if (!fp->have_ref || m_fabsf(climb_rate) < FLIGHT_PHASE_LAND_RATE_MS) {
      fp->ground_ref = baro_alt;
      fp->have_ref = true;
    }
    fp->agl = 0.0f;
    fp->powered = false;
    fp->takeoff_timer = 0.0f;
    fp->land_timer = 0.0f;
    return FLIGHT_PHASE_EVENT_NONE;
  }

  fp->agl = fp->have_ref ? (fused_alt - fp->ground_ref) : 0.0f;

  /* Latch that lift was commanded since arm. A real takeoff drives throttle past
   * the gate before/while rising; a chop mid-coast (wobbly outer loop) then
   * can't veto the climb. A bench throttle blip latches this but never produces
   * the sustained AGL+climb the gate below also requires. */
  if (throttle > FLIGHT_PHASE_TAKEOFF_THROTTLE)
    fp->powered = true;

  if (dt <= 0.0f)
    return FLIGHT_PHASE_EVENT_NONE;

  if (!in_air) {
    /* Takeoff: altitude AND climb AND lift-was-commanded, sustained. */
    bool gate = fp->agl > FLIGHT_PHASE_TAKEOFF_ALT_M &&
                climb_rate > FLIGHT_PHASE_TAKEOFF_RATE_MS && fp->powered;
    fp->land_timer = 0.0f;
    if (!gate) {
      fp->takeoff_timer = 0.0f;
      return FLIGHT_PHASE_EVENT_NONE;
    }
    fp->takeoff_timer += dt;
    if (fp->takeoff_timer >= FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S) {
      fp->takeoff_timer = 0.0f;
      return FLIGHT_PHASE_EVENT_TAKEOFF;
    }
    return FLIGHT_PHASE_EVENT_NONE;
  }

  /* In air -> touchdown: settled near the ground, near-zero motion, throttle
   * backed off, sustained past the (longer) landing debounce. */
  bool gate = fp->agl < FLIGHT_PHASE_LAND_ALT_M &&
              m_fabsf(climb_rate) < FLIGHT_PHASE_LAND_RATE_MS &&
              throttle < FLIGHT_PHASE_LAND_THROTTLE;
  fp->takeoff_timer = 0.0f;
  if (!gate) {
    fp->land_timer = 0.0f;
    return FLIGHT_PHASE_EVENT_NONE;
  }
  fp->land_timer += dt;
  if (fp->land_timer >= FLIGHT_PHASE_LAND_DEBOUNCE_S) {
    fp->land_timer = 0.0f;
    return FLIGHT_PHASE_EVENT_LAND;
  }
  return FLIGHT_PHASE_EVENT_NONE;
}
