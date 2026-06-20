/**
 * @file flight_phase.h
 * @brief Takeoff / landing detector and FC-owned AGL ground reference.
 *
 * Closes the gap the barometer finally unblocks (plan §5): driving the
 * SYSTEM_STATE_IN_AIR transition, which nothing could do before because the FC
 * had no vertical observability. This is a pure state machine — no HAL, no
 * queues, no globals — fed the fused vertical estimate (VERT), the commanded
 * throttle, and the armed/in-air flags; it returns a transition event and caches
 * the authoritative AGL. The VERT task owns one instance and applies the event
 * via system_state_set() (src/est/vertical_task.c).
 *
 * Ground reference (decision D4): the FC captures the ground altitude
 * continuously while disarmed (so it absorbs slow baro drift) and FREEZES it at
 * arm; AGL = fused_altitude - ground_ref. The reference is anchored to the RAW
 * baro altitude, not the fused estimate: baro is instantaneous (no accel
 * integration), so an arming transient can't poison the ground level. The GCS
 * consumes this authoritative AGL instead of computing its own.
 *
 * Detection (plan §5) requires altitude AND climb rate AND throttle to agree —
 * so baro noise, prop-wash, or a bench throttle blip can't false-trip it:
 *   - ARMED -> IN_AIR (takeoff):  agl > TAKEOFF, climb > +rate, AND lift was
 *     commanded since arm (throttle crossed the gate — a LATCH, not an
 *     instantaneous test, so a throttle chop mid-coast can't veto a real climb),
 *     all sustained past the takeoff debounce.
 *   - IN_AIR -> ARMED (touchdown): agl < LAND, |climb| < rate, throttle < gate,
 *     sustained past the (longer) landing debounce.
 *
 * Pure/host-testable: see tools/sim_host/tests/test_flight_phase.c.
 */
#ifndef VAYU_FLIGHT_PHASE_H
#define VAYU_FLIGHT_PHASE_H

#include <stdbool.h>

/* Takeoff gates: AGL (m), climb rate (m/s, up-positive), commanded throttle
 * (0..1), and the time all three must hold before the transition fires (s). The
 * throttle gate sits below the ~0.55 hover seed (angle_rate_controller.c:191) so
 * a real liftoff trips it but a bench idle does not. */
#ifndef FLIGHT_PHASE_TAKEOFF_ALT_M
#define FLIGHT_PHASE_TAKEOFF_ALT_M 0.4f
#endif
#ifndef FLIGHT_PHASE_TAKEOFF_RATE_MS
#define FLIGHT_PHASE_TAKEOFF_RATE_MS 0.3f
#endif
#ifndef FLIGHT_PHASE_TAKEOFF_THROTTLE
#define FLIGHT_PHASE_TAKEOFF_THROTTLE 0.35f
#endif
#ifndef FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S
#define FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S 0.3f
#endif

/* Touchdown gates: settled near the ground reference, near-zero vertical motion,
 * throttle backed off below hover, held for the (longer) landing debounce. */
#ifndef FLIGHT_PHASE_LAND_ALT_M
#define FLIGHT_PHASE_LAND_ALT_M 0.25f
#endif
#ifndef FLIGHT_PHASE_LAND_RATE_MS
#define FLIGHT_PHASE_LAND_RATE_MS 0.3f
#endif
#ifndef FLIGHT_PHASE_LAND_THROTTLE
#define FLIGHT_PHASE_LAND_THROTTLE 0.3f
#endif
#ifndef FLIGHT_PHASE_LAND_DEBOUNCE_S
#define FLIGHT_PHASE_LAND_DEBOUNCE_S 0.7f
#endif

typedef enum {
  FLIGHT_PHASE_EVENT_NONE = 0,
  FLIGHT_PHASE_EVENT_TAKEOFF, /**< request ARMED -> IN_AIR */
  FLIGHT_PHASE_EVENT_LAND,    /**< request IN_AIR -> ARMED */
} flight_phase_event_t;

typedef struct {
  float ground_ref;    /**< baro altitude of the ground (captured while disarmed). */
  bool have_ref;       /**< false until the first grounded sample seeds it. */
  float agl;           /**< last computed AGL (m) — cached for telemetry. */
  bool powered;        /**< latched true once throttle crossed the takeoff gate
                        *   since arming; cleared on disarm. */
  float takeoff_timer; /**< s the takeoff gates have held continuously. */
  float land_timer;    /**< s the landing gates have held continuously. */
} flight_phase_t;

/** Clear the detector; requires re-seeding the ground reference. */
void flight_phase_init(flight_phase_t *fp);

/**
 * @brief Advance the detector by one step.
 *
 * @param fp          detector instance.
 * @param armed       craft is committed (ARMED or IN_AIR) — freezes the ground
 *                    reference. When false the reference is recaptured each step.
 * @param in_air      craft is currently IN_AIR (selects landing vs takeoff test).
 * @param fused_alt   fused absolute altitude (m, up-positive) from VERT — the
 *                    AGL numerator.
 * @param baro_alt    raw baro altitude (m, up-positive) — the ground-reference
 *                    anchor (instantaneous, no integration transient).
 * @param climb_rate  fused climb rate (m/s, up-positive) from VERT.
 * @param throttle    commanded throttle (0..1).
 * @param dt          step interval (s).
 * @return            the detected transition event (FLIGHT_PHASE_EVENT_NONE if
 *                    no debounced edge this step). fp->agl is updated regardless.
 */
flight_phase_event_t flight_phase_update(flight_phase_t *fp, bool armed,
                                         bool in_air, float fused_alt,
                                         float baro_alt, float climb_rate,
                                         float throttle, float dt);

#endif /* VAYU_FLIGHT_PHASE_H */
