/**
 * @file tools/sim_host/tests/test_flight_phase.c
 * @brief Host verification suite for the takeoff/landing detector (flight_phase).
 *
 * Exercises the pure core in src/est/flight_phase.c (linked via vayu_sitl_core)
 * with synthetic vertical state + throttle so each property is checked against a
 * known ground truth:
 *
 *   FP-001  ground ref: recaptured from baro every step while disarmed; AGL == 0
 *   FP-002  ground ref freezes at arm; AGL = fused_alt - frozen_ref
 *   FP-003  takeoff needs altitude AND climb AND throttle-was-applied
 *   FP-004  takeoff fires only after the debounce window, then once (edge)
 *   FP-005  touchdown: settled gates sustained -> LAND event
 *   FP-006  no false takeoff from a high-but-static reading even with throttle
 *   FP-007  disarm mid-air clears the timers + the throttle latch
 *
 * Companion to test_vertical_est.c. Run via ctest or directly.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "est/flight_phase.h"

static int g_checks = 0;
static int g_fails = 0;

static void check(bool pass, const char *name) {
  g_checks++;
  if (pass) {
    printf("    ok   %s\n", name);
  } else {
    g_fails++;
    printf("    FAIL %s\n", name);
  }
}

/* Step the detector N times at dt with fixed inputs (baro tracks fused while
 * armed — the ground ref is already frozen). Return the FIRST non-NONE event. */
static flight_phase_event_t run(flight_phase_t *fp, bool armed, bool in_air,
                                float alt, float climb, float thr, float dt,
                                int n) {
  flight_phase_event_t first = FLIGHT_PHASE_EVENT_NONE;
  for (int i = 0; i < n; i++) {
    flight_phase_event_t e =
        flight_phase_update(fp, armed, in_air, alt, alt, climb, thr, dt);
    if (e != FLIGHT_PHASE_EVENT_NONE && first == FLIGHT_PHASE_EVENT_NONE)
      first = e;
  }
  return first;
}

/* Seed the ground reference from baro while disarmed at the given ground level. */
static void seed_ground(flight_phase_t *fp, float ground) {
  flight_phase_update(fp, false, false, ground, ground, 0.0f, 0.0f, 0.004f);
}

/* --- FP-001 / FP-002: ground reference capture + freeze ----------------- */
static void test_ground_ref(void) {
  flight_phase_t fp;
  flight_phase_init(&fp);

  /* Disarmed: ref follows the (drifting) baro; AGL pinned to 0. The fused value
   * is deliberately different to prove the ref tracks BARO, not fused. */
  flight_phase_update(&fp, false, false, 999.0f, 480.0f, 0.0f, 0.0f, 0.004f);
  flight_phase_update(&fp, false, false, 999.0f, 480.5f, 0.0f, 0.0f, 0.004f);
  check(fp.have_ref && fabsf(fp.ground_ref - 480.5f) < 1e-4f && fp.agl == 0.0f,
        "FP-001 ground ref tracks baro (not fused) while disarmed, AGL=0");

  /* Arm at 480.5 m: ref freezes; AGL = fused - frozen ref. */
  flight_phase_update(&fp, true, false, 481.5f, 481.5f, 0.0f, 0.2f, 0.004f);
  check(fabsf(fp.ground_ref - 480.5f) < 1e-4f && fabsf(fp.agl - 1.0f) < 1e-4f,
        "FP-002 ref frozen at arm; AGL = fused - frozen ref");
}

/* --- FP-003: takeoff needs all three signals ---------------------------- */
static void test_takeoff_gates(void) {
  const float dt = 0.004f;
  const int n = (int)(FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S / dt) + 20;
  const float G = 100.0f;
  const float ALT = G + FLIGHT_PHASE_TAKEOFF_ALT_M + 0.3f;
  const float CLB = FLIGHT_PHASE_TAKEOFF_RATE_MS + 0.3f;
  const float THR = FLIGHT_PHASE_TAKEOFF_THROTTLE + 0.2f;

  /* altitude only (no climb, no throttle) */
  flight_phase_t fp;
  flight_phase_init(&fp);
  seed_ground(&fp, G);
  check(run(&fp, true, false, ALT, 0.0f, 0.0f, dt, n) == FLIGHT_PHASE_EVENT_NONE,
        "FP-003 altitude alone does not trip takeoff");

  /* climb only (at ground level, no throttle) */
  flight_phase_init(&fp);
  seed_ground(&fp, G);
  check(run(&fp, true, false, G, CLB, 0.0f, dt, n) == FLIGHT_PHASE_EVENT_NONE,
        "FP-003 climb alone does not trip takeoff");

  /* throttle only (at ground level, no climb): latches powered but can't trip */
  flight_phase_init(&fp);
  seed_ground(&fp, G);
  check(run(&fp, true, false, G, 0.0f, THR, dt, n) == FLIGHT_PHASE_EVENT_NONE,
        "FP-003 throttle alone does not trip takeoff");

  /* all three -> takeoff */
  flight_phase_init(&fp);
  seed_ground(&fp, G);
  check(run(&fp, true, false, ALT, CLB, THR, dt, n) ==
            FLIGHT_PHASE_EVENT_TAKEOFF,
        "FP-003 altitude+climb+throttle trips takeoff");

  /* throttle LATCH: a brief throttle pulse then a chop (wobbly outer loop) still
   * takes off while AGL+climb hold — the latch is sticky. */
  flight_phase_init(&fp);
  seed_ground(&fp, G);
  flight_phase_update(&fp, true, false, ALT, ALT, CLB, THR, dt); /* powered */
  check(run(&fp, true, false, ALT, CLB, 0.0f, dt, n) ==
            FLIGHT_PHASE_EVENT_TAKEOFF,
        "FP-003 takeoff survives a throttle chop after lift was commanded");
}

/* --- FP-004: debounce timing + single edge ------------------------------ */
static void test_takeoff_debounce(void) {
  const float dt = 0.01f;
  const float G = 0.0f;
  const float ALT = FLIGHT_PHASE_TAKEOFF_ALT_M + 0.5f;
  const float CLB = FLIGHT_PHASE_TAKEOFF_RATE_MS + 0.5f;
  const float THR = FLIGHT_PHASE_TAKEOFF_THROTTLE + 0.2f;

  flight_phase_t fp;
  flight_phase_init(&fp);
  seed_ground(&fp, G);

  /* Just shy of the debounce window: no event yet. */
  int short_n = (int)(FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S / dt) - 2;
  flight_phase_event_t early =
      run(&fp, true, false, ALT, CLB, THR, dt, short_n);
  check(early == FLIGHT_PHASE_EVENT_NONE,
        "FP-004 no takeoff before the debounce window elapses");

  /* A couple more steps pushes past the window -> exactly one TAKEOFF edge.
   * Model the real caller: once TAKEOFF fires the system goes IN_AIR, so
   * subsequent updates pass in_air=true (running the landing test instead). */
  int events = 0;
  bool in_air = false;
  for (int i = 0; i < 50; i++) {
    if (flight_phase_update(&fp, true, in_air, ALT, ALT, CLB, THR, dt) ==
        FLIGHT_PHASE_EVENT_TAKEOFF) {
      events++;
      in_air = true;
    }
  }
  check(events == 1, "FP-004 takeoff fires exactly once (edge, not level)");
}

/* --- FP-005: touchdown -------------------------------------------------- */
static void test_landing(void) {
  const float dt = 0.01f;
  const int n = (int)(FLIGHT_PHASE_LAND_DEBOUNCE_S / dt) + 20;
  const float settled = FLIGHT_PHASE_LAND_ALT_M - 0.1f;
  const float low_thr = FLIGHT_PHASE_LAND_THROTTLE - 0.1f;

  flight_phase_t fp;
  flight_phase_init(&fp);
  seed_ground(&fp, 0.0f);

  /* In air, settled near the ground, throttle backed off -> LAND. */
  check(run(&fp, true, true, settled, 0.0f, low_thr, dt, n) ==
            FLIGHT_PHASE_EVENT_LAND,
        "FP-005 settled near ground + low throttle trips touchdown");

  /* Still descending fast does NOT count as landed. */
  flight_phase_init(&fp);
  seed_ground(&fp, 0.0f);
  check(run(&fp, true, true, settled, -1.0f, low_thr, dt, n) ==
            FLIGHT_PHASE_EVENT_NONE,
        "FP-005 fast descent near ground is not yet a touchdown");
}

/* --- FP-006: no false takeoff from a high-but-static reading ------------ */
static void test_no_false_takeoff(void) {
  const float dt = 0.01f;
  const int n = 500;
  const float THR = FLIGHT_PHASE_TAKEOFF_THROTTLE + 0.3f;
  flight_phase_t fp;
  flight_phase_init(&fp);
  seed_ground(&fp, 100.0f);
  /* High AGL and throttle applied, but ZERO climb: the climb gate must still
   * block (a stuck-high baro reading / prop-wash with the craft not rising). */
  check(run(&fp, true, false, 100.0f + 2.0f, 0.0f, THR, dt, n) ==
            FLIGHT_PHASE_EVENT_NONE,
        "FP-006 high altitude + throttle but no climb never trips takeoff");
}

/* --- FP-007: disarm mid-air clears timers + latch ----------------------- */
static void test_disarm_resets(void) {
  const float dt = 0.01f;
  const float ALT = FLIGHT_PHASE_TAKEOFF_ALT_M + 0.5f;
  const float CLB = FLIGHT_PHASE_TAKEOFF_RATE_MS + 0.5f;
  const float THR = FLIGHT_PHASE_TAKEOFF_THROTTLE + 0.2f;

  flight_phase_t fp;
  flight_phase_init(&fp);
  seed_ground(&fp, 0.0f);
  /* Accumulate most of the takeoff debounce (also latches powered)... */
  run(&fp, true, false, ALT, CLB, THR, dt,
      (int)(FLIGHT_PHASE_TAKEOFF_DEBOUNCE_S / dt) - 3);
  check(fp.takeoff_timer > 0.0f && fp.powered,
        "FP-007 timer accumulated + powered latched while armed");
  /* ...then disarm: the timer and the latch must clear. */
  flight_phase_update(&fp, false, false, 0.0f, 0.0f, 0.0f, 0.0f, dt);
  check(fp.takeoff_timer == 0.0f && fp.land_timer == 0.0f && !fp.powered,
        "FP-007 disarm clears the debounce timers and throttle latch");
}

int main(void) {
  printf("== flight_phase takeoff/landing detector verification ==\n");
  test_ground_ref();
  test_takeoff_gates();
  test_takeoff_debounce();
  test_landing();
  test_no_false_takeoff();
  test_disarm_resets();
  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
