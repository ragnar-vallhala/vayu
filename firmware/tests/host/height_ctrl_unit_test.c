/* height_ctrl_unit_test.c — host unit test for the height-mode core.
 *
 * Pure function under test: height_ctrl_update(). No HAL/RTOS — links only
 * height_controller.c. The behaviours that matter are the safety ones: the
 * controller must never command a collective below the throttle at which the
 * rate loop fades out its own authority (except once it has touched down), it
 * must not lift off again after landing, and it must not step the throttle when
 * the height reference hands over between the rangefinder and the baro.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/height_ctrl_unit_test.c \
 *       src/control/height_controller.c -lm -o /tmp/ht && /tmp/ht
 */
#include "control/height_controller.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}
static int near(float a, float b, float tol) { return fabsf(a - b) < tol; }

#define DT 0.002f
#define STICK_HOVER 0.50f
#define GROUND 0.05f

int main(void) {
  printf("height mode controller\n");

  /* 1. Centre = OFF: the pilot's stick passes straight through. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    float out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.73f, 1.0f, true,
                                   0.0f, DT, HEIGHT_HOVER_GUESS);
    check("OFF passes the stick through", near(out, 0.73f, 1e-6f));
    check("OFF stays disengaged", !h.engaged);
  }

  /* 2. Up from the GROUND = lift off to the target height. No pilot stick to
   *    learn hover from, so it starts at the nominal guess and climbs. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    float out = height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND,
                                   true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("lift-off targets HEIGHT_TARGET_M",
          near(h.alt_sp, HEIGHT_TARGET_M, 1e-6f));
    check("lift-off starts from the hover guess, not the bottomed stick",
          near(h.base, HEIGHT_HOVER_GUESS, 1e-6f));
    check("lift-off commands more than hover to climb",
          out > HEIGHT_HOVER_GUESS);
  }

  /* 3. Up while ALREADY FLYING = hold the height you are at, and trust the
   *    pilot's stick as the hover reference. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    float out = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER,
                                   2.40f, false, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("in-flight engage holds the current height",
          near(h.alt_sp, 2.40f, 1e-6f));
    check("in-flight engage captures the stick as hover",
          near(h.base, STICK_HOVER, 1e-6f));
    check("in-flight engage at the setpoint holds the baseline",
          near(out, STICK_HOVER, 1e-3f));
  }

  /* 4. Converging on the target: below it asks for more thrust, above it less. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    float low = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER,
                                   0.7f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("sagging below the setpoint asks for more thrust", low > STICK_HOVER);
    float high = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER,
                                    1.3f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("floating above the setpoint asks for less thrust",
          high < STICK_HOVER);
  }

  /* 5. THE SAFETY CLAMP. Airborne and climbing away hard, so the controller
   *    wants a big negative correction — it must still never command below
   *    HEIGHT_THR_MIN, which is what keeps the rate loop at authority. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    float lowest = 1.0f;
    for (int i = 0; i < 5000; i++) {
      float out =
          height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 8.0f,
                             false, +3.0f, DT, HEIGHT_HOVER_GUESS);
      if (out < lowest)
        lowest = out;
    }
    check("HOLD never commands below the authority floor",
          lowest >= HEIGHT_THR_MIN - 1e-6f);
    check("the floor is at or above the rate loop's full-authority throttle",
          HEIGHT_THR_MIN >= 0.20f);
    check("integrator stays bounded under sustained saturation",
          fabsf(h.i) <= HEIGHT_I_MAX + 1e-6f);
  }

  /* 6. Down = LAND: sink while airborne, but never below the authority floor
   *    on the way down. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    float out = height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER,
                                   1.0f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("LAND commands less than hover to descend", out < STICK_HOVER);
    float lowest = 1.0f;
    for (int i = 0; i < 2000; i++) {
      float o = height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER,
                                   0.9f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
      if (o < lowest)
        lowest = o;
    }
    check("LAND holds the authority floor while still airborne",
          lowest >= HEIGHT_THR_MIN - 1e-6f);
  }

  /* 7. Touchdown idles the motors and LATCHES — a bounce, a noisy range, or the
   *    craft being nudged must not make it lift off again. Only centring the
   *    switch releases it. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    float out = height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER,
                                   0.10f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("touchdown idles the motors", near(out, HEIGHT_IDLE_THROTTLE, 1e-6f));
    check("touchdown latches", h.landed);
    out = height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER, 0.60f,
                             true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("latched: a bounce does not command thrust again",
          near(out, HEIGHT_IDLE_THROTTLE, 1e-6f));
    out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.42f, 0.60f, true,
                             0.0f, DT, HEIGHT_HOVER_GUESS);
    check("centring the switch releases the latch and returns the stick",
          !h.landed && near(out, 0.42f, 1e-6f));
  }

  /* 8. The FC's own landing detector dropping IN_AIR also counts as down. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_LAND, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    float out = height_ctrl_update(&h, HEIGHT_MODE_LAND, false, STICK_HOVER,
                                   0.8f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("losing IN_AIR counts as touchdown",
          h.landed && near(out, HEIGHT_IDLE_THROTTLE, 1e-6f));
  }

  /* 9. Source handoff in flight (ToF drops out of range -> baro). The two
   *    references disagree by metres; the setpoint must re-anchor rather than
   *    carry a stale value across and step the throttle. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.2f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    float before = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER,
                                      1.2f, true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    float after =
        height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 41.7f,
                           false, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("handoff re-anchors the setpoint", near(h.alt_sp, 41.7f, 1e-6f));
    check("handoff does not step the throttle", near(after, before, 0.02f));
  }

  /* 10. A lift-off must NOT re-anchor to a mid-climb handoff — its setpoint is
   *     the fixed target, not wherever it happens to be.
   *
   *     BOTH in_air states matter here. flight_phase declares IN_AIR at 0.15 m
   *     (FLIGHT_PHASE_TOF_TAKEOFF_ALT_M), so the whole climb from there to the
   *     1.0 m target runs airborne — testing only the grounded case exercises
   *     the one path that was never broken. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, HEIGHT_HOVER_GUESS);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, 0.30f, false, 0.4f,
                       DT, HEIGHT_HOVER_GUESS);
    check("lift-off keeps its target across a source change (on the ground)",
          near(h.alt_sp, HEIGHT_TARGET_M, 1e-6f));

    /* Same climb, now past the 0.15 m IN_AIR threshold: a ToF dropout at 0.30 m
     * (tilt > 30 deg, a dark floor, 200 ms of staleness) must not abort the
     * lift-off by adopting 0.30 m as the target. */
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, HEIGHT_HOVER_GUESS);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.0f, 0.20f, true, 0.5f, DT,
                       HEIGHT_HOVER_GUESS);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.0f, 0.30f, false, 0.5f, DT,
                       HEIGHT_HOVER_GUESS);
    check("lift-off survives a ToF dropout AFTER IN_AIR is declared",
          near(h.alt_sp, HEIGHT_TARGET_M, 1e-6f));

    /* ...but once it HAS reached the target, it is holding, not lifting off —
     * from then on a handoff must re-anchor as normal. */
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.0f, 1.02f, false, 0.0f, DT,
                       HEIGHT_HOVER_GUESS);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.0f, 1.40f, true, 0.0f, DT,
                       HEIGHT_HOVER_GUESS);
    check("after arriving, a handoff re-anchors normally",
          near(h.alt_sp, 1.40f, 1e-6f));
  }

  /* 11. Runaway guard: if the craft ends up well above the setpoint and STAYS
   *     there, the mode is not in control — hand the collective back and latch
   *     off until the switch is centred. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    /* A brief excursion must NOT trip it (baro AGL is metres-noisy). */
    for (int i = 0; i < 100; i++) /* 0.2 s */
      height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 9.0f, true,
                         0.0f, DT, HEIGHT_HOVER_GUESS);
    check("a brief excursion does not trip the runaway guard", !h.failed);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, HEIGHT_HOVER_GUESS);
    check("dropping back in bounds resets the timer",
          near(h.runaway_t, 0.0f, 1e-6f));
    /* Sustained, though, is a runaway. */
    float out = 0.0f;
    for (int i = 0; i < 1000; i++) /* 2 s */
      out = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.42f, 9.0f, true,
                               +5.0f, DT, HEIGHT_HOVER_GUESS);
    check("a sustained excursion trips the guard", h.failed);
    check("giving up returns the pilot's stick", near(out, 0.42f, 1e-6f));
    out = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.31f, 1.0f, true,
                             0.0f, DT, HEIGHT_HOVER_GUESS);
    check("stays given up even back in bounds", near(out, 0.31f, 1e-6f));
    out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.31f, 1.0f, true, 0.0f,
                             DT, HEIGHT_HOVER_GUESS);
    check("centring the switch clears the failure latch", !h.failed);
  }

  /* 12. The guard must not fire during a normal lift-off, where the craft is
   *     legitimately below a target it has not reached yet. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    for (int i = 0; i < 1000; i++)
      height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, 0.05f, true, 0.0f,
                         DT, HEIGHT_HOVER_GUESS);
    check("lift-off below the target never trips the guard", !h.failed);
  }

  /* 13. Hand-back must not chop the collective. During an auto lift-off the
   *     pilot's stick is at the BOTTOM; centring the switch mid-air has to hold
   *     the last commanded collective until their stick catches up, or we
   *     recreate the very accident this feature exists to prevent. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    /* fly it: engage in air at hover and settle */
    float held = 0.0f;
    for (int i = 0; i < 50; i++)
      held = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f,
                                true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    /* pilot centres the switch with the stick parked at idle */
    float out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.0f, 1.0f, true,
                                   0.0f, DT, HEIGHT_HOVER_GUESS);
    check("centring mid-air does NOT dump to an idle stick", out > 0.4f);
    check("it holds the last commanded collective", near(out, held, 1e-3f));
    /* compare against the KNOWN hover, not h.base — OFF has zeroed it by now */
    check("a normal hand-back is at or below hover",
          out <= STICK_HOVER + 1e-3f);
    /* pilot raises the stick but not yet to the held value */
    out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.30f, 1.0f, true, 0.0f,
                             DT, HEIGHT_HOVER_GUESS);
    check("still held while the stick is below it", near(out, held, 1e-3f));
    /* stick catches up -> pilot has it */
    out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.62f, 1.0f, true, 0.0f,
                             DT, HEIGHT_HOVER_GUESS);
    check("stick catching up takes control", near(out, 0.62f, 1e-6f));
    out = height_ctrl_update(&h, HEIGHT_MODE_OFF, true, 0.10f, 1.0f, true, 0.0f,
                             DT, HEIGHT_HOVER_GUESS);
    check("after catching up the stick is followed down",
          near(out, 0.10f, 1e-6f));
  }

  /* 14. The runaway guard hands back the same way — giving up must not chop. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    float held = 0.0f;
    for (int i = 0; i < 50; i++)
      held = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f,
                                true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    float out = 0.0f;
    for (int i = 0; i < 1000; i++) /* sustained runaway, stick at idle */
      out = height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, 0.0f, 9.0f, true,
                               +5.0f, DT, HEIGHT_HOVER_GUESS);
    check("runaway hand-back does not dump to an idle stick",
          h.failed && out > 0.2f);
    /* ...and just as important, it must NOT hand back the runaway thrust. On
     * 2026-09-04 this held the saturated collective until the pilot's idle stick
     * rose to meet it, and the craft climbed into the ceiling. Whatever went
     * wrong, hand a pilot roughly hover, never more. */
    check("runaway hand-back never exceeds the hover baseline",
          out <= h.base + 1e-6f);
    check("runaway hand-back is well below saturation",
          out < HEIGHT_THR_MAX - 0.05f);
    (void)held;
  }

  /* 15. On the ground there is nothing to protect — hand back immediately. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    for (int i = 0; i < 50; i++)
      height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                         0.0f, DT, HEIGHT_HOVER_GUESS);
    float out = height_ctrl_update(&h, HEIGHT_MODE_OFF, false, 0.0f, 0.05f,
                                   true, 0.0f, DT, HEIGHT_HOVER_GUESS);
    check("on the ground the stick is returned at once",
          near(out, 0.0f, 1e-6f));
  }

  /* 16. The lift-off baseline follows the MEASURED hover, not the compiled
   *     fallback. Opening a lift-off at a hover 1.7x too high is what put the
   *     airframe into the ceiling, so this is the wiring that matters. */
  {
    height_ctrl_t h;
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, 0.31f);
    check("lift-off opens at the measured hover", near(h.base, 0.31f, 1e-6f));

    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, 0.55f);
    check("a different measurement moves the baseline",
          near(h.base, 0.55f, 1e-6f));

    /* A nonsense reference must fall back, not fly on it. */
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, 0.95f);
    check("out-of-band hover falls back to the constant",
          near(h.base, HEIGHT_HOVER_GUESS, 1e-6f));
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, false, 0.0f, GROUND, true, 0.0f,
                       DT, 0.0f);
    check("zero hover falls back to the constant",
          near(h.base, HEIGHT_HOVER_GUESS, 1e-6f));

    /* An IN-FLIGHT engage still captures the pilot's stick -- the reference is
     * only for lift-off, where there is no stick to learn from. */
    height_ctrl_reset(&h);
    height_ctrl_update(&h, HEIGHT_MODE_HOLD, true, STICK_HOVER, 1.0f, true,
                       0.0f, DT, 0.31f);
    check("in-flight engage still captures the stick, not the reference",
          near(h.base, STICK_HOVER, 1e-6f));
  }

  printf(fails ? "\nFAILED (%d)\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
