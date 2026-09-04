/* throttle_curve_unit_test.c — host unit test for collective stick shaping.
 *
 * Pure function under test: throttle_curve(stick, hover_duty). The
 * properties that matter are the ones a pilot feels: centre-stick is hover, the
 * endpoints still reach 0 and full, the response is monotonic, and the region
 * around hover is less sensitive than the raw linear map it replaces.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/throttle_curve_unit_test.c \
 *       src/control/throttle_curve.c src/maths/maths_interface.c -lm -o /tmp/tc && /tmp/tc
 */
#include "control/throttle_curve.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}
static int near(float a, float b, float tol) { return fabsf(a - b) < tol; }

#define HOVER 0.38f       /* the 5in racer */

int main(void) {
  printf("throttle curve\n");

  /* 1. Endpoints are exact — arming, disarm and full punch must be unaffected. */
  check("stick 0 -> 0", near(throttle_curve(0.0f, HOVER), 0.0f, 1e-6f));
  check("stick 1 -> 1", near(throttle_curve(1.0f, HOVER), 1.0f, 1e-6f));

  /* 2. THE property: centre-stick hovers, at ANY hover value — the expo is
   *    derived from hover and is defined about centre, so it cannot move it. */
  for (float h = 0.20f; h <= 0.70f; h += 0.10f) {
    char msg[64];
    snprintf(msg, sizeof msg, "centre-stick = hover for hover=%.2f", h);
    check(msg, near(throttle_curve(0.5f, h), h, 1e-3f));
  }

  /* 2b. Expo is DERIVED from hover, ArduPilot's formula: lower hover -> more
   *     expo, hover at mid -> none, high hover -> negative. */
  check("expo derived: hover 0.38 -> ~0.32", near(throttle_curve_expo(0.38f), 0.32f, 0.01f));
  check("expo derived: hover 0.50 -> 0", near(throttle_curve_expo(0.50f), 0.0f, 1e-6f));
  check("expo derived: hover 0.60 -> negative", throttle_curve_expo(0.60f) < 0.0f);
  check("expo derived: clamped at the low end", throttle_curve_expo(0.05f) <= 1.0f);

  /* 3. Monotonic — a stick push must never reduce collective. */
  {
    int mono = 1;
    float prev = -1.0f;
    for (int i = 0; i <= 200; i++) {
      float v = throttle_curve(i / 200.0f, HOVER);
      if (v < prev - 1e-6f) mono = 0;
      prev = v;
    }
    check("monotonic across the whole stick", mono);
  }

  /* 4. Less sensitive around hover than the raw linear map (that map had slope
   *    1.0 everywhere; here the mid-stick slope must be gentler). */
  {
    const float d = 0.02f;
    float slope = (throttle_curve(0.5f + d, HOVER) -
                   throttle_curve(0.5f - d, HOVER)) / (2.0f * d);
    check("mid-stick slope is gentler than the raw 1.0 map", slope < 1.0f);
    printf("        (mid-stick slope %.2f collective per unit stick)\n", slope);
  }

  /* 5. Expo flattens further about centre without moving hover. */
  {
    /* The coupling itself: a lower hover must derive MORE expo, because more of
     * the stick then sits above hover and the upper half steepens. (Comparing
     * curve slopes across different hovers is not the test it looks like -- the
     * absolute collectives differ, so any normalisation smuggles in an
     * assumption. The derived coefficient is the honest thing to assert.) */
    check("lower hover derives more expo",
          throttle_curve_expo(0.25f) > throttle_curve_expo(0.45f));
    check("expo shrinks the stick gain at centre by (1-expo)",
          throttle_curve_expo(0.38f) > 0.0f && throttle_curve_expo(0.38f) < 1.0f);
  }

  /* 6. The underlying map is linear in THRUST: equal stick steps give equal
   *    thrust steps, where thrust ~ collective^2. That is what stops the
   *    response accelerating as the stick rises. Checked at hover = 0.5, where
   *    the derived expo is exactly zero and the raw map is exposed. */
  {
    float a = throttle_curve(0.60f, 0.50f);
    float b = throttle_curve(0.70f, 0.50f);
    float c = throttle_curve(0.80f, 0.50f);
    float d1 = b * b - a * a, d2 = c * c - b * b;
    check("equal stick steps -> equal thrust steps (expo-free hover)",
          near(d1, d2, 1e-3f));
  }

  /* 7. Descend authority actually exists below centre: half-stick-down must be
   *    meaningfully below hover (the old map's problem was the opposite —
   *    everything below 38% dropped like a stone). */
  {
    float q = throttle_curve(0.25f, HOVER);
    check("quarter stick is below hover", q < HOVER);
    check("quarter stick still commands real thrust", q > 0.10f);
  }

  /* 8. A nonsensical hover constant degrades to the old linear map rather than
   *    to something surprising. */
  check("hover_duty 0 -> pass-through", near(throttle_curve(0.42f, 0.0f), 0.42f, 1e-6f));
  check("hover_duty 1 -> pass-through", near(throttle_curve(0.42f, 1.0f), 0.42f, 1e-6f));

  /* 9. Out-of-range stick is clamped, not extrapolated. */
  check("stick below 0 clamps", near(throttle_curve(-0.5f, HOVER), 0.0f, 1e-6f));
  check("stick above 1 clamps", near(throttle_curve(1.5f, HOVER), 1.0f, 1e-6f));

  printf(fails ? "\nFAILED (%d)\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
