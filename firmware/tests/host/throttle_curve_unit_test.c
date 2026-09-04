/* throttle_curve_unit_test.c — host unit test for collective stick shaping.
 *
 * Pure function under test: throttle_curve(stick, hover_duty, expo). The
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
  check("stick 0 -> 0", near(throttle_curve(0.0f, HOVER, 0.3f), 0.0f, 1e-6f));
  check("stick 1 -> 1", near(throttle_curve(1.0f, HOVER, 0.3f), 1.0f, 1e-6f));

  /* 2. THE property: centre-stick hovers. Must hold at any expo, because expo
   *    is defined about centre. */
  for (float e = 0.0f; e <= 0.9f; e += 0.3f) {
    char msg[64];
    snprintf(msg, sizeof msg, "centre-stick = hover at expo %.1f", e);
    check(msg, near(throttle_curve(0.5f, HOVER, e), HOVER, 1e-3f));
  }

  /* 3. Monotonic — a stick push must never reduce collective. */
  {
    int mono = 1;
    float prev = -1.0f;
    for (int i = 0; i <= 200; i++) {
      float v = throttle_curve(i / 200.0f, HOVER, 0.3f);
      if (v < prev - 1e-6f) mono = 0;
      prev = v;
    }
    check("monotonic across the whole stick", mono);
  }

  /* 4. Less sensitive around hover than the raw linear map (that map had slope
   *    1.0 everywhere; here the mid-stick slope must be gentler). */
  {
    const float d = 0.02f;
    float slope = (throttle_curve(0.5f + d, HOVER, 0.3f) -
                   throttle_curve(0.5f - d, HOVER, 0.3f)) / (2.0f * d);
    check("mid-stick slope is gentler than the raw 1.0 map", slope < 1.0f);
    printf("        (mid-stick slope %.2f collective per unit stick)\n", slope);
  }

  /* 5. Expo flattens further about centre without moving hover. */
  {
    const float d = 0.05f;
    float s0 = (throttle_curve(0.5f + d, HOVER, 0.0f) -
                throttle_curve(0.5f - d, HOVER, 0.0f));
    float s9 = (throttle_curve(0.5f + d, HOVER, 0.9f) -
                throttle_curve(0.5f - d, HOVER, 0.9f));
    check("more expo = flatter around hover", s9 < s0);
  }

  /* 6. Linear in THRUST above hover (expo off): equal stick steps give equal
   *    thrust steps, where thrust ~ collective^2. This is the property that
   *    stops the response accelerating as the stick goes up. */
  {
    float a = throttle_curve(0.60f, HOVER, 0.0f);
    float b = throttle_curve(0.70f, HOVER, 0.0f);
    float c = throttle_curve(0.80f, HOVER, 0.0f);
    float d1 = b * b - a * a, d2 = c * c - b * b;
    check("equal stick steps -> equal thrust steps", near(d1, d2, 1e-3f));
  }

  /* 7. Descend authority actually exists below centre: half-stick-down must be
   *    meaningfully below hover (the old map's problem was the opposite —
   *    everything below 38% dropped like a stone). */
  {
    float q = throttle_curve(0.25f, HOVER, 0.3f);
    check("quarter stick is below hover", q < HOVER);
    check("quarter stick still commands real thrust", q > 0.10f);
  }

  /* 8. A nonsensical hover constant degrades to the old linear map rather than
   *    to something surprising. */
  check("hover_duty 0 -> pass-through", near(throttle_curve(0.42f, 0.0f, 0.3f), 0.42f, 1e-6f));
  check("hover_duty 1 -> pass-through", near(throttle_curve(0.42f, 1.0f, 0.3f), 0.42f, 1e-6f));

  /* 9. Out-of-range stick is clamped, not extrapolated. */
  check("stick below 0 clamps", near(throttle_curve(-0.5f, HOVER, 0.3f), 0.0f, 1e-6f));
  check("stick above 1 clamps", near(throttle_curve(1.5f, HOVER, 0.3f), 1.0f, 1e-6f));

  printf(fails ? "\nFAILED (%d)\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
