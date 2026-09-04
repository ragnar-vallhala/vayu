/* First-order low-pass filter (firmware/src/est/lpf.c) — pure math, no RTOS.
 *
 * Small but not pointless: the accel/gyro chains run every sample through
 * lpf_apply(), and the D-term LPF time constant is a live tuning knob, so the
 * recurrence and its alpha end-cases are worth pinning down. */
#include <math.h>
#include <stdio.h>

#include "est/est.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

#define CLOSE(a, b, tol) (fabsf((a) - (b)) < (tol))

int main(void) {
  printf("test_lpf: first-order low-pass filter\n");

  printf("  [1] init clears state and stores alpha\n");
  {
    lpf_t f;
    lpf_init(&f, 0.25f);
    CHECK(CLOSE(f.alpha, 0.25f, 1e-6f), "alpha stored");
    CHECK(CLOSE(f.output, 0.0f, 1e-6f), "output starts at zero");
  }

  printf("  [2] alpha = 1 is a pass-through\n");
  {
    lpf_t f;
    lpf_init(&f, 1.0f);
    CHECK(CLOSE(lpf_apply(&f, 3.5f), 3.5f, 1e-6f), "first sample passes through");
    CHECK(CLOSE(lpf_apply(&f, -2.0f), -2.0f, 1e-6f), "second sample passes through");
  }

  printf("  [3] alpha = 0 freezes the output\n");
  {
    lpf_t f;
    lpf_init(&f, 0.0f);
    lpf_apply(&f, 100.0f);
    CHECK(CLOSE(lpf_apply(&f, 100.0f), 0.0f, 1e-6f), "input never reaches output");
  }

  printf("  [4] one step matches the recurrence exactly\n");
  {
    lpf_t f;
    lpf_init(&f, 0.5f);
    /* y1 = 0.5*10 + 0.5*0 = 5; y2 = 0.5*10 + 0.5*5 = 7.5 */
    CHECK(CLOSE(lpf_apply(&f, 10.0f), 5.0f, 1e-6f), "y1 = 5");
    CHECK(CLOSE(lpf_apply(&f, 10.0f), 7.5f, 1e-6f), "y2 = 7.5");
    CHECK(CLOSE(f.output, 7.5f, 1e-6f), "state tracks the return value");
  }

  printf("  [5] converges to a constant input, monotonically, without overshoot\n");
  {
    lpf_t f;
    lpf_init(&f, 0.1f);
    float prev = f.output;
    for (int i = 0; i < 400; i++) {
      float y = lpf_apply(&f, 1.0f);
      if (y < prev - 1e-7f || y > 1.0f + 1e-6f) {
        CHECK(0, "step response is monotonic and never overshoots");
        break;
      }
      prev = y;
    }
    CHECK(CLOSE(f.output, 1.0f, 1e-3f), "settles on the input value");
  }

  printf("  [6] a smaller alpha filters harder\n");
  {
    lpf_t slow, fast;
    lpf_init(&slow, 0.05f);
    lpf_init(&fast, 0.60f);
    for (int i = 0; i < 5; i++) {
      lpf_apply(&slow, 1.0f);
      lpf_apply(&fast, 1.0f);
    }
    CHECK(slow.output < fast.output, "low alpha lags the faster filter");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
