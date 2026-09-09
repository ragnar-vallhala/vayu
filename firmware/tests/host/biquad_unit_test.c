/* biquad_unit_test.c — host unit test for src/maths/biquad.c.
 *
 * The notch (band-stop) biquad: a tone at the center frequency is killed, tones
 * elsewhere pass at unity gain, DC passes, bad params fail safe to a bypass, and
 * the recurrence stays bounded. Pure float math (no HAL), so it runs on host.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/biquad_unit_test.c src/maths/biquad.c \
 *       -lm -o /tmp/bq && /tmp/bq
 */
#include "maths/maths_interface.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}

/* Drive a unit-amplitude sine of `f_hz` through the section and return the
 * steady-state peak output amplitude (warm up past the transient, then measure
 * the peak over a stretch of full cycles). */
static float tone_gain(const biquad_coeffs_t *c, float f_hz, float fs_hz) {
  biquad_state_t st;
  m_biquad_reset(&st);
  const float w = 2.0f * PI * f_hz / fs_hz;
  const int warmup = 4000, measure = 4000;
  float peak = 0.0f;
  for (int n = 0; n < warmup + measure; n++) {
    float x = sinf(w * (float)n);
    float y = m_biquad_step(c, &st, x);
    if (n >= warmup && fabsf(y) > peak)
      peak = fabsf(y);
  }
  return peak; /* input peak is 1.0, so this is the gain */
}

int main(void) {
  const float fs = 1000.0f;

  /* 1. Bypass is the identity. */
  printf("Test 1: bypass passes the signal unchanged\n");
  {
    biquad_coeffs_t c;
    m_biquad_bypass(&c);
    biquad_state_t st;
    m_biquad_reset(&st);
    int ok = 1;
    for (int n = 0; n < 100; n++) {
      float x = 0.3f * (float)n - 7.0f;
      if (m_biquad_step(&c, &st, x) != x)
        ok = 0;
    }
    check("y == x for a bypass section", ok);
  }

  /* 2. A notch at f0 kills a tone at f0 but passes a far-off tone. */
  printf("Test 2: notch attenuates the center tone, passes the rest\n");
  {
    biquad_coeffs_t c;
    m_biquad_notch_design(&c, 100.0f, 8.0f, fs);
    float g_center = tone_gain(&c, 100.0f, fs);
    float g_pass = tone_gain(&c, 250.0f, fs);
    printf("    gain @100Hz=%.4f  @250Hz=%.4f\n", g_center, g_pass);
    check("center tone deeply attenuated (< -20 dB)", g_center < 0.1f);
    check("passband tone within +/-10% of unity",
          g_pass > 0.9f && g_pass < 1.1f);
  }

  /* 3. DC passes at unity (the notch only removes its band). */
  printf("Test 3: DC gain is unity\n");
  {
    biquad_coeffs_t c;
    m_biquad_notch_design(&c, 80.0f, 5.0f, fs);
    biquad_state_t st;
    m_biquad_reset(&st);
    float y = 0.0f;
    for (int n = 0; n < 2000; n++)
      y = m_biquad_step(&c, &st, 1.0f);
    check("constant input settles to itself", fabsf(y - 1.0f) < 1e-3f);
  }

  /* 4. A higher Q gives a narrower notch: a tone one bin off the center leaks
   *    through more with high Q than with low Q. */
  printf("Test 4: higher Q is a narrower notch\n");
  {
    biquad_coeffs_t narrow, wide;
    m_biquad_notch_design(&narrow, 100.0f, 20.0f, fs);
    m_biquad_notch_design(&wide, 100.0f, 2.0f, fs);
    float g_narrow = tone_gain(&narrow, 120.0f, fs);
    float g_wide = tone_gain(&wide, 120.0f, fs);
    printf("    @120Hz  Q=20: %.4f   Q=2: %.4f\n", g_narrow, g_wide);
    check("narrow notch leaks the off-center tone more", g_narrow > g_wide);
  }

  /* 5. Bad params fail safe to a bypass (no unstable coeffs from a garbage
   *    peak estimate). */
  printf("Test 5: out-of-range design falls back to bypass\n");
  {
    biquad_coeffs_t c;
    biquad_coeffs_t id;
    m_biquad_bypass(&id);
    int same;

    m_biquad_notch_design(&c, 600.0f, 8.0f, fs); /* above Nyquist (500) */
    same = c.b0 == id.b0 && c.b1 == id.b1 && c.b2 == id.b2 && c.a1 == id.a1 &&
           c.a2 == id.a2;
    check("f0 above Nyquist -> bypass", same);

    m_biquad_notch_design(&c, 100.0f, 0.0f, fs); /* Q == 0 */
    same = c.b0 == id.b0 && c.b1 == id.b1 && c.b2 == id.b2 && c.a1 == id.a1 &&
           c.a2 == id.a2;
    check("Q == 0 -> bypass", same);

    m_biquad_notch_design(&c, 100.0f, 8.0f, 0.0f); /* fs == 0 */
    same = c.b0 == id.b0 && c.b1 == id.b1 && c.b2 == id.b2 && c.a1 == id.a1 &&
           c.a2 == id.a2;
    check("fs == 0 -> bypass", same);
  }

  /* 6. The section is stable: bounded, finite output for a broadband input. */
  printf("Test 6: stable (bounded, finite) on a broadband input\n");
  {
    biquad_coeffs_t c;
    m_biquad_notch_design(&c, 150.0f, 10.0f, fs);
    biquad_state_t st;
    m_biquad_reset(&st);
    int finite = 1;
    float peak = 0.0f;
    /* sum of three tones + a unit impulse, all within |x| <= ~3 */
    for (int n = 0; n < 20000; n++) {
      float t = (float)n / fs;
      float x = sinf(2.0f * PI * 30.0f * t) + sinf(2.0f * PI * 150.0f * t) +
                sinf(2.0f * PI * 320.0f * t) + (n == 0 ? 1.0f : 0.0f);
      float y = m_biquad_step(&c, &st, x);
      if (!isfinite(y))
        finite = 0;
      if (fabsf(y) > peak)
        peak = fabsf(y);
    }
    check("output stays finite", finite);
    check("output stays bounded", peak < 10.0f);
  }

  /* 7. reset clears the delay state. */
  printf("Test 7: reset zeroes the state\n");
  {
    biquad_coeffs_t c;
    m_biquad_notch_design(&c, 100.0f, 8.0f, fs);
    biquad_state_t st;
    m_biquad_reset(&st);
    for (int n = 0; n < 50; n++)
      m_biquad_step(&c, &st, 1.0f);
    m_biquad_reset(&st);
    check("state is zero after reset", st.s1 == 0.0f && st.s2 == 0.0f);
    /* first sample after reset == b0 * x (no history) */
    float y0 = m_biquad_step(&c, &st, 2.0f);
    check("first post-reset sample == b0*x", fabsf(y0 - c.b0 * 2.0f) < 1e-6f);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
