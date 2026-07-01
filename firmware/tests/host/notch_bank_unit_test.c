/* notch_bank_unit_test.c — host unit test for src/dsp/notch_bank.c.
 *
 * The bank ties the FFT analyzer to the biquad cascade: after observing a tone
 * it must retune a notch onto it so that same tone, run through the filter, is
 * strongly attenuated while an out-of-notch tone passes near unity. Also checks
 * the identity/bypass default, num_notches clamping, multi-notch tuning, the
 * silent case, and a distinct filter rate. Const N=128 flash tables -> zero-heap
 * path. Pure float (no HAL) -> host.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/notch_bank_unit_test.c \
 *       src/dsp/notch_bank.c src/dsp/notch_fft.c src/maths/fft.c \
 *       src/maths/biquad.c -lm -o /tmp/nb && /tmp/nb
 */
#include "dsp/notch_bank.h"
#include "maths/fft_tables_N128.h"

#include <math.h>
#include <stdio.h>

#define N 128u
#define FS 1000.0f

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

/* Caller-owned analyzer buffers. */
static float g_ring[N];
static float g_frame[N];
static fft_complex_t g_bins[N / 2 + 1];
static fft_complex_t g_scratch[N / 2];

typedef struct { float f, a; } tone_t;

static void setup(notch_bank_t *nb, float filter_fs, unsigned num_notches,
                  float q, float fmin, float fmax, float ratio) {
  notch_fft_cfg_t cfg = {.n = N,
                         .fs_hz = FS,
                         .fmin_hz = fmin,
                         .fmax_hz = fmax,
                         .min_peak_ratio = ratio};
  notch_bank_init(nb, &cfg, filter_fs, num_notches, q, FFT_HANN_N128,
                  FFT_TWIDDLE_N128, g_ring, g_frame, g_bins, g_scratch);
}

/* Observe `ntones` tones (sampled at FS) until the analyzer converges, retuning
 * on every ready frame. Returns the active notch count after the last update. */
static unsigned tune(notch_bank_t *nb, const tone_t *tones, unsigned ntones) {
  unsigned active = 0;
  for (unsigned k = 0; k < 4u * N; k++) {
    float x = 0.0f;
    for (unsigned t = 0; t < ntones; t++)
      x += tones[t].a * sinf(2.0f * PI * tones[t].f * (float)k / FS);
    if (notch_bank_observe(nb, x)) active = notch_bank_update(nb);
  }
  return active;
}

/* Steady-state gain of the tuned cascade at `freq`, sampled at the bank's own
 * filter rate. Returns the RMS ratio (out/in) over an integer number of settled
 * cycles — RMS, not peak, so it stays accurate even at a few samples/cycle. */
static float filtered_gain(notch_bank_t *nb, float freq, float filter_fs) {
  unsigned per_cycle = (unsigned)(filter_fs / freq);
  if (per_cycle < 1u) per_cycle = 1u;
  unsigned settle = per_cycle * 40u;   /* let the biquads reach steady state */
  unsigned window = per_cycle * 20u;   /* whole cycles -> stable RMS */
  float sx2 = 0.0f, sy2 = 0.0f;
  for (unsigned k = 0; k < settle + window; k++) {
    float x = sinf(2.0f * PI * freq * (float)k / filter_fs);
    float y = notch_bank_filter(nb, x);
    if (k >= settle) {
      sx2 += x * x;
      sy2 += y * y;
    }
  }
  return sx2 > 0.0f ? sqrtf(sy2 / sx2) : 0.0f;
}

int main(void) {
  notch_bank_t nb;

  /* 1. Fresh bank is identity: every biquad bypassed until the first update. */
  printf("Test 1: bypass default is identity\n");
  {
    setup(&nb, FS, 3u, 8.0f, 50.0f, 400.0f, 4.0f);
    float g = filtered_gain(&nb, 200.0f, FS);
    check("passes a tone unchanged before any update", fabsf(g - 1.0f) < 0.02f);
    check("no notches active before update", nb.active == 0);
  }

  /* 2. Tune onto a tone: that tone is killed, a far tone survives. */
  printf("Test 2: tuned notch attenuates its tone, passband survives\n");
  {
    setup(&nb, FS, 3u, 8.0f, 50.0f, 400.0f, 4.0f);
    tone_t tones[] = {{200.0f, 1.0f}};
    unsigned active = tune(&nb, tones, 1);
    float notch_g = filtered_gain(&nb, 200.0f, FS);
    setup(&nb, FS, 3u, 8.0f, 50.0f, 400.0f, 4.0f); /* re-tune for clean passband probe */
    tune(&nb, tones, 1);
    float pass_g = filtered_gain(&nb, 380.0f, FS);
    printf("    active=%u  notch_gain=%.3f  pass_gain=%.3f\n", active, notch_g, pass_g);
    check("one notch active", active == 1);
    check("tone at notch strongly attenuated (< -12 dB)", notch_g < 0.25f);
    check("far passband tone near unity", pass_g > 0.85f);
  }

  /* 3. num_notches is clamped into [1, MAX]. */
  printf("Test 3: num_notches clamping\n");
  {
    setup(&nb, FS, 0u, 8.0f, 50.0f, 400.0f, 4.0f);
    check("zero clamps up to 1", nb.num_notches == 1u);
    setup(&nb, FS, 99u, 8.0f, 50.0f, 400.0f, 4.0f);
    check("oversize clamps to MAX", nb.num_notches == NOTCH_BANK_MAX_NOTCHES);
  }

  /* 4. Two tones -> two notches, both attenuated. */
  printf("Test 4: two tones both notched\n");
  {
    setup(&nb, FS, 3u, 8.0f, 50.0f, 400.0f, 3.0f);
    tone_t tones[] = {{120.0f, 1.0f}, {260.0f, 1.0f}};
    unsigned active = tune(&nb, tones, 2);
    float g120 = filtered_gain(&nb, 120.0f, FS);
    float g260 = filtered_gain(&nb, 260.0f, FS);
    printf("    active=%u  g120=%.3f  g260=%.3f\n", active, g120, g260);
    check("two notches active", active == 2);
    check("120 Hz attenuated", g120 < 0.3f);
    check("260 Hz attenuated", g260 < 0.3f);
  }

  /* 5. Silent spectrum -> nothing tuned, filter stays identity. */
  printf("Test 5: silence tunes nothing\n");
  {
    setup(&nb, FS, 3u, 8.0f, 50.0f, 400.0f, 4.0f);
    unsigned active = tune(&nb, NULL, 0);
    float g = filtered_gain(&nb, 200.0f, FS);
    check("no notches active on silence", active == 0);
    check("filter is identity", fabsf(g - 1.0f) < 0.02f);
  }

  /* 6. Distinct filter rate: biquads designed/run at 2 kHz still notch. */
  printf("Test 6: filter rate != analyzer rate\n");
  {
    setup(&nb, 2000.0f, 3u, 8.0f, 50.0f, 400.0f, 4.0f);
    tone_t tones[] = {{200.0f, 1.0f}}; /* analyzer sees it at FS=1000 */
    unsigned active = tune(&nb, tones, 1);
    float notch_g = filtered_gain(&nb, 200.0f, 2000.0f); /* filter runs at 2 kHz */
    printf("    active=%u  notch_gain@2k=%.3f\n", active, notch_g);
    check("notch active", active == 1);
    check("200 Hz attenuated at the 2 kHz filter rate", notch_g < 0.3f);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
