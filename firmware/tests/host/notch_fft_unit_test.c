/* notch_fft_unit_test.c — host unit test for src/dsp/notch_fft.c.
 *
 * The FFT analysis front-end: it must recover a synthetic tone's frequency to
 * sub-bin accuracy, find two peaks when two tones are present, stay silent on a
 * flat spectrum, ignore energy outside the prop band, and cross the hop/ready
 * boundary at the right cadence. Driven with the const N=128 flash tables, so
 * this also exercises the zero-heap default path. Pure float (no HAL) -> host.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/notch_fft_unit_test.c \
 *       src/dsp/notch_fft.c src/maths/fft.c -lm -o /tmp/nf && /tmp/nf
 */
#include "dsp/notch_fft.h"
#include "maths/fft_tables_N128.h"

#include <math.h>
#include <stdio.h>

#define N 128u
#define FS 1000.0f

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}

/* Working buffers (caller-owned, as the module requires). */
static float g_ring[N];
static float g_frame[N];
static fft_complex_t g_bins[N / 2 + 1];
static fft_complex_t g_scratch[N / 2];

static void setup(notch_fft_t *nf, float fmin, float fmax, float ratio) {
  notch_fft_cfg_t cfg = {.n = N,
                         .fs_hz = FS,
                         .fmin_hz = fmin,
                         .fmax_hz = fmax,
                         .min_peak_ratio = ratio};
  notch_fft_init(nf, &cfg, FFT_HANN_N128, FFT_TWIDDLE_N128, g_ring, g_frame,
                 g_bins, g_scratch);
}

/* Feed `count` samples of the given tone sum; run analyze at the first ready
 * frame and return the peak count (peaks written to `out`). tones: array of
 * (freq, amp) pairs, `ntones` of them. */
typedef struct {
  float f, a;
} tone_t;

static unsigned drive(notch_fft_t *nf, const tone_t *tones, unsigned ntones,
                      notch_peak_t *out, unsigned max_out) {
  unsigned got = 0;
  for (unsigned k = 0; k < 4u * N; k++) {
    float x = 0.0f;
    for (unsigned t = 0; t < ntones; t++) {
      x += tones[t].a * sinf(2.0f * PI * tones[t].f * (float)k / FS);
    }
    if (notch_fft_push(nf, x)) {
      got = notch_fft_analyze(nf, out, max_out);
    }
  }
  return got;
}

int main(void) {
  notch_peak_t peaks[4];
  notch_fft_t nf;

  /* 1. Push/ready cadence: no ready before N samples, then every N/2. */
  printf("Test 1: hop/ready cadence\n");
  {
    setup(&nf, 50.0f, 300.0f, 4.0f);
    int first = -1, second = -1;
    for (int k = 0; k < (int)(2 * N); k++) {
      if (notch_fft_push(&nf, 0.0f)) {
        if (first < 0) {
          first = k;
        } else if (second < 0) {
          second = k;
        }
      }
    }
    check("first ready at the Nth sample", first == (int)N - 1);
    check("next ready one hop (N/2) later", second - first == (int)(N / 2));
  }

  /* 2. Single tone recovered to sub-bin accuracy. Bin width is 7.8 Hz; a tone
   *    between bins must interpolate to within ~1 Hz. */
  printf("Test 2: single-tone frequency recovery\n");
  {
    setup(&nf, 50.0f, 300.0f, 4.0f);
    tone_t tones[] = {{133.0f, 1.0f}}; /* 133 Hz -> bin 17.02, off-grid */
    unsigned got = drive(&nf, tones, 1, peaks, 4);
    printf("    found=%u  f0=%.2f Hz  (want ~133)\n", got,
           got ? peaks[0].freq_hz : 0.0f);
    check("exactly one peak", got == 1);
    check("frequency within 2 Hz of 133",
          got && fabsf(peaks[0].freq_hz - 133.0f) < 2.0f);
  }

  /* 3. Two well-separated tones -> two peaks, strongest first. */
  printf("Test 3: two tones, ranked by power\n");
  {
    setup(&nf, 50.0f, 400.0f, 3.0f);
    tone_t tones[] = {{90.0f, 0.5f}, {240.0f, 1.0f}}; /* 240 is stronger */
    unsigned got = drive(&nf, tones, 2, peaks, 4);
    printf("    found=%u  p0=%.1f Hz  p1=%.1f Hz\n", got,
           got > 0 ? peaks[0].freq_hz : 0.0f,
           got > 1 ? peaks[1].freq_hz : 0.0f);
    check("two peaks found", got == 2);
    check("strongest peak (240 Hz) reported first",
          got == 2 && fabsf(peaks[0].freq_hz - 240.0f) < 3.0f);
    check("second peak is 90 Hz",
          got == 2 && fabsf(peaks[1].freq_hz - 90.0f) < 3.0f);
  }

  /* 4. Flat / silent spectrum -> nothing above threshold. */
  printf("Test 4: silent spectrum yields no peaks\n");
  {
    setup(&nf, 50.0f, 300.0f, 4.0f);
    unsigned got = drive(&nf, NULL, 0, peaks, 4); /* all-zero input */
    check("no peaks on a zero signal", got == 0);
  }

  /* 5. Band gating: a tone above fmax is ignored. */
  printf("Test 5: out-of-band tone is ignored\n");
  {
    setup(&nf, 50.0f, 200.0f, 4.0f);
    tone_t tones[] = {{380.0f, 1.0f}}; /* above the 200 Hz fmax */
    unsigned got = drive(&nf, tones, 1, peaks, 4);
    check("tone above fmax not reported", got == 0);
  }

  /* 6. max_out caps the reported peaks. */
  printf("Test 6: max_out caps the peak count\n");
  {
    setup(&nf, 50.0f, 400.0f, 2.0f);
    tone_t tones[] = {{90.0f, 1.0f}, {180.0f, 1.0f}, {300.0f, 1.0f}};
    unsigned got = drive(&nf, tones, 3, peaks, 2); /* ask for at most 2 */
    check("no more than max_out peaks", got <= 2);
  }

  /* 7. analyze before the first full window is safe and empty. */
  printf("Test 7: analyze before a full window is empty\n");
  {
    setup(&nf, 50.0f, 300.0f, 4.0f);
    for (int k = 0; k < (int)(N / 4); k++)
      notch_fft_push(&nf, 1.0f);
    check("no peaks before a window fills",
          notch_fft_analyze(&nf, peaks, 4) == 0);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
