/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* fft_unit_test.c — standalone host test for the maths_interface FFT.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/fft_unit_test.c src/maths/fft.c -lm -o /tmp/fftt \
 *       && /tmp/fftt
 *
 * Proves, against an independent naive O(N^2) DFT as ground truth:
 *   (1) forward complex FFT matches the DFT for random input at several N;
 *   (2) inverse FFT round-trips (ifft(fft(x)) == x) and is 1/n-normalized;
 *   (3) the real-FFT wrapper matches the full complex FFT on its n/2+1 bins,
 *       including the DC and Nyquist real-only bins;
 *   (4) a pure tone lands all its energy in the expected bin (the property the
 *       notch peak-pick relies on);
 *   (5) Parseval: sum|x|^2 == (1/n) sum|X|^2;
 *   (6) the const-in-flash table (fft_tables_N128.h) matches m_fft_make_twiddles
 *       and drives a correct transform — the flash table is a verified drop-in.
 */
#include "maths/maths_interface.h"
#include "maths/fft_tables_N128.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* rand()/srand() build the test vectors. A fixed seed is the requirement --
 * a failing FFT case has to be reproducible -- and no output is a secret. */
// NOLINTBEGIN(cert-msc30-c,cert-msc50-cpp,cert-msc32-c,cert-msc51-cpp)

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}

#define TWO_PI 6.283185307179586

/* Independent ground truth: naive forward DFT, X[k] = sum_n x[n] exp(-i2pi kn/N). */
static void naive_dft(const fft_complex_t *x, fft_complex_t *X, unsigned n) {
  for (unsigned k = 0; k < n; k++) {
    double re = 0.0, im = 0.0;
    for (unsigned t = 0; t < n; t++) {
      double ang = -TWO_PI * (double)k * (double)t / (double)n;
      re += x[t].re * cos(ang) - x[t].im * sin(ang);
      im += x[t].re * sin(ang) + x[t].im * cos(ang);
    }
    X[k].re = (float)re;
    X[k].im = (float)im;
  }
}

static int cnear(fft_complex_t a, fft_complex_t b, float tol) {
  return fabsf(a.re - b.re) < tol && fabsf(a.im - b.im) < tol;
}

/* relative tolerance scaled by n (FFT round-off grows ~sqrt(n), be generous). */
static float tol_for(unsigned n) { return 1e-3f * (float)n; }

static void test_forward_vs_dft(unsigned n) {
  fft_complex_t *x = malloc(n * sizeof *x);
  fft_complex_t *ref = malloc(n * sizeof *ref);
  fft_complex_t *got = malloc(n * sizeof *got);
  fft_complex_t *tw = malloc((n / 2) * sizeof *tw);
  m_fft_make_twiddles(tw, n);

  for (unsigned i = 0; i < n; i++) {
    x[i].re = (float)(rand() % 2000 - 1000) / 1000.0f;
    x[i].im = (float)(rand() % 2000 - 1000) / 1000.0f;
    got[i] = x[i];
  }
  naive_dft(x, ref, n);
  m_fft_forward(got, n, tw, 1);

  int ok = 1;
  float tol = tol_for(n);
  for (unsigned k = 0; k < n; k++) {
    ok &= cnear(got[k], ref[k], tol);
  }
  char msg[64];
  snprintf(msg, sizeof msg, "forward FFT == naive DFT (N=%u)", n);
  check(msg, ok);

  free(x);
  free(ref);
  free(got);
  free(tw);
}

static void test_roundtrip(unsigned n) {
  fft_complex_t *x = malloc(n * sizeof *x);
  fft_complex_t *y = malloc(n * sizeof *y);
  fft_complex_t *tw = malloc((n / 2) * sizeof *tw);
  m_fft_make_twiddles(tw, n);

  for (unsigned i = 0; i < n; i++) {
    x[i].re = (float)(rand() % 2000 - 1000) / 1000.0f;
    x[i].im = (float)(rand() % 2000 - 1000) / 1000.0f;
    y[i] = x[i];
  }
  m_fft_forward(y, n, tw, 1);
  m_fft_inverse(y, n, tw, 1);

  int ok = 1;
  for (unsigned i = 0; i < n; i++) {
    ok &= cnear(y[i], x[i], 1e-4f);
  }
  char msg[64];
  snprintf(msg, sizeof msg, "ifft(fft(x)) == x, 1/n normalized (N=%u)", n);
  check(msg, ok);

  free(x);
  free(y);
  free(tw);
}

static void test_rfft_vs_complex(unsigned n) {
  unsigned nh = n / 2;
  float *r = malloc(n * sizeof *r);
  fft_complex_t *cx = malloc(n * sizeof *cx);
  fft_complex_t *cref = malloc(n * sizeof *cref);
  fft_complex_t *rout = malloc((nh + 1) * sizeof *rout);
  fft_complex_t *scratch = malloc(nh * sizeof *scratch);
  fft_complex_t *tw = malloc((n / 2) * sizeof *tw);
  m_fft_make_twiddles(tw, n);

  for (unsigned i = 0; i < n; i++) {
    r[i] = (float)(rand() % 2000 - 1000) / 1000.0f;
    cx[i].re = r[i];
    cx[i].im = 0.0f;
  }
  naive_dft(cx, cref, n);
  m_rfft_forward(r, n, rout, scratch, tw);

  int ok = 1;
  float tol = tol_for(n);
  for (unsigned k = 0; k <= nh; k++) {
    ok &= cnear(rout[k], cref[k], tol);
  }
  /* DC and Nyquist must be exactly real. */
  ok &= fabsf(rout[0].im) < 1e-5f;
  ok &= fabsf(rout[nh].im) < 1e-5f;
  char msg[80];
  snprintf(msg, sizeof msg,
           "rfft == complex DFT on n/2+1 bins, real DC/Nyq (N=%u)", n);
  check(msg, ok);

  free(r);
  free(cx);
  free(cref);
  free(rout);
  free(scratch);
  free(tw);
}

static void test_tone_lands_in_bin(void) {
  /* A real cosine at bin k0 must put (essentially) all power in bin k0. This is
   * the property the notch peak-pick depends on. */
  const unsigned n = 128;
  const unsigned k0 = 17;
  unsigned nh = n / 2;
  float *r = malloc(n * sizeof *r);
  fft_complex_t *rout = malloc((nh + 1) * sizeof *rout);
  fft_complex_t *scratch = malloc(nh * sizeof *scratch);
  fft_complex_t *tw = malloc((n / 2) * sizeof *tw);
  m_fft_make_twiddles(tw, n);

  for (unsigned i = 0; i < n; i++) {
    r[i] = cosf((float)(TWO_PI * (double)k0 * (double)i / (double)n));
  }
  m_rfft_forward(r, n, rout, scratch, tw);

  /* Find the peak power bin. */
  unsigned peak = 0;
  float peak_pow = -1.0f;
  float total_pow = 0.0f;
  for (unsigned k = 0; k <= nh; k++) {
    float p = m_fft_bin_power(rout[k]);
    total_pow += p;
    if (p > peak_pow) {
      peak_pow = p;
      peak = k;
    }
  }
  int ok = (peak == k0) && (peak_pow > 0.99f * total_pow);
  check("pure tone -> all power in its bin (peak-pick precondition)", ok);

  free(r);
  free(rout);
  free(scratch);
  free(tw);
}

static void test_parseval(unsigned n) {
  fft_complex_t *x = malloc(n * sizeof *x);
  fft_complex_t *X = malloc(n * sizeof *X);
  fft_complex_t *tw = malloc((n / 2) * sizeof *tw);
  m_fft_make_twiddles(tw, n);

  double time_energy = 0.0;
  for (unsigned i = 0; i < n; i++) {
    x[i].re = (float)(rand() % 2000 - 1000) / 1000.0f;
    x[i].im = (float)(rand() % 2000 - 1000) / 1000.0f;
    time_energy += (double)x[i].re * x[i].re + (double)x[i].im * x[i].im;
    X[i] = x[i];
  }
  m_fft_forward(X, n, tw, 1);
  double freq_energy = 0.0;
  for (unsigned k = 0; k < n; k++) {
    freq_energy += (double)m_fft_bin_power(X[k]);
  }
  freq_energy /= (double)n;

  int ok = fabs(time_energy - freq_energy) < 1e-2 * time_energy;
  char msg[64];
  snprintf(msg, sizeof msg, "Parseval: sum|x|^2 == (1/n)sum|X|^2 (N=%u)", n);
  check(msg, ok);

  free(x);
  free(X);
  free(tw);
}

static void test_const_table(void) {
  /* The committed flash table must match the runtime twiddles and drive a
   * correct transform — so the FC can read it from flash with zero init. */
  fft_complex_t gen[FFT_TABLE_N / 2];
  m_fft_make_twiddles(gen, FFT_TABLE_N);
  float maxerr = 0.0f;
  for (unsigned k = 0; k < FFT_TABLE_N / 2; k++) {
    float e1 = fabsf(gen[k].re - FFT_TWIDDLE_N128[k].re);
    float e2 = fabsf(gen[k].im - FFT_TWIDDLE_N128[k].im);
    if (e1 > maxerr)
      maxerr = e1;
    if (e2 > maxerr)
      maxerr = e2;
  }
  check("const flash twiddles == runtime twiddles (<1e-6)", maxerr < 1e-6f);

  /* rfft of a tone using the CONST table localizes to the right bin. */
  const unsigned n = FFT_TABLE_N, k0 = 23, nh = n / 2;
  float r[FFT_TABLE_N];
  fft_complex_t out[FFT_TABLE_N / 2 + 1], sc[FFT_TABLE_N / 2];
  for (unsigned i = 0; i < n; i++) {
    r[i] = cosf((float)(TWO_PI * (double)k0 * (double)i / (double)n));
  }
  m_rfft_forward(r, n, out, sc, FFT_TWIDDLE_N128);
  unsigned peak = 0;
  float pp = -1.0f;
  for (unsigned k = 0; k <= nh; k++) {
    float p = m_fft_bin_power(out[k]);
    if (p > pp) {
      pp = p;
      peak = k;
    }
  }
  check("const-table rfft localizes a tone (bin 23)", peak == k0);
}

int main(void) {
  srand(12345);

  printf("Test 1: forward FFT vs naive DFT\n");
  test_forward_vs_dft(2);
  test_forward_vs_dft(8);
  test_forward_vs_dft(64);
  test_forward_vs_dft(128);
  test_forward_vs_dft(256);

  printf("Test 2: inverse round-trip\n");
  test_roundtrip(8);
  test_roundtrip(128);
  test_roundtrip(256);

  printf("Test 3: real-FFT wrapper vs complex DFT\n");
  test_rfft_vs_complex(8);
  test_rfft_vs_complex(64);
  test_rfft_vs_complex(128);
  test_rfft_vs_complex(256);

  printf("Test 4: tone localization\n");
  test_tone_lands_in_bin();

  printf("Test 5: Parseval energy conservation\n");
  test_parseval(64);
  test_parseval(128);

  printf("Test 6: const-in-flash table\n");
  test_const_table();

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}

// NOLINTEND(cert-msc30-c,cert-msc50-cpp,cert-msc32-c,cert-msc51-cpp)