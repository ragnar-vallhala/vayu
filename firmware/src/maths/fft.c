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
/* fft.c — self-contained radix-2 FFT for the maths_interface module.
 *
 * Part of the FFT-driven dynamic-notch effort (design: firmware/docs/journal/log-analysis/
 * 20260625-233852-pitch-indi-campaign/reference-autopilots-comparison.md
 * §10.5-10.6). See maths/maths_interface.h for the contract and the no-CMSIS /
 * no-static-state / portable-float constraints. libm (sinf/cosf) is used only by
 * m_fft_make_twiddles at init; the transform itself is plain arithmetic. */
#include "maths/maths_interface.h"

#include <math.h>
#include <stddef.h> /* size_t (was reached transitively) */

/* 2*pi as a single-precision constant (matches -fsingle-precision-constant). */
#define FFT_TWO_PI 6.28318530717958647692f

/** @noreq forward-twiddle table generator for the maths-interface FFT. */
void m_fft_make_twiddles(fft_complex_t *tw, unsigned n) {
  unsigned half = n >> 1;
  for (unsigned k = 0; k < half; k++) {
    float ang = -FFT_TWO_PI * (float)k / (float)n;
    tw[k].re = cosf(ang);
    tw[k].im = sinf(ang);
  }
}

/* In-place bit-reversal permutation (the DIT reorder). n is a power of two. */
static void fft_bit_reverse(fft_complex_t *x, unsigned n) {
  unsigned j = 0;
  for (unsigned i = 1; i < n; i++) {
    unsigned bit = n >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      fft_complex_t t = x[i];
      x[i] = x[j];
      x[j] = t;
    }
  }
}

/** @noreq in-place radix-2 DIT complex FFT (maths-interface FFT core). */
void m_fft_forward(fft_complex_t *x, unsigned n, const fft_complex_t *tw,
                   unsigned tw_stride) {
  if (n < 2) {
    return;
  }
  fft_bit_reverse(x, n);

  /* Cooley-Tukey: combine length-`len` sub-DFTs, len = 2,4,...,n. The twiddle
   * for butterfly position j in a length-`len` stage is W_len^j, which equals
   * the table entry tw[j * (n/len) * tw_stride] — the stride lets the same
   * full-N table serve a half-N transform (see maths_interface.h). */
  for (unsigned len = 2; len <= n; len <<= 1) {
    unsigned half = len >> 1;
    unsigned step = (n / len) * tw_stride;
    for (unsigned base = 0; base < n; base += len) {
      for (unsigned j = 0; j < half; j++) {
        fft_complex_t w = tw[(size_t)j * step];
        fft_complex_t *a = &x[base + j];
        fft_complex_t *b = &x[base + j + half];
        /* t = w * b */
        float tr = w.re * b->re - w.im * b->im;
        float ti = w.re * b->im + w.im * b->re;
        b->re = a->re - tr;
        b->im = a->im - ti;
        a->re += tr;
        a->im += ti;
      }
    }
  }
}

/** @noreq 1/n-normalized inverse FFT via the conjugate identity. */
void m_fft_inverse(fft_complex_t *x, unsigned n, const fft_complex_t *tw,
                   unsigned tw_stride) {
  if (n < 1) {
    return;
  }
  /* ifft(x) = (1/n) * conj( fft( conj(x) ) ). */
  for (unsigned i = 0; i < n; i++) {
    x[i].im = -x[i].im;
  }
  m_fft_forward(x, n, tw, tw_stride);
  float inv = 1.0f / (float)n;
  for (unsigned i = 0; i < n; i++) {
    x[i].re = x[i].re * inv;
    x[i].im = -x[i].im * inv;
  }
}

/** @noreq real-input FFT: n reals -> n/2+1 bins, via a half-size complex FFT. */
void m_rfft_forward(const float *in, unsigned n, fft_complex_t *out,
                    fft_complex_t *scratch, const fft_complex_t *tw) {
  if (n < 2) {
    if (n == 1) {
      out[0].re = in[0];
      out[0].im = 0.0f;
    }
    return;
  }
  unsigned nh = n >> 1; /* half length: pack 2 reals per complex sample */

  /* Pack: z[m] = in[2m] + i*in[2m+1], then an nh-point complex FFT. The full-N
   * twiddle table serves the nh-point transform via tw_stride == 2. */
  for (unsigned m = 0; m < nh; m++) {
    scratch[m].re = in[2u * (size_t)m];
    scratch[m].im = in[2 * m + 1];
  }
  m_fft_forward(scratch, nh, tw, 2);

  /* Recombine the nh complex bins into the n/2+1 real-input bins.
   *   Xe[k] = 0.5*(Z[k] + conj(Z[nh-k]))                (even-sample DFT)
   *   Xo[k] = -0.5*i*(Z[k] - conj(Z[nh-k]))             (odd-sample DFT)
   *   X[k]  = Xe[k] + W_n^k * Xo[k],   W_n^k = tw[k]
   * DC and Nyquist are purely real and fall out of Z[0]. */
  out[0].re = scratch[0].re + scratch[0].im;
  out[0].im = 0.0f;
  out[nh].re = scratch[0].re - scratch[0].im;
  out[nh].im = 0.0f;

  for (unsigned k = 1; k < nh; k++) {
    fft_complex_t a = scratch[k];
    fft_complex_t b = scratch[nh - k];

    /* Xe = 0.5*(a + conj(b)),  Xo = -0.5*i*(a - conj(b)). */
    float xe_re = 0.5f * (a.re + b.re);
    float xe_im = 0.5f * (a.im - b.im);
    float xo_re = 0.5f * (a.im + b.im);
    float xo_im = 0.5f * (b.re - a.re);

    fft_complex_t w = tw[k]; /* exp(-i*2*pi*k/n) */
    float wxo_re = w.re * xo_re - w.im * xo_im;
    float wxo_im = w.re * xo_im + w.im * xo_re;

    out[k].re = xe_re + wxo_re;
    out[k].im = xe_im + wxo_im;
  }
}
