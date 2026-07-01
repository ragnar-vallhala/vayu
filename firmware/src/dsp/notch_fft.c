/* notch_fft.c — FFT analysis front-end for the dynamic gyro notch.
 *
 * Accumulates decimated gyro samples into an overlapped window, transforms it
 * with the maths-module real FFT, and greedily picks the strongest peaks in the
 * prop band with parabolic sub-bin interpolation. The center frequencies feed
 * the biquad notch bank (maths m_biquad_*). See dsp/notch_fft.h for the contract
 * and the no-CMSIS / no-static-state / caller-owned-buffers constraints; design
 * ref §10.5-10.6. Pure float arithmetic — no libm, no HAL. */
#include "dsp/notch_fft.h"

/* Guard band (in bins) zeroed around a picked peak so its spectral skirt is not
 * re-picked as a second peak. ~2 bins each side. */
#define NOTCH_FFT_PEAK_GUARD 2u

void notch_fft_init(notch_fft_t *nf, const notch_fft_cfg_t *cfg,
                    const float *window, const fft_complex_t *tw, float *ring,
                    float *frame, fft_complex_t *bins, fft_complex_t *scratch) {
  nf->cfg = *cfg;
  nf->window = window;
  nf->tw = tw;
  nf->ring = ring;
  nf->frame = frame;
  nf->bins = bins;
  nf->scratch = scratch;
  nf->write = 0;
  nf->filled = 0;
  nf->since_hop = 0;
  for (unsigned i = 0; i < cfg->n; i++) {
    ring[i] = 0.0f;
  }
}

int notch_fft_push(notch_fft_t *nf, float sample) {
  unsigned n = nf->cfg.n;
  unsigned hop = n >> 1; /* 50% overlap */

  nf->ring[nf->write] = sample;
  nf->write = (nf->write + 1u) % n;
  if (nf->filled < n) {
    nf->filled++;
  }
  nf->since_hop++;

  /* Ready once a full window exists and another hop's worth has arrived. */
  if (nf->filled >= n && nf->since_hop >= hop) {
    nf->since_hop = 0;
    return 1;
  }
  return 0;
}

/* Sub-bin offset (in bins) of a peak from the three power samples straddling it,
 * by parabolic interpolation. Clamped to [-0.5, 0.5]. */
static float parabolic_delta(float y0, float y1, float y2) {
  float denom = y0 - 2.0f * y1 + y2;
  if (denom == 0.0f) {
    return 0.0f;
  }
  float delta = 0.5f * (y0 - y2) / denom;
  if (delta > 0.5f) {
    return 0.5f;
  }
  if (delta < -0.5f) {
    return -0.5f;
  }
  return delta;
}

unsigned notch_fft_analyze(notch_fft_t *nf, notch_peak_t *peaks,
                           unsigned max_out) {
  unsigned n = nf->cfg.n;
  float fs = nf->cfg.fs_hz;

  if (nf->filled < n || max_out == 0) {
    return 0;
  }

  /* Window the ring in time order (oldest sample sits at the write cursor). */
  for (unsigned i = 0; i < n; i++) {
    nf->frame[i] = nf->window[i] * nf->ring[(nf->write + i) % n];
  }
  m_rfft_forward(nf->frame, n, nf->bins, nf->scratch, nf->tw);

  /* Prop-band bin range [a, b], excluding DC and Nyquist. Integer casts give
   * floor / ceil (positive args) without pulling in libm. */
  unsigned nyq = n >> 1;
  unsigned a = (unsigned)(nf->cfg.fmin_hz * (float)n / fs);
  unsigned b = (unsigned)(nf->cfg.fmax_hz * (float)n / fs + 1.0f);
  if (a < 1u) {
    a = 1u;
  }
  if (b > nyq - 1u) {
    b = nyq - 1u;
  }
  if (a > b) {
    return 0;
  }

  /* Threshold against the WHOLE-spectrum mean power, not just the band's: a
   * strong out-of-band tone leaks a rising skirt into the band edge, and a
   * band-local mean would let that skirt masquerade as a peak. Averaging over
   * the full [1, Nyquist) makes the out-of-band peak inflate the floor and swamp
   * its own leakage, so only genuinely dominant in-band lines survive. */
  float sum = 0.0f;
  for (unsigned k = 1; k < nyq; k++) {
    sum += m_fft_bin_power(nf->bins[k]);
  }
  float mean = sum / (float)(nyq - 1u);
  float threshold = nf->cfg.min_peak_ratio * mean;

  /* Reduce the search band to bin power in place (bins[k].re := |bins[k]|^2).
   * The full-spectrum sum above already read every bin, so overwriting re now is
   * safe. */
  for (unsigned k = a; k <= b; k++) {
    nf->bins[k].re = m_fft_bin_power(nf->bins[k]);
  }

  /* Greedy: repeatedly take the strongest remaining bin above threshold,
   * interpolate its frequency, then zero a guard band around it so its skirt is
   * not re-picked. */
  unsigned found = 0;
  while (found < max_out) {
    unsigned kp = a;
    float vp = -1.0f;
    for (unsigned k = a; k <= b; k++) {
      if (nf->bins[k].re > vp) {
        vp = nf->bins[k].re;
        kp = k;
      }
    }
    if (vp <= 0.0f || vp < threshold) {
      break;
    }

    /* Interpolate only with interior neighbours; clamp at the band edges. */
    float delta = 0.0f;
    if (kp > a && kp < b) {
      delta = parabolic_delta(nf->bins[kp - 1].re, nf->bins[kp].re,
                              nf->bins[kp + 1].re);
    }
    float freq = ((float)kp + delta) * fs / (float)n;

    peaks[found].freq_hz = freq;
    peaks[found].power = vp;
    found++;

    unsigned lo = (kp > a + NOTCH_FFT_PEAK_GUARD) ? kp - NOTCH_FFT_PEAK_GUARD : a;
    unsigned hi = (kp + NOTCH_FFT_PEAK_GUARD < b) ? kp + NOTCH_FFT_PEAK_GUARD : b;
    for (unsigned k = lo; k <= hi; k++) {
      nf->bins[k].re = 0.0f;
    }
  }

  return found;
}
