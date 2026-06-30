/* biquad.c — second-order notch (band-stop) section for the maths_interface
 * module.
 *
 * The evaluator half of the FFT-driven dynamic-notch effort (design:
 * firmware/docs/journal/log-analysis/20260625-233852-pitch-indi-campaign/
 * reference-autopilots-comparison.md §10.5-10.6). See maths/maths_interface.h
 * for the contract and the no-CMSIS / no-static-state / portable-float
 * constraints. libm (sinf/cosf) is used only by the coefficient designer at the
 * gated coeff-update cadence; the per-sample m_biquad_step is plain arithmetic
 * and lives inline in the header. */
#include "maths/maths_interface.h"

#include <math.h>

/* 2*pi as a single-precision constant (matches -fsingle-precision-constant). */
#define BIQUAD_TWO_PI 6.28318530717958647692f

/** @noreq identity passthrough section (y == x). */
void m_biquad_bypass(biquad_coeffs_t *c) {
  c->b0 = 1.0f;
  c->b1 = 0.0f;
  c->b2 = 0.0f;
  c->a1 = 0.0f;
  c->a2 = 0.0f;
}

/** @noreq RBJ band-stop (notch) coefficient designer for the gyro notch. */
void m_biquad_notch_design(biquad_coeffs_t *c, float f0_hz, float q,
                           float fs_hz) {
  /* Fail safe to a bypass on any out-of-range request: the FFT peak-pick can
   * hand us a garbage center frequency, and an unstable section in the gyro
   * path is far worse than no notch. f0 must sit strictly inside the open band
   * (0, Nyquist); q and fs must be positive. */
  if (!(fs_hz > 0.0f) || !(q > 0.0f) || !(f0_hz > 0.0f) ||
      !(f0_hz < 0.5f * fs_hz)) {
    m_biquad_bypass(c);
    return;
  }

  /* RBJ audio-EQ cookbook, band-stop ("notch"):
   *   w0    = 2*pi*f0/fs
   *   alpha = sin(w0) / (2*Q)
   *   b0 =  1        b1 = -2*cos(w0)   b2 =  1
   *   a0 =  1+alpha  a1 = -2*cos(w0)   a2 =  1-alpha
   * Then normalize every coefficient by a0 so a0 == 1 and drops out of the
   * recurrence. Gives exactly unity gain at DC and Nyquist and a true null at
   * f0. */
  float w0 = BIQUAD_TWO_PI * f0_hz / fs_hz;
  float cos_w0 = cosf(w0);
  float alpha = sinf(w0) / (2.0f * q);
  float a0 = 1.0f + alpha;
  float inv_a0 = 1.0f / a0;

  c->b0 = inv_a0;
  c->b1 = -2.0f * cos_w0 * inv_a0;
  c->b2 = inv_a0;
  c->a1 = -2.0f * cos_w0 * inv_a0;
  c->a2 = (1.0f - alpha) * inv_a0;
}
