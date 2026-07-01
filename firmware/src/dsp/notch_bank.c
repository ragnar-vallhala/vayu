/* notch_bank.c — dynamic notch bank for one gyro channel.
 *
 * Composes the notch_fft analyzer (frequency source) with a cascade of RBJ
 * band-stop biquads (the filter). notch_bank_observe feeds the analyzer;
 * notch_bank_update retunes the cascade to the strongest peaks (bump-free —
 * coefficients change, state does not); notch_bank_filter runs the cascade on
 * the gyro hot path. See dsp/notch_bank.h for the contract and the
 * caller-owned-buffers / no-static-state constraints. Pure float — the only
 * libm touch is inside m_biquad_notch_design, off the per-sample path. */
#include "dsp/notch_bank.h"

void notch_bank_init(notch_bank_t *nb, const notch_fft_cfg_t *fft_cfg,
                     float filter_fs_hz, unsigned num_notches, float q,
                     const float *window, const fft_complex_t *tw, float *ring,
                     float *frame, fft_complex_t *bins, fft_complex_t *scratch) {
  notch_fft_init(&nb->fft, fft_cfg, window, tw, ring, frame, bins, scratch);
  nb->filter_fs_hz = filter_fs_hz;
  nb->q = q;
  if (num_notches < 1u) {
    num_notches = 1u;
  }
  if (num_notches > NOTCH_BANK_MAX_NOTCHES) {
    num_notches = NOTCH_BANK_MAX_NOTCHES;
  }
  nb->num_notches = num_notches;
  nb->active = 0;
  for (unsigned i = 0; i < NOTCH_BANK_MAX_NOTCHES; i++) {
    m_biquad_bypass(&nb->coeffs[i]);
    m_biquad_reset(&nb->state[i]);
    nb->freqs[i] = 0.0f;
  }
}

int notch_bank_observe(notch_bank_t *nb, float sample) {
  return notch_fft_push(&nb->fft, sample);
}

unsigned notch_bank_update(notch_bank_t *nb) {
  notch_peak_t peaks[NOTCH_BANK_MAX_NOTCHES];
  unsigned found = notch_fft_analyze(&nb->fft, peaks, nb->num_notches);

  for (unsigned i = 0; i < nb->num_notches; i++) {
    if (i < found) {
      /* Design at the FILTER rate (not the analyzer's fs): the biquad runs on
       * the rate loop. m_biquad_notch_design self-bypasses on an out-of-range
       * f0, so a peak past filter Nyquist degrades gracefully. State is left
       * alone — the coeff swap is transient-free. */
      m_biquad_notch_design(&nb->coeffs[i], peaks[i].freq_hz, nb->q,
                            nb->filter_fs_hz);
      nb->freqs[i] = peaks[i].freq_hz;
    } else {
      /* No peak for this slot this round — pass the signal straight through. */
      m_biquad_bypass(&nb->coeffs[i]);
      nb->freqs[i] = 0.0f;
    }
  }
  nb->active = found < nb->num_notches ? found : nb->num_notches;
  return nb->active;
}

float notch_bank_filter(notch_bank_t *nb, float sample) {
  float y = sample;
  for (unsigned i = 0; i < nb->num_notches; i++) {
    y = m_biquad_step(&nb->coeffs[i], &nb->state[i], y);
  }
  return y;
}
