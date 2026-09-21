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
                     float *frame, fft_complex_t *bins,
                     fft_complex_t *scratch) {
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
  nb->hold_on_miss = 0;
  for (unsigned i = 0; i < NOTCH_BANK_MAX_NOTCHES; i++) {
    m_biquad_bypass(&nb->coeffs[i]);
    m_biquad_reset(&nb->state[i]);
    nb->freqs[i] = 0.0f;
  }
}

void notch_bank_set_hold(notch_bank_t *nb, int hold_on_miss) {
  nb->hold_on_miss = hold_on_miss;
}

void notch_bank_set_detection(notch_bank_t *nb, float q, float fmin_hz,
                              float fmax_hz, float min_ratio) {
  if (q > 0.0f) {
    nb->q = q;
  }
  if (fmin_hz > 0.0f) {
    nb->fft.cfg.fmin_hz = fmin_hz;
  }
  if (fmax_hz > 0.0f) {
    nb->fft.cfg.fmax_hz = fmax_hz;
  }
  if (min_ratio > 0.0f) {
    nb->fft.cfg.min_peak_ratio = min_ratio;
  }
}

void notch_bank_reset(notch_bank_t *nb) {
  for (unsigned i = 0; i < NOTCH_BANK_MAX_NOTCHES; i++) {
    m_biquad_bypass(&nb->coeffs[i]);
    m_biquad_reset(&nb->state[i]);
    nb->freqs[i] = 0.0f;
  }
  nb->active = 0;
}

int notch_bank_observe(notch_bank_t *nb, float sample) {
  return notch_fft_push(&nb->fft, sample);
}

unsigned notch_bank_update(notch_bank_t *nb) {
  notch_peak_t peaks[NOTCH_BANK_MAX_NOTCHES];
  unsigned found = notch_fft_analyze(&nb->fft, peaks, nb->num_notches);

  unsigned active = 0;
  for (unsigned i = 0; i < nb->num_notches; i++) {
    if (i < found) {
      /* Design at the FILTER rate (not the analyzer's fs): the biquad runs on
       * the rate loop. m_biquad_notch_design self-bypasses on an out-of-range
       * f0, so a peak past filter Nyquist degrades gracefully. State is left
       * alone — the coeff swap is transient-free. */
      m_biquad_notch_design(&nb->coeffs[i], peaks[i].freq_hz, nb->q,
                            nb->filter_fs_hz);
      nb->freqs[i] = peaks[i].freq_hz;
      active++;
    } else if (nb->hold_on_miss && nb->freqs[i] != 0.0f) {
      /* No peak this round but hold is on and the slot was tuned before: keep
       * the last coefficients so the notch doesn't flicker off on a brief dip. */
      active++;
    } else {
      /* No peak (and nothing to hold) — pass the signal straight through. */
      m_biquad_bypass(&nb->coeffs[i]);
      nb->freqs[i] = 0.0f;
    }
  }
  nb->active = active;
  return active;
}

float notch_bank_filter(notch_bank_t *nb, float sample) {
  float y = sample;
  for (unsigned i = 0; i < nb->num_notches; i++) {
    y = m_biquad_step(&nb->coeffs[i], &nb->state[i], y);
  }
  return y;
}
