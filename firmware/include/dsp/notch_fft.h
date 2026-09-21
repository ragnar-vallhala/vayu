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
#ifndef VAYU_DSP_NOTCH_FFT_H
#define VAYU_DSP_NOTCH_FFT_H

/* notch_fft.h — FFT analysis front-end for the dynamic gyro notch.
 *
 * The frequency *source* for the notch: it watches the live gyro spectrum and
 * reports the strongest prop-band peaks, whose center frequencies drive the
 * biquad notch bank (maths/maths_interface.h m_biquad_*). This craft has analog
 * ESCs with no RPM telemetry, so the FFT is the only honest source of where the
 * vibration actually is (design: firmware/docs/journal/log-analysis/
 * 20260625-233852-pitch-indi-campaign/reference-autopilots-comparison.md
 * §10.5-10.6).
 *
 * Same discipline as the maths FFT it consumes: portable float, no CMSIS, no
 * HAL, and NO static/heap state of its own (.bss stays flat — the F401 boot
 * invariant). Every working buffer is caller-owned: the firmware wires
 * v_malloc'd heap in at init (§10.0), the host test wires stack arrays, and for
 * the default N=128 the window+twiddle tables are the const-in-flash pair in
 * maths/fft_tables_N128.h (zero heap, zero init). One instance per analyzed
 * channel (e.g. per gyro axis); the caller runs one axis per cycle / in a
 * background task, never on the rate-loop critical path.
 */
#include "maths/maths_interface.h" /* fft_complex_t, m_rfft_forward */

/* A detected spectral peak: sub-bin-interpolated center frequency + bin power. */
typedef struct {
  float freq_hz;
  float power;
} notch_peak_t;

/* Static configuration for one analyzer channel. */
typedef struct {
  unsigned n;           /* FFT length, power of two >= 4 (e.g. 128) */
  float fs_hz;          /* sample rate the analyzer runs at (post-decimation) */
  float fmin_hz;        /* prop-band lower bound: peaks below are ignored */
  float fmax_hz;        /* prop-band upper bound: peaks above are ignored */
  float min_peak_ratio; /* a peak's power must exceed this * mean band power */
} notch_fft_cfg_t;

/* One analyzer channel. All pointers are CALLER-OWNED (see the header note).
 * `window` (n Hann coeffs) and `tw` (n/2 forward twiddles, the full-N table)
 * are const inputs; `ring`/`frame`/`bins`/`scratch` are working buffers the
 * analyzer writes. Treat the struct as opaque after notch_fft_init. */
typedef struct {
  notch_fft_cfg_t cfg;
  const float *window;     /* n     Hann analysis window */
  const fft_complex_t *tw; /* n/2   forward twiddles (full-N table) */
  float *ring;             /* n     circular sample accumulator */
  float *frame;            /* n     windowed frame handed to the rfft */
  fft_complex_t *bins;     /* n/2+1 rfft output (reused as a power scratch) */
  fft_complex_t *scratch;  /* n/2   rfft scratch */
  unsigned write;          /* ring write cursor */
  unsigned filled;         /* samples accumulated so far (saturates at n) */
  unsigned since_hop;      /* samples since the last frame went ready */
} notch_fft_t;

/* Wire up an analyzer over caller-owned buffers. `cfg->n` must be a power of two
 * >= 4; `window` has n entries, `tw` has n/2, `ring`/`frame` have n, `bins` has
 * n/2+1, `scratch` has n/2. Zeroes the accumulator; no allocation, no libm. */
void notch_fft_init(notch_fft_t *nf, const notch_fft_cfg_t *cfg,
                    const float *window, const fft_complex_t *tw, float *ring,
                    float *frame, fft_complex_t *bins, fft_complex_t *scratch);

/* Push one (decimated) sample onto the channel. Returns 1 when a hop boundary
 * (hop = n/2, i.e. 50% overlap) is crossed AND a full window has accumulated —
 * the caller should then run notch_fft_analyze; returns 0 otherwise. */
int notch_fft_push(notch_fft_t *nf, float sample);

/* Window -> rfft -> greedy peak-pick with parabolic sub-bin interpolation over
 * the [fmin,fmax] band. Writes up to `max_out` peaks (strongest first) into
 * `peaks` and returns the count. Non-destructive to the sample window, so
 * overlapping frames keep working. Safe to call before the first ready frame
 * (returns 0). */
unsigned notch_fft_analyze(notch_fft_t *nf, notch_peak_t *peaks,
                           unsigned max_out);

#endif /* VAYU_DSP_NOTCH_FFT_H */
