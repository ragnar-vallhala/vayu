#ifndef VAYU_DSP_NOTCH_BANK_H
#define VAYU_DSP_NOTCH_BANK_H

/* notch_bank.h — dynamic notch bank for one gyro channel.
 *
 * Glues the two DSP primitives into an actual filter: the notch_fft analyzer
 * (dsp/notch_fft.h) supplies the strongest prop-band peak frequencies, and a
 * short cascade of RBJ band-stop biquads (maths m_biquad_*) removes them from
 * the gyro stream. One bank per analyzed channel (e.g. per gyro axis); the
 * caller runs one axis per cycle / in a background task, never on the rate-loop
 * critical path (design: reference-autopilots-comparison.md §10.5-10.6).
 *
 * Two sample rates, deliberately distinct:
 *   - the analyzer runs at the (decimated) fs_hz in notch_fft_cfg_t;
 *   - the biquads run — and are DESIGNED — at filter_fs_hz, the full rate-loop
 *     rate where notch_bank_filter() is called every sample. Passing the peak
 *     Hz through m_biquad_notch_design(., ., ., filter_fs_hz) keeps the notch
 *     centred correctly regardless of the analysis decimation.
 *
 * Same discipline as the primitives it composes: portable float, no CMSIS, no
 * HAL, no static state. The analyzer's working buffers are caller-owned (wired
 * in at init exactly as notch_fft wants them); the small fixed coeff/state
 * arrays live inside the bank struct (no heap, .bss stays flat). Redesign is
 * bump-free: notch_bank_update rewrites the coefficients but never touches the
 * biquad state (Direct-Form-II-Transposed carries no transient across a coeff
 * change), so the notch can track a moving peak without clicking.
 */
#include "dsp/notch_fft.h" /* notch_fft_t, notch_peak_t, fft_complex_t + maths */

/* Max biquads cascaded per channel. Three covers the fundamental plus two
 * harmonics; the bump is cheap (5 mul + 4 add per section per sample). */
#define NOTCH_BANK_MAX_NOTCHES 4u

/* One dynamic notch bank for a single channel. Treat as opaque after init. */
typedef struct {
  notch_fft_t fft;      /* the analysis front-end (buffers wired at init) */
  float filter_fs_hz;   /* rate at which the biquads run / are designed */
  float q;              /* notch Q (bandwidth); higher = narrower */
  unsigned num_notches; /* biquads in the cascade (1..MAX) */
  unsigned active;      /* notches currently tuned to a peak (0..num_notches) */
  biquad_coeffs_t coeffs[NOTCH_BANK_MAX_NOTCHES];
  biquad_state_t state[NOTCH_BANK_MAX_NOTCHES];
  float freqs
      [NOTCH_BANK_MAX_NOTCHES]; /* last tuned center Hz per slot; 0 if bypassed */
  int hold_on_miss; /* if set, a slot with no peak keeps its last coeffs */
} notch_bank_t;

/* Wire up a bank. `fft_cfg`/`window`/`tw`/`ring`/`frame`/`bins`/`scratch` are
 * the notch_fft analyzer inputs (see notch_fft_init — same ownership rules).
 * `filter_fs_hz` is the rate notch_bank_filter runs at; `num_notches` is clamped
 * to [1, NOTCH_BANK_MAX_NOTCHES]; `q` is the band-stop Q. All biquads start
 * bypassed and their state cleared. No allocation, no libm. */
void notch_bank_init(notch_bank_t *nb, const notch_fft_cfg_t *fft_cfg,
                     float filter_fs_hz, unsigned num_notches, float q,
                     const float *window, const fft_complex_t *tw, float *ring,
                     float *frame, fft_complex_t *bins, fft_complex_t *scratch);

/* Drop all tuning: bypass every biquad, clear the recorded center freqs, and
 * zero the filter state. Keeps the config/buffers/analyzer wiring intact. Use on
 * a disengage edge (e.g. throttle idle) so no stale notch is applied on the next
 * spool-up. */
void notch_bank_reset(notch_bank_t *nb);

/* Feed one (decimated) sample to the analyzer. Returns 1 when a frame is ready
 * — the caller should then call notch_bank_update — and 0 otherwise. This does
 * NOT filter; it only observes the spectrum. */
int notch_bank_observe(notch_bank_t *nb, float sample);

/* Run the FFT analysis and retune the cascade: the i-th strongest peak drives
 * biquad i (designed at filter_fs_hz), any surplus biquads are bypassed. Biquad
 * state is preserved (bump-free). Returns the number of notches now active. Only
 * call when notch_bank_observe returned 1 (otherwise a no-op returning 0). */
unsigned notch_bank_update(notch_bank_t *nb);

/* Live-update the detection parameters: notch Q (band-stop width, applied at the
 * next retune) and the analyzer's band / prominence gate (fmin_hz, fmax_hz,
 * min_peak_ratio). Non-positive arguments are ignored so a caller can set just
 * one field by passing 0 for the rest. Scalar writes — safe to call from another
 * task; a concurrent retune sees at worst a one-frame mix of old/new. */
void notch_bank_set_detection(notch_bank_t *nb, float q, float fmin_hz,
                              float fmax_hz, float min_ratio);

/* Fallback policy for a slot that finds no peak in an update: 0 (default) =
 * bypass it (pass through), non-zero = HOLD its previous coefficients, so a peak
 * that momentarily dips below threshold doesn't make the notch flicker off. A
 * slot that has never been tuned still passes through under hold. */
void notch_bank_set_hold(notch_bank_t *nb, int hold_on_miss);

/* Filter one gyro sample through the active biquad cascade, at filter_fs_hz.
 * Bypassed sections pass through unchanged, so this is safe to call before the
 * first update (acts as identity). Cheap enough for the rate-loop hot path. */
float notch_bank_filter(notch_bank_t *nb, float sample);

#endif /* VAYU_DSP_NOTCH_BANK_H */
