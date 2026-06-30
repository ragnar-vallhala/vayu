#ifndef MATHS_INTERFACE_H
#define MATHS_INTERFACE_H
#define PI 3.14159265358979323846f

float m_sin(float x);
float m_cos(float x);
float m_asin(float x);
float m_atan2(float y, float x);
float m_sqrt(float x);
float m_pow(float base, float exp);
float m_clamp(float val, float min, float max);
float m_fabsf(float x);
/* Non-zero iff x is NaN. Lets consumers guard against NaN without pulling in
 * <math.h> themselves (this module is the single owner of libm). */
int m_isnan(float x);
/* Non-zero iff x is finite (not NaN or +/-inf). */
int m_isfinite(float x);
#define to_radians(degrees) (degrees * (PI / 180.0f))
#define to_degrees(radians) (radians * (180.0f / PI))
typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef struct {
  float *values;
  int length;
} vector_t;

typedef struct {
  float **data;
  int rows;
  int cols;
} matrix_t;

void normalize_vector(vector_t *v);
void normalize_quaternion(quaternion_t *q);
void quaternion_multiply(const quaternion_t *qa, const quaternion_t *qb,
                         quaternion_t *out);
void quaternion_conjugate(const quaternion_t *q, quaternion_t *out);
void quaternion_from_euler(float roll, float pitch, float yaw, quaternion_t *q);

// -------------------------
// FFT (src/maths/fft.c)
// -------------------------
// Self-contained radix-2 FFT for the FFT-driven gyro notch (design:
// firmware/docs/journal/log-analysis/20260625-233852-pitch-indi-campaign/
// reference-autopilots-comparison.md §10.5-10.6). Plain float, no CMSIS-DSP,
// in-place, caller owns all buffers (no static/heap state -> .bss stays flat,
// the F401 boot invariant). The whole API works off a SINGLE full-N twiddle
// table of N/2 entries; the complex core indexes it with a stride so a
// half-size FFT reuses the same table (W_{N/2}^k = W_N^{2k}), which is what
// lets the real-FFT wrapper and its inner complex FFT share one const table.

// Interleaved complex sample. POD (not C99 _Complex) so it is identical under
// any toolchain / C++ and trivially const-table'able.
typedef struct {
  float re;
  float im;
} fft_complex_t;

// Fill `tw` (>= n/2 entries) with the forward twiddles W_N^k = exp(-i*2pi*k/N),
// k = 0..n/2-1. Call once at init (uses libm); the hot path then reads the
// table. `n` must be a power of two >= 2. The default size is also available as
// a const-in-flash table (maths/fft_tables_N128.h), so this need not run.
void m_fft_make_twiddles(fft_complex_t *tw, unsigned n);

// In-place iterative radix-2 DIT FFT of `n` complex points (power of two >= 1).
// `tw` holds forward twiddles at the FULL table size base_n = n * tw_stride. A
// standalone transform passes tw_stride == 1 with an n/2-entry table; the
// real-FFT wrapper passes tw_stride == 2 so an n-point input's n/2-point inner
// FFT reuses the full-N table. Output is the unnormalized forward DFT.
void m_fft_forward(fft_complex_t *x, unsigned n, const fft_complex_t *tw,
                   unsigned tw_stride);

// In-place inverse FFT of `n` complex points, 1/n-normalized so that
// m_fft_inverse(m_fft_forward(x)) == x up to float rounding. Uses the same
// forward twiddle table via the conjugate identity; mainly for round-trip /
// Parseval testing.
void m_fft_inverse(fft_complex_t *x, unsigned n, const fft_complex_t *tw,
                   unsigned tw_stride);

// Real-input FFT: `n` real samples (power of two >= 2) -> n/2+1 complex bins
// (DC..Nyquist). out[0].im and out[n/2].im are 0. `scratch` is a caller-owned
// work buffer of >= n/2 complex entries. `tw` is the full-N forward twiddle
// table (n/2 entries). `out` and `scratch` must not alias; `in` aliases neither.
void m_rfft_forward(const float *in, unsigned n, fft_complex_t *out,
                    fft_complex_t *scratch, const fft_complex_t *tw);

// Squared magnitude |b|^2 (no sqrt — peak-pick and thresholding work in power).
static inline float m_fft_bin_power(fft_complex_t b) {
  return b.re * b.re + b.im * b.im;
}

// -------------------------
// Biquad notch / band-stop (src/maths/biquad.c)
// -------------------------
// One second-order section, the evaluator half of the FFT-driven gyro notch
// (design ref as above, §10.5-10.6): the FFT analysis front-end picks the prop
// peak; this filters it out of the gyro stream. Same constraints as the FFT —
// portable float, no CMSIS, no static/heap state (.bss stays flat): the caller
// owns the coeffs and the per-channel state. libm (sinf/cosf) is touched only by
// the coefficient designer, which runs at the gated coeff-update cadence, never
// per sample; m_biquad_step is plain arithmetic (~5 mul + 4 add).

// Transfer-function coefficients, a0 normalized to 1 (so it drops out of the
// recurrence). One set per notch; const after design until the center frequency
// is re-estimated.
typedef struct {
  float b0, b1, b2; // feedforward
  float a1, a2;     // feedback
} biquad_coeffs_t;

// Direct-Form II Transposed delay state. One per filtered channel (e.g. per gyro
// axis), zeroed by m_biquad_reset before first use. df2t is the canonical choice
// for time-varying coeffs: the state holds filtered history, not raw input, so
// retuning the center frequency mid-stream stays bump-free and well-conditioned.
typedef struct {
  float s1, s2;
} biquad_state_t;

// Design an RBJ band-stop (notch): unity gain everywhere except a null at
// `f0_hz`, width set by quality factor `q` (higher q == narrower). `fs_hz` is the
// sample rate the filter runs at. Out-of-range args (f0 not in (0, fs/2),
// q <= 0, fs <= 0) fail safe to a bypass (identity) section rather than emitting
// unstable coeffs — flight code must never get a blowing-up filter from a bad
// peak estimate.
void m_biquad_notch_design(biquad_coeffs_t *c, float f0_hz, float q,
                           float fs_hz);

// Identity section: y == x. Used as the fail-safe and to disable a notch slot.
void m_biquad_bypass(biquad_coeffs_t *c);

// Zero the delay state (call once before the first sample on a channel).
static inline void m_biquad_reset(biquad_state_t *st) {
  st->s1 = 0.0f;
  st->s2 = 0.0f;
}

// Filter one sample (Direct-Form II Transposed). Hot path — inline, no libm.
static inline float m_biquad_step(const biquad_coeffs_t *c, biquad_state_t *st,
                                  float x) {
  float y = c->b0 * x + st->s1;
  st->s1 = c->b1 * x - c->a1 * y + st->s2;
  st->s2 = c->b2 * x - c->a2 * y;
  return y;
}

#endif // !MATHS_INTERFACE_H
