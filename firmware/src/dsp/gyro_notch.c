/* gyro_notch.c — firmware glue owning the per-axis dynamic notch banks.
 *
 * Allocates every DSP working buffer on the heap at init (keeps .bss flat),
 * drives one notch_bank per gyro axis, and staggers the FFT retune to one axis
 * per tick. Two gates guard it: the VAYU_FFT_NOTCH COMPILE switch (below —
 * needs the FPU) and a runtime throttle gate (the notch only engages once the
 * props spin up past GYRO_NOTCH_THROTTLE_MIN, where the vibration it targets
 * actually exists). See dsp/gyro_notch.h for the contract and the single-task /
 * disabled-by-default constraints. Design ref §9.3 / §10. */
#include "dsp/gyro_notch.h"

#ifdef VAYU_FFT_NOTCH

#include "dsp/notch_bank.h"
#include "maths/fft_tables_N128.h" /* const FFT_HANN_N128 / FFT_TWIDDLE_N128 */
#include "memory.h"                /* v_malloc */
#include "variables.h"             /* NUM_AXES, INNER_LOOP_FREQ_HZ */

/* FFT / band parameters. N is fixed at 128 to reuse the const flash tables (Hann
 * window + twiddles) — the zero-heap-tables path from notch_fft. Band + Q are
 * sensible props-noise defaults; the tuning surface (task #6) will make them
 * live-settable. */
#define GYRO_NOTCH_N 128u
#define GYRO_NOTCH_NOTCHES 3u
#define GYRO_NOTCH_Q 8.0f
#define GYRO_NOTCH_FMIN_HZ 60.0f
#define GYRO_NOTCH_FMAX_HZ 450.0f /* < INNER_LOOP_FREQ_HZ/2 = 500 Hz Nyquist */
#define GYRO_NOTCH_MIN_RATIO 4.0f

/* Throttle (0..1) below which the notch stays disengaged: props effectively
 * idle, so there is no prop-wash line to track and analysing sensor noise would
 * only mistune. */
#define GYRO_NOTCH_THROTTLE_MIN 0.10f

/* Target analyzer sample rate. The biquads always run at the full loop rate
 * (INNER_LOOP_FREQ_HZ); the FFT front-end only needs a Nyquist comfortably above
 * the analysis band (fmax = 450 Hz), so once the loop rate climbs past ~1 kHz we
 * DECIMATE the observe path — feeding the analyzer every Dth sample — to keep its
 * bin resolution in-band and its per-frame cost bounded. D = floor(loop / this),
 * clamped >= 1. At the current 1 kHz loop D == 1 and nothing changes. Keep this
 * strictly above 2*fmax so the decimated Nyquist never falls into the band. */
#define GYRO_NOTCH_ANALYZER_FS_HZ 1000.0f

/* Auto-band: a one-shot pass that watches the peaks the analyzer actually finds
 * (while engaged) and then tightens the detection band around them, replacing
 * the hand-picked default with the craft's real vibration signature. Learns over
 * this many retunes, brackets the observed min/max peak by this margin, and never
 * drops the lower edge below the floor. Live-only (the learned band isn't
 * auto-persisted; re-issue an explicit tune to save it). */
#define GYRO_NOTCH_AUTOBAND_FRAMES 40u
#define GYRO_NOTCH_AUTOBAND_MARGIN_HZ 20.0f
#define GYRO_NOTCH_FMIN_FLOOR 40.0f

/* All heap-owned so nothing here grows .bss. NULL until a successful init. */
static notch_bank_t *s_bank;       /* [NUM_AXES] */
static bool s_enabled;             /* master (user/param) gate */
static float s_throttle;           /* latest throttle 0..1, from the rate loop */
static bool s_engaged;             /* enabled AND above the throttle gate */
static bool s_ready[NUM_AXES];     /* axis has a frame awaiting retune */
static uint8_t s_service_turn;     /* round-robin cursor for gyro_notch_service */
static unsigned s_decim;           /* analyzer decimation factor D (>= 1) */
static uint8_t s_decim_phase[NUM_AXES]; /* per-axis observe phase, 0..D-1 */
static bool s_autoband;            /* auto-band learn pass armed / in progress */
static float s_ab_min, s_ab_max;   /* running peak-freq bounds during a learn */
static unsigned s_ab_frames;       /* retunes observed so far this learn */

/* Master AND throttle gate: the condition under which the notch actually filters
 * and analyses. */
static bool engaged(void) {
  return s_bank && s_enabled && s_throttle >= GYRO_NOTCH_THROTTLE_MIN;
}

/* Allocate one axis's analyzer buffers and wire up its bank. Returns false if
 * any allocation fails (caller aborts the whole init). */
static bool init_axis(unsigned axis) {
  const unsigned n = GYRO_NOTCH_N;
  float *ring = (float *)v_malloc(sizeof(float) * n);
  float *frame = (float *)v_malloc(sizeof(float) * n);
  fft_complex_t *bins = (fft_complex_t *)v_malloc(sizeof(fft_complex_t) * (n / 2u + 1u));
  fft_complex_t *scratch = (fft_complex_t *)v_malloc(sizeof(fft_complex_t) * (n / 2u));
  if (!ring || !frame || !bins || !scratch) {
    return false; /* leaked on failure, but init failure is a boot-time abort */
  }

  /* Analyzer runs at the decimated rate; the biquads run/are designed at the
   * full loop rate (filter_fs_hz below), so a peak Hz stays correctly centred
   * regardless of D. */
  notch_fft_cfg_t cfg = {.n = n,
                         .fs_hz = (float)INNER_LOOP_FREQ_HZ / (float)s_decim,
                         .fmin_hz = GYRO_NOTCH_FMIN_HZ,
                         .fmax_hz = GYRO_NOTCH_FMAX_HZ,
                         .min_peak_ratio = GYRO_NOTCH_MIN_RATIO};
  notch_bank_init(&s_bank[axis], &cfg, (float)INNER_LOOP_FREQ_HZ,
                  GYRO_NOTCH_NOTCHES, GYRO_NOTCH_Q, FFT_HANN_N128,
                  FFT_TWIDDLE_N128, ring, frame, bins, scratch);
  /* Hold a tuned notch through a brief peak dropout instead of unfiltering. */
  notch_bank_set_hold(&s_bank[axis], 1);
  return true;
}

bool gyro_notch_init(void) {
  if (s_bank) {
    return true; /* already initialised */
  }
  /* Fix the decimation factor from the loop rate before wiring any analyzer
   * (init_axis reads s_decim). floor keeps the decimated Nyquist >= the target;
   * clamp to 1 so a sub-target loop rate never decimates. */
  s_decim = (unsigned)((float)INNER_LOOP_FREQ_HZ / GYRO_NOTCH_ANALYZER_FS_HZ);
  if (s_decim < 1u) {
    s_decim = 1u;
  }
  s_bank = (notch_bank_t *)v_malloc(sizeof(notch_bank_t) * NUM_AXES);
  if (!s_bank) {
    return false;
  }
  for (unsigned i = 0; i < NUM_AXES; i++) {
    if (!init_axis(i)) {
      s_bank = 0; /* leave inert; partial buffers leaked at a failed boot */
      return false;
    }
    s_ready[i] = false;
    s_decim_phase[i] = 0;
  }
  s_enabled = false;
  s_throttle = 0.0f;
  s_engaged = false;
  s_service_turn = 0;
  return true;
}

void gyro_notch_set_enabled(bool enabled) { s_enabled = enabled; }
bool gyro_notch_enabled(void) { return s_enabled; }
void gyro_notch_set_throttle(float throttle01) { s_throttle = throttle01; }

float gyro_notch_apply(uint8_t axis, float gyro) {
  if (!engaged() || axis >= NUM_AXES) {
    return gyro;
  }
  /* Observe the RAW (pre-notch) gyro so the analyzer keeps seeing the peaks it
   * is removing; latch a retune when a frame goes ready. Decimate to the analyzer
   * rate: feed only every Dth sample (per-axis phase, since apply is called once
   * per axis per tick). At D == 1 this observes every sample. */
  if (++s_decim_phase[axis] >= s_decim) {
    s_decim_phase[axis] = 0;
    if (notch_bank_observe(&s_bank[axis], gyro)) {
      s_ready[axis] = true;
    }
  }
  return notch_bank_filter(&s_bank[axis], gyro);
}

/* Fold one axis's freshly-tuned peaks into the running auto-band bounds, and once
 * enough retunes have been seen, replace the detection band with a tight bracket
 * around what was actually observed. */
static void autoband_step(unsigned axis) {
  for (unsigned i = 0; i < s_bank[axis].num_notches; i++) {
    float f = s_bank[axis].freqs[i];
    if (f > 0.0f) {
      if (f < s_ab_min) {
        s_ab_min = f;
      }
      if (f > s_ab_max) {
        s_ab_max = f;
      }
    }
  }
  if (++s_ab_frames < GYRO_NOTCH_AUTOBAND_FRAMES) {
    return;
  }
  s_autoband = false; /* learn window elapsed */
  if (s_ab_max <= 0.0f || s_ab_max < s_ab_min) {
    return; /* never saw a peak — leave the band as-is */
  }
  float lo = s_ab_min - GYRO_NOTCH_AUTOBAND_MARGIN_HZ;
  float hi = s_ab_max + GYRO_NOTCH_AUTOBAND_MARGIN_HZ;
  if (lo < GYRO_NOTCH_FMIN_FLOOR) {
    lo = GYRO_NOTCH_FMIN_FLOOR;
  }
  float nyq = 0.5f * (float)INNER_LOOP_FREQ_HZ / (float)s_decim;
  if (hi > nyq - 10.0f) {
    hi = nyq - 10.0f;
  }
  if (hi > lo) {
    for (unsigned a = 0; a < NUM_AXES; a++) {
      notch_bank_set_detection(&s_bank[a], 0.0f, lo, hi, 0.0f);
    }
  }
}

void gyro_notch_service(void) {
  if (!s_bank) {
    return;
  }
  bool now = engaged();
  /* Disengage edge (throttle chopped / master off): drop all tuning so no stale
   * notch is applied on the next spool-up, clear any pending retunes, and abort
   * an in-flight auto-band learn (it must see a continuous engaged window). */
  if (s_engaged && !now) {
    for (unsigned i = 0; i < NUM_AXES; i++) {
      notch_bank_reset(&s_bank[i]);
      s_ready[i] = false;
      s_decim_phase[i] = 0;
    }
    s_autoband = false;
  }
  s_engaged = now;
  if (!now) {
    return;
  }

  /* At most one FFT/retune per tick: scan axes round-robin for a pending frame,
   * service the first, and advance the cursor. Bounds the worst-case tick cost
   * to a single N=128 transform regardless of how many axes went ready. */
  for (unsigned k = 0; k < NUM_AXES; k++) {
    uint8_t axis = (uint8_t)((s_service_turn + k) % NUM_AXES);
    if (s_ready[axis]) {
      s_ready[axis] = false;
      notch_bank_update(&s_bank[axis]);
      if (s_autoband) {
        autoband_step(axis);
      }
      s_service_turn = (uint8_t)((axis + 1u) % NUM_AXES);
      return;
    }
  }
}

void gyro_notch_set_params(float q, float fmin_hz, float fmax_hz,
                           float min_ratio) {
  if (!s_bank) {
    return;
  }
  for (unsigned i = 0; i < NUM_AXES; i++) {
    notch_bank_set_detection(&s_bank[i], q, fmin_hz, fmax_hz, min_ratio);
  }
}

float gyro_notch_center_hz(uint8_t axis, uint8_t idx) {
  if (!s_bank || axis >= NUM_AXES || idx >= NOTCH_BANK_MAX_NOTCHES) {
    return 0.0f;
  }
  return s_bank[axis].freqs[idx];
}

bool gyro_notch_get_params(float *q, float *fmin_hz, float *fmax_hz,
                           float *min_ratio) {
  if (!s_bank) {
    return false; /* uninitialised: leave the caller's values untouched */
  }
  /* Params are global (applied identically to every axis), so axis 0 is
   * authoritative. */
  const notch_bank_t *b = &s_bank[0];
  if (q) {
    *q = b->q;
  }
  if (fmin_hz) {
    *fmin_hz = b->fft.cfg.fmin_hz;
  }
  if (fmax_hz) {
    *fmax_hz = b->fft.cfg.fmax_hz;
  }
  if (min_ratio) {
    *min_ratio = b->fft.cfg.min_peak_ratio;
  }
  return true;
}

void gyro_notch_start_autoband(void) {
  if (!s_bank) {
    return;
  }
  /* Arm the learn. It only accumulates while engaged, so on the bench this waits
   * for the next spool-up and then characterises the hover spectrum. */
  s_autoband = true;
  s_ab_min = 1.0e9f;
  s_ab_max = 0.0f;
  s_ab_frames = 0;
}

bool gyro_notch_autoband_active(void) { return s_bank && s_autoband; }

unsigned gyro_notch_decimation(void) { return s_bank ? s_decim : 0u; }

#else /* !VAYU_FFT_NOTCH — compile the notch out; every entry point is inert. */

bool gyro_notch_init(void) { return false; }
void gyro_notch_set_enabled(bool enabled) { (void)enabled; }
bool gyro_notch_enabled(void) { return false; }
void gyro_notch_set_throttle(float throttle01) { (void)throttle01; }
float gyro_notch_apply(uint8_t axis, float gyro) {
  (void)axis;
  return gyro;
}
void gyro_notch_service(void) {}
void gyro_notch_set_params(float q, float fmin_hz, float fmax_hz,
                           float min_ratio) {
  (void)q;
  (void)fmin_hz;
  (void)fmax_hz;
  (void)min_ratio;
}
float gyro_notch_center_hz(uint8_t axis, uint8_t idx) {
  (void)axis;
  (void)idx;
  return 0.0f;
}
bool gyro_notch_get_params(float *q, float *fmin_hz, float *fmax_hz,
                           float *min_ratio) {
  (void)q;
  (void)fmin_hz;
  (void)fmax_hz;
  (void)min_ratio;
  return false;
}
void gyro_notch_start_autoband(void) {}
bool gyro_notch_autoband_active(void) { return false; }
unsigned gyro_notch_decimation(void) { return 0u; }

#endif /* VAYU_FFT_NOTCH */
