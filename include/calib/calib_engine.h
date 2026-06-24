#ifndef CALIB_ENGINE_H
#define CALIB_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

/* Sensor-agnostic calibration engine. A sensor describes its calibration as a
 * calib_target_t — a raw-sample source, a result sink, and a few thresholds —
 * and the engine runs the acquisition + fit, committing only on success. This
 * decouples the fit/orchestration from any particular driver so the
 * accelerometer, magnetometer (and future sensors) share one path. The fit math
 * itself lives in calib_ellipsoid.{c,h}. */

typedef enum {
  CALIB_FIT_ELLIPSOID, /* sphere-constrained: offset + 3x3 (accel, mag) */
  CALIB_FIT_BIAS,      /* zero-rate bias: mean of still samples (gyro, Phase 5) */
} calib_fit_t;

typedef struct calib_target {
  const char *name;        /* used in log lines */
  calib_fit_t fit;
  float       radius;      /* ellipsoid target |v| in raw units (g, |B|, ...) */
  uint16_t    min_samples; /* gate before a fit is attempted */
  float       cov_done;    /* per-axis coverage % to early-finish (ellipsoid) */
  uint16_t    max_ticks;   /* hard cap on acquisition ticks */
  uint16_t    poll_ms;     /* delay between acquisition ticks */

  /* Pop one RAW physical 3-vector (uT, m/s^2, ...). Return false if no valid
   * sample is ready this tick — the engine skips it but still counts the tick
   * (so the max_ticks timeout still bounds the routine). */
  bool (*read_raw)(float v[3], void *ctx);
  /* Poll for an operator cancel. NULL = never cancelled. */
  bool (*cancelled)(void *ctx);
  /* Per-axis coverage 0..100 for the GCS progress UI. NULL = no reporting. */
  void (*on_coverage)(float cx, float cy, float cz, void *ctx);
  /* Store a successful fit: offset in raw units, mat = row-major 3x3 (identity
   * for BIAS). Called at most once, only on a successful fit. */
  void (*commit)(const float offset[3], const float mat[9], void *ctx);
  void *ctx;
} calib_target_t;

/* Run the calibration described by t. Returns 0 if a fit succeeded and commit()
 * was called; -1 on cancel / too-few-samples / fit failure (the caller then
 * keeps the previous calibration). */
int calib_engine_run(const calib_target_t *t);

#endif /* CALIB_ENGINE_H */
