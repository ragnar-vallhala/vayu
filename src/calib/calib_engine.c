/* Sensor-agnostic calibration engine — runs the acquisition loop + fit for a
 * calib_target_t and commits only on success. The ellipsoid path here is the
 * generalised form of the magnetometer free-rotation fit that used to live
 * inline in bmx160.c. See include/calib/calib_engine.h. */

#include "calib/calib_engine.h"
#include "calib/calib_ellipsoid.h"
#include "vaios.h" /* v_delay */

/* Sphere-constrained fit (accel, mag): accumulate the 9x9 normal equations over
 * the acquisition window, tracking per-axis raw-component coverage, then fit the
 * ellipsoid and commit offset (in raw units) + the 3x3 shape matrix. */
static int run_ellipsoid(const calib_target_t *t) {
  float S[81] = {0};
  float t9[9] = {0};
  int nvalid = 0;

  /* Per-axis coverage from the RAW component span (offset-invariant): under full
   * coverage each component sweeps ~[-radius, +radius] so its span -> 2*radius. */
  float cmin[3] = {1e9f, 1e9f, 1e9f};
  float cmax[3] = {-1e9f, -1e9f, -1e9f};
  float cov[3] = {0.0f, 0.0f, 0.0f};
  const float inv_r = 1.0f / t->radius;
  const float cov_scale = 100.0f / (2.0f * t->radius); /* span -> % */
  const int iters = (int)t->max_ticks;
  const int cov_period = (iters / 20) > 0 ? (iters / 20) : 1;

  for (int i = 0; i < iters; i++) {
    if (t->cancelled && t->cancelled(t->ctx))
      return -1;

    float m[3];
    if (t->read_raw(m, t->ctx)) {
      float x = m[0] * inv_r, y = m[1] * inv_r, z = m[2] * inv_r;
      float r[9] = {x * x,     y * y, z * z, 2 * y * z, 2 * x * z,
                    2 * x * y, 2 * x, 2 * y, 2 * z};
      for (int a = 0; a < 9; a++) {
        t9[a] += r[a];
        for (int b = 0; b < 9; b++)
          S[a * 9 + b] += r[a] * r[b];
      }
      nvalid++;

      for (int k = 0; k < 3; k++) {
        if (m[k] < cmin[k]) cmin[k] = m[k];
        if (m[k] > cmax[k]) cmax[k] = m[k];
        float c = (cmax[k] - cmin[k]) * cov_scale;
        cov[k] = c < 0.0f ? 0.0f : (c > 100.0f ? 100.0f : c);
      }
    }

    if (t->on_coverage && (i % cov_period) == 0)
      t->on_coverage(cov[0], cov[1], cov[2], t->ctx);

    /* Finish as soon as every axis is well covered and the fit has enough
     * points; the max_ticks loop is the fallback cap. */
    if (nvalid >= (int)t->min_samples && cov[0] >= t->cov_done &&
        cov[1] >= t->cov_done && cov[2] >= t->cov_done)
      break;

    v_delay(t->poll_ms);
  }

  if (nvalid < (int)t->min_samples)
    return -1;

  float offset[3], soft[9];
  if (calib_fit_ellipsoid(S, t9, offset, soft) != 0)
    return -1;

  /* offset is in scaled units (samples were divided by radius); restore raw. */
  for (int i = 0; i < 3; i++)
    offset[i] *= t->radius;

  t->commit(offset, soft, t->ctx);
  return 0;
}

int calib_engine_run(const calib_target_t *t) {
  switch (t->fit) {
  case CALIB_FIT_ELLIPSOID:
    return run_ellipsoid(t);
  case CALIB_FIT_BIAS:
  default:
    /* Gyro bias path lands in Phase 5 (separate routine). */
    return -1;
  }
}
