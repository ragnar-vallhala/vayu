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
/* Sensor-agnostic calibration engine — runs the acquisition loop + fit for a
 * calib_target_t and commits only on success. The ellipsoid path here is the
 * generalised form of a magnetometer free-rotation fit, shared across sensors.
 * See include/calib/calib_engine.h. */

#include "calib/calib_engine.h"
#include "calib/calib_ellipsoid.h"
#include "vaios.h"                 /* v_delay */
#include "maths/maths_interface.h" /* m_sqrt */
#include <stddef.h>                /* NULL */

/* Accumulate one raw 3-vector (scaled by 1/radius) into the normal equations.
 * @noreq Internal normal-equation accumulation helper for the ellipsoid fit. */
static void accum(float S[81], float t9[9], const float m[3], float inv_r) {
  float x = m[0] * inv_r, y = m[1] * inv_r, z = m[2] * inv_r;
  float r[9] = {x * x,     y * y, z * z, 2 * y * z, 2 * x * z,
                2 * x * y, 2 * x, 2 * y, 2 * z};
  for (int a = 0; a < 9; a++) {
    t9[a] += r[a];
    for (int b = 0; b < 9; b++)
      S[a * 9 + b] += r[a] * r[b];
  }
}

/* Solve the accumulated system, optionally rescale to an absolute radius, and
 * commit. `pts`/`npts` are needed only when normalize_radius is set (to measure
 * the common corrected magnitude). Returns 0 (committed) or -1.
 * @noreq Shared fit solve + optional radius-normalisation + commit; the
 * per-sensor behaviour is carried by the SNS-CAL reqs on the callers. */
static int finalize(const calib_target_t *t, float S[81], float t9[9],
                    int nvalid, const float (*pts)[3], int npts) {
  if (nvalid < (int)t->min_samples)
    return -1;

  float offset[3], soft[9];
  if (calib_fit_ellipsoid(S, t9, offset, soft) != 0)
    return -1;

  /* offset comes back in scaled units (samples were divided by radius). */
  for (int i = 0; i < 3; i++)
    offset[i] *= t->radius;

  /* The bare fit normalises corrected vectors to the geometric-mean semi-axis.
   * For an absolute-magnitude sensor (accel: |a|=g) rescale soft so the mean
   * corrected magnitude over the samples is exactly `radius`. */
  if (t->normalize_radius && pts && npts > 0) {
    const float inv_r = 1.0f / t->radius;
    float msum = 0.0f;
    for (int j = 0; j < npts; j++) {
      float d[3] = {(pts[j][0] - offset[0]) * inv_r,
                    (pts[j][1] - offset[1]) * inv_r,
                    (pts[j][2] - offset[2]) * inv_r};
      float c0 = soft[0] * d[0] + soft[1] * d[1] + soft[2] * d[2];
      float c1 = soft[3] * d[0] + soft[4] * d[1] + soft[5] * d[2];
      float c2 = soft[6] * d[0] + soft[7] * d[1] + soft[8] * d[2];
      msum += m_sqrt(c0 * c0 + c1 * c1 + c2 * c2);
    }
    float mean = msum / (float)npts;
    if (mean > 1e-6f)
      for (int i = 0; i < 9; i++)
        soft[i] /= mean;
  }

  t->commit(offset, soft, t->ctx);
  return 0;
}

/* Sphere-constrained fit (accel, mag): accumulate the 9x9 normal equations over
 * the acquisition window, tracking per-axis raw-component coverage, then fit the
 * ellipsoid and commit offset (in raw units) + the 3x3 shape matrix.
 * @implements SNS-CAL-103 */
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
      accum(S, t9, m, inv_r);
      nvalid++;

      for (int k = 0; k < 3; k++) {
        if (m[k] < cmin[k])
          cmin[k] = m[k];
        if (m[k] > cmax[k])
          cmax[k] = m[k];
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

  /* free-running acquisition keeps no samples, so no radius normalisation here */
  return finalize(t, S, t9, nvalid, NULL, 0);
}

/* Zero-rate bias (gyro): average raw samples that the provider only returns when
 * the board is still, then commit the mean as the offset (matrix = identity).
 * Fails (keeps the old offset) if too few still samples are gathered within the
 * tick budget, or if the accepted samples are too noisy.
 * @implements SNS-CAL-102 */
static int run_bias(const calib_target_t *t) {
  float sum[3] = {0.0f, 0.0f, 0.0f};
  float sumsq[3] = {0.0f, 0.0f, 0.0f};
  int n = 0;
  const int iters = (int)t->max_ticks;
  const int prog_period = (iters / 20) > 0 ? (iters / 20) : 1;

  for (int i = 0; i < iters; i++) {
    if (t->cancelled && t->cancelled(t->ctx))
      return -1;
    float v[3];
    if (t->read_raw(v, t->ctx)) {
      for (int k = 0; k < 3; k++) {
        sum[k] += v[k];
        sumsq[k] += v[k] * v[k];
      }
      n++;
      if (n >= (int)t->min_samples)
        break;
    }
    if (t->on_progress && (i % prog_period) == 0) {
      float pct = 100.0f * (float)n / (float)t->min_samples;
      t->on_progress(pct > 100.0f ? 100.0f : pct, t->ctx);
    }
    v_delay(t->poll_ms);
  }

  if (n < (int)t->min_samples)
    return -1; // couldn't gather enough still samples (board kept moving)

  float mean[3];
  for (int k = 0; k < 3; k++)
    mean[k] = sum[k] / (float)n;

  if (t->bias_var_max > 0.0f) {
    for (int k = 0; k < 3; k++) {
      float var = sumsq[k] / (float)n - mean[k] * mean[k];
      if (var > t->bias_var_max)
        return -1; // accepted window too noisy — disturbed capture
    }
  }

  const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  t->commit(mean, identity, t->ctx);
  return 0;
}

/** @implements SNS-CAL-102, SNS-CAL-103 */
int calib_engine_run(const calib_target_t *t) {
  switch (t->fit) {
  case CALIB_FIT_ELLIPSOID:
    return run_ellipsoid(t);
  case CALIB_FIT_BIAS:
    return run_bias(t);
  default:
    return -1;
  }
}

/** @implements SNS-CAL-101 */
int calib_engine_fit_points(const calib_target_t *t, const float (*pts)[3],
                            int npts) {
  /* Closed-form 6-side accel fit: classify the collected poses into the six
   * axis-aligned sides and solve directly (no normal equations). Selected via
   * the target's fit type so the caller can pick it in place of the ellipsoid
   * LSQ with no other change. */
  if (t->fit == CALIB_FIT_SIXPOINT) {
    if (npts < (int)t->min_samples)
      return -1;
    float offset[3], soft[9];
    if (calib_fit_sixpoint(pts, npts, t->radius, offset, soft) != 0)
      return -1;
    t->commit(offset, soft, t->ctx);
    return 0;
  }

  float S[81] = {0};
  float t9[9] = {0};
  const float inv_r = 1.0f / t->radius;
  for (int j = 0; j < npts; j++)
    accum(S, t9, pts[j], inv_r);
  return finalize(t, S, t9, npts, pts, npts);
}
