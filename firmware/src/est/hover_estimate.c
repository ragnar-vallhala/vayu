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
#include "est/hover_estimate.h"

#include "maths/maths_interface.h"

/** @noreq Trivial clamp helper. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** @noreq Seed from the airframe constant; see hover_estimate.h. */
void hover_est_init(hover_est_t *h, float initial) {
  h->estimate = clampf(initial, HOVER_EST_MIN, HOVER_EST_MAX);
  h->measured = false;
  h->settle_t = 0.0f;
}

/** @noreq One estimator step; see hover_estimate.h. */
float hover_est_update(hover_est_t *h, bool in_air, float collective,
                       float climb_rate, float vert_accel, float cos_tilt,
                       float dt) {
  /* Steady, level, airborne, and commanding a plausible hover collective. The
   * collective band is a gate in its own right: during a punch-out every other
   * condition can momentarily read fine at the top of a ballistic arc. */
  bool steady = in_air && dt > 0.0f &&
                m_fabsf(climb_rate) < HOVER_EST_CLIMB_MAX_MS &&
                m_fabsf(vert_accel) < HOVER_EST_ACCEL_MAX &&
                cos_tilt > HOVER_EST_COS_TILT_MIN &&
                collective > HOVER_EST_MIN && collective < HOVER_EST_MAX;

  if (!steady) {
    h->settle_t = 0.0f;
    return h->estimate;
  }

  /* Require the conditions to have HELD — a transient crossing of "steady" on
   * the way through a manoeuvre is not a hover observation. */
  h->settle_t += dt;
  if (h->settle_t < HOVER_EST_SETTLE_S) {
    return h->estimate;
  }

  /* Slew toward the observation. Rate-limited rather than filtered so a single
   * bad sample can only ever move the estimate by slew*dt. */
  float step = HOVER_EST_SLEW_PER_S * dt;
  float err = collective - h->estimate;
  if (err > step) {
    err = step;
  } else if (err < -step) {
    err = -step;
  }
  h->estimate = clampf(h->estimate + err, HOVER_EST_MIN, HOVER_EST_MAX);
  h->measured = true;
  return h->estimate;
}
