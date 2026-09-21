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
#ifndef VAYU_HOVER_ESTIMATE_H
#define VAYU_HOVER_ESTIMATE_H

#include <stdbool.h>

/*
 * In-flight hover-collective estimate.
 *
 * Hover is the one number the whole vertical stack keys off: the throttle
 * curve centres the stick on it, the height mode opens a lift-off at it, and
 * the flight-phase throttle gate is placed relative to it. Until now it was a
 * compiled-in constant derived from an ASSUMED thrust per motor — and being
 * wrong by 1.7x is what flew the airframe into the ceiling on 2026-09-04.
 *
 * Both references measure it instead. PX4 runs a hover-thrust estimator and
 * slew-limits the curve's centre to it (mc_att_control_main.cpp: the default
 * MPC_THR_CURVE case interpolates through `_hover_thrust_slew_rate`, with the
 * MPC_THR_HOVER parameter used only when no estimate is finite). ArduPilot
 * learns MOT_THST_HOVER in flight. This is the same idea, kept deliberately
 * simple: no filter model, just the observation that
 *
 *   in steady level flight with no vertical acceleration, the collective
 *   being commanded IS hover.
 *
 * So it waits for those conditions to hold, then slews toward the commanded
 * collective at HOVER_EST_SLEW_PER_S (PX4 uses 0.05/s; matched here). Slow on
 * purpose: this must not chase a gust or a stick input, and a wrong fast answer
 * is worse than a slow right one.
 *
 * Deliberately NOT modelled: tilt compensation. Holding altitude at bank angle
 * needs thrust/cos(tilt), so a banked sample reads high. Rather than correct it
 * the sampler simply refuses to look while banked — cheaper and it cannot be
 * wrong in a way that biases the estimate upward.
 */

/* Sampling gates — all must hold, continuously, for HOVER_EST_SETTLE_S. */
#ifndef HOVER_EST_CLIMB_MAX_MS
#define HOVER_EST_CLIMB_MAX_MS 0.30f /* |climb rate|, m/s */
#endif
#ifndef HOVER_EST_ACCEL_MAX
#define HOVER_EST_ACCEL_MAX 0.50f /* |vertical accel|, m/s^2 */
#endif
#ifndef HOVER_EST_COS_TILT_MIN
#define HOVER_EST_COS_TILT_MIN 0.95f /* ~18 deg; banked samples read high */
#endif
#ifndef HOVER_EST_SETTLE_S
#define HOVER_EST_SETTLE_S 0.50f
#endif

/* Only ever believe a collective inside this band — outside it the craft is
 * not hovering whatever the other gates say. */
#ifndef HOVER_EST_MIN
#define HOVER_EST_MIN 0.10f
#endif
#ifndef HOVER_EST_MAX
#define HOVER_EST_MAX 0.80f
#endif

/* Convergence rate toward an accepted sample (collective units per second). */
#ifndef HOVER_EST_SLEW_PER_S
#define HOVER_EST_SLEW_PER_S 0.05f
#endif

typedef struct {
  float estimate; /**< current hover collective. Seeded from the airframe
                   *   constant, then measured. */
  bool measured;  /**< true once a real in-flight sample has moved it — lets a
                   *   consumer tell a measurement from the initial guess. */
  float settle_t; /**< s the sampling gates have held continuously. */
} hover_est_t;

/* Seed with the airframe's compiled-in guess; `measured` stays false. */
void hover_est_init(hover_est_t *h, float initial);

/* One step. Returns the current estimate (always usable — it is the seed until
 * a real sample arrives).
 *   in_air      FC believes it is flying; on the ground nothing is learned
 *   collective  commanded collective (0..1)
 *   climb_rate  fused climb rate (m/s)
 *   vert_accel  fused vertical acceleration (m/s^2, gravity removed)
 *   cos_tilt    world-down component of the body-down axis (1 = level)
 *   dt          step (s) */
float hover_est_update(hover_est_t *h, bool in_air, float collective,
                       float climb_rate, float vert_accel, float cos_tilt,
                       float dt);

#endif /* VAYU_HOVER_ESTIMATE_H */
