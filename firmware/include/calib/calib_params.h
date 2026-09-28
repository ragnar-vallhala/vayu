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
/**
 * @file calib/calib_params.h
 * @brief How each calibration routine decides it has enough good data.
 *
 * Sample counts, stillness gates, pose-coverage thresholds and the choice of
 * accel fit. These are the procedure's parameters, not the vehicle's tune --
 * changing one changes what a calibration ACCEPTS, which is why they are not
 * in control/tuning.h next to the gains.
 */
#ifndef VAYU_CALIB_PARAMS_H
#define VAYU_CALIB_PARAMS_H

#define CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION                             \
  2000 // 2 seconds, waits before recording once user has reached the direction
       // orientation
#define CALIBRATION_WAIT_USER_TIME_POST_CALIBRATION                            \
  1000 // 1 seconds, waits after recording before saving the calibration
#define CALIBRATION_SAMPLE_COUNT                                               \
  500 // Number of samples to take for calibration
#define MAG_FIT_MIN_SAMPLES                                                    \
  400 // minimum valid samples required to attempt the mag ellipsoid fit

/* Accel calibration fit method (SNS-CAL). Optional switch between the two fits:
 *   ELLIPSOID — pose-tolerant full-3x3 least-squares over 6 faces + 6 edges
 *               (captures misalignment, needs coverage, can reject degenerate
 *               data). This is the historical default.
 *   SIXPOINT  — PX4-style exact closed form over the 6 faces only
 *               (deterministic, simpler "lay it on each side" UX, cannot land on
 *               a degenerate fit; still recovers the full 3x3 in vayu since the
 *               soft-iron store is 3x3). See docs/plans/imu-calib-improvements.md.
 * The capture flow adapts to the choice (SIXPOINT prompts the 6 faces only). */
#define ACCEL_CALIB_ELLIPSOID 0
#define ACCEL_CALIB_SIXPOINT 1
#ifndef ACCEL_CALIB_METHOD
#define ACCEL_CALIB_METHOD                                                     \
  ACCEL_CALIB_SIXPOINT /* 6-side closed-form accel fit */
#endif

/* Pose-tolerant full-3x3 accel calibration (calib engine, point-set fit). */
#define ACCEL_CAL_POSES                                                        \
  12 // 6 faces + 6 edges/corners — enough spread directions for a 9-DOF fit
#define ACCEL_CAL_MIN_POSES                                                    \
  9 // minimum captured poses to attempt the fit (9 DOF)
#define ACCEL_POSE_STILL_SAMPLES                                               \
  100 // contiguous static samples averaged per pose (~2 s at the 50 Hz cal poll)
#define ACCEL_CAL_GYRO_STILL_DPS                                               \
  3.0f // |gyro| below this (per axis sum-of-squares) counts the board as still
#define ACCEL_CAL_FACE_POSES                                                   \
  6 // first N of the prompt list are the 6 faces; the rest are edges/corners

/* Pose-coverage gate (full-3x3 accel). A still hold is banked only if it ADVANCES
 * coverage, so the same orientation can't be recorded twice and the 9-DOF
 * ellipsoid always sees directions spanning the sphere. A "face" hold must have
 * one body axis clearly dominate (|a_dom|/|a| >= FACE_DOMINANCE) and land on a
 * signed body axis no prior face used — there are exactly six, so the six face
 * prompts must cover all six. An "edge/corner" hold must instead SHARE gravity
 * (no axis dominates, second-largest component >= EDGE_MIN_SECOND) and sit at
 * least acos(MIN_SEP_COS) from every direction already banked. Matching is by
 * geometric distinctness, not the prompted code, so it is independent of how the
 * board's axes are signed/mounted. */
#define ACCEL_POSE_FACE_DOMINANCE                                              \
  0.85f // |a_dom|/|a| for a hold to count as a clean face (~32 deg cone)
#define ACCEL_POSE_EDGE_MIN_SECOND                                             \
  0.40f // 2nd-largest |a_i|/|a| required for a shared-gravity edge/corner
#define ACCEL_POSE_MIN_SEP_COS                                                 \
  0.866f // edge holds must sit > 30 deg apart (cos 30) to count as distinct

/* Stillness-gated gyro bias capture (calib engine, bias fit). */
#define GYRO_CAL_STILL_SAMPLES                                                 \
  300 // still samples to average for the bias (~6 s at the 50 Hz cal poll)
#define GYRO_CAL_MAX_TICKS                                                     \
  1500 // ~30 s budget; if the board never settles, fail (keep the old offset)
#define GYRO_CAL_VAR_MAX                                                       \
  1.0f // dps^2 per-axis variance ceiling on the accepted window

#endif // VAYU_CALIB_PARAMS_H
