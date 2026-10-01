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
 * @file vertical_task.c
 * @brief Vertical estimator (VERT) task — sibling of the attitude task.
 *
 * The vertical estimate runs in its own task rather than inside attitude_task,
 * so vertical estimation is decoupled from the attitude loop's timing. It drains
 * the synchronized {q, body specific force,
 * dt} triple the attitude task publishes (vert_input_queue) — same sample the
 * EKF ran on, so attitude and accel are self-consistent — and integrates that
 * into climb_rate/altitude (predict). It corrects against the latest BME280
 * altitude whenever a fresh baro sample appears (latest-wins, no baro queue;
 * the driver publishes ~10-20 Hz). The fused state is published to
 * vertical_state_queue for the control loop / IN_AIR detector / telemetry.
 *
 * The math core (predict/correct/gravity-removal) lives in
 * src/est/vertical_estimator.c and is unit-tested headlessly
 * (sim/host/tests/test_vertical_est.c). This file is just the I/O wrapper.
 */
#include "control/angle_controller.h"  /* angle_controller_last_throttle */
#include "control/height_controller.h" /* HEIGHT_HOVER_GUESS (initial seed) */
#include "est/flight_phase.h"
#include "storage/fs_owner.h" /* vayu_log */
#include "storage/imu_hs_log.h"
#include "est/hover_estimate.h"
#include "storage/hover_store.h"
#include "est/vertical_estimator.h"
#include "maths/linalg.h" /* m_quat_rotate */
#include "hub/hub.h"
#include "sys/state.h" /* system_state_get/set, SYSTEM_STATE_* */
#include "vaios.h"
#include "vaios_app_config.h"
#include "vayu_tasks.h"
#include <stdbool.h>

/* Cap the blocking wait so the task keeps publishing (and the baro path keeps
 * correcting) even if attitude input stalls; normally it is sample-driven. */
#define VERT_MAX_PERIOD_MS 50u

/* The estimator-side gates are recorded as the task evaluated them, so a
 * recording says WHICH one rejected the ToF rather than only that something
 * did. TOF_FRESH separates those from the driver rejecting a read before the
 * hub ever sees it. Here rather than inline in the task so it can be tested. */
uint16_t vert_log_flags(const vertical_state_t *s, bool tof_fresh,
                        uint32_t tof_age_steps, float cos_tilt) {
  return (
      uint16_t)((s->tof_valid ? HSL_VRT_F_TOF_VALID : 0u) |
                (s->accel_unhealthy ? HSL_VRT_F_ACCEL_UNHEALTHY : 0u) |
                (s->valid ? HSL_VRT_F_VALID : 0u) |
                (s->hover_measured ? HSL_VRT_F_HOVER_MEASURED : 0u) |
                (tof_fresh ? HSL_VRT_F_TOF_FRESH : 0u) |
                (tof_age_steps >= VERT_TOF_STALE_STEPS ? HSL_VRT_F_TOF_STALE
                                                       : 0u) |
                (cos_tilt <= VERT_TOF_MAX_TILT_COS ? HSL_VRT_F_TOF_TILT : 0u) |
                ((s->agl_tof < VERT_TOF_MIN_M || s->agl_tof > VERT_TOF_MAX_M)
                     ? HSL_VRT_F_TOF_RANGE
                     : 0u));
}

/* @implements EST-ALT-001 */
void vertical_estimator_task(void *args) {
  (void)args;
  vertical_estimator_t ve;
  vert_est_init_default(&ve);

  flight_phase_t fp;
  flight_phase_init(&fp);

  /* Hover estimate: seeded from the airframe constant, then measured in steady
   * level flight. Everything vertical keys off this number. */
  hover_est_t hov;
  /* Seed from SD when a previous flight measured one, so the FIRST lift-off
   * after a reboot already uses what was learned rather than the compiled
   * guess. Degrades to the guess if the card, file or value is missing/bad. */
  hover_est_init(&hov, hover_store_load(HEIGHT_HOVER_GUESS));
  bool hov_was_armed = false;
  float hov_saved = hov.estimate;

  uint32_t last_baro_stamp = 0;
  bool have_baro_stamp = false;

  uint32_t last_tof_stamp = 0;
  bool have_tof_stamp = false;
  uint32_t tof_age_steps = VERT_TOF_STALE_STEPS;
  float agl_tof = 0.0f;
  bool tof_valid = false;
  bool tof_fresh = false; /* this iteration carries a NEW range sample */
  float tof_dt = 0.0f;    /* seconds since the last sample fed to the aiding */

  while (1) {
    /* Block until the attitude task hands over the next synchronized triple
     * (or the cap elapses, so the baro correction still ticks on a stall). */
    if (!vert_input_queue_wait(MS_TO_TICKS(VERT_MAX_PERIOD_MS)))
      continue;

    vert_input_t in;
    if (!vert_input_queue_pop(&in))
      continue;

    /* Predict: integrate world-up inertial acceleration over this step. */
    float a_up = vert_world_up_accel(&in.q, in.a_body);
    vert_est_predict(&ve, a_up, in.dt);
    tof_dt += in.dt; /* age of the next range sample, for the aiding below */

    /* Correct: fold in the latest baro altitude when a new sample is ready.
     * bme280_read_all() returns the last published reading with its own
     * acquisition stamp; we correct only when that stamp advances so each baro
     * sample is used once (latest-wins). */
    baro_sample_t baro;
    float baro_alt = ve.altitude; /* fallback for telemetry before first baro */
    if (baro_latest(&baro) && baro.valid) {
      /* Pressure -> height happens here, where the datum lives. */
      baro_alt = hub_altitude_m(baro.pressure_pa, HUB_SEA_LEVEL_PA_DEFAULT);
      if (!have_baro_stamp || baro.t_cyc != last_baro_stamp) {
        last_baro_stamp = baro.t_cyc;
        have_baro_stamp = true;
        vert_est_correct(&ve, baro_alt);
      }
    }

    /* Tilt, from the attitude alone. This MUST NOT live inside the rangefinder
     * branch: vl53l0x_read_all() returns an error until the first in-window
     * sample, so on a board with no working ToF (and in SITL, which has no
     * feeder) cos_tilt would sit at 1.0 forever and the hover estimator's tilt
     * gate would never reject anything — banked samples read high, biasing the
     * persisted hover upward, which is exactly how a lift-off opens too hot. */
    const float body_down[3] = {0.0f, 0.0f, 1.0f};
    float w_down[3];
    m_quat_rotate(&in.q, body_down, w_down);
    const float cos_tilt = w_down[2];

    /* Hover-collective estimate. */
    hover_est_update(&hov, system_state_get() == SYSTEM_STATE_IN_AIR,
                     angle_controller_last_throttle(), ve.climb_rate,
                     ve.vertical_accel, cos_tilt, in.dt);

    /* Persist on the disarm edge: the flight's learned value is final by then,
     * it is naturally rare, and it costs one SD write per flight rather than
     * one per loop. Only when it actually moved — rewriting an unchanged value
     * is pure wear. */
    {
      sys_state_t hst = system_state_get();
      bool armed_now =
          (hst == SYSTEM_STATE_ARMED) || (hst == SYSTEM_STATE_IN_AIR);
      if (hov_was_armed && !armed_now && hov.measured &&
          m_fabsf(hov.estimate - hov_saved) > 0.005f) {
        if (hover_store_save(hov.estimate)) {
          hov_saved = hov.estimate;
          vayu_log("hover: saved %d/1000", (int)(hov.estimate * 1000));
        }
      }
      hov_was_armed = armed_now;
    }

    /* Rangefinder AGL. Deliberately NOT fused into the estimator: the ToF is an
     * AGL reference over whatever is directly below (it steps when the ground
     * does), while the filter tracks a baro reference. Folding one into the
     * other makes the fused altitude jump on every terrain step and every
     * in/out-of-range transition. Instead it is published alongside, and the
     * height controller picks the better source and re-biases on handoff. */
    range_sample_t tof;
    if (range_latest(&tof) && tof.valid) {
      tof_fresh = false;
      if (!have_tof_stamp || tof.t_cyc != last_tof_stamp) {
        last_tof_stamp = tof.t_cyc;
        have_tof_stamp = true;
        tof_age_steps = 0;
        tof_fresh = true;
      } else if (tof_age_steps < VERT_TOF_STALE_STEPS) {
        tof_age_steps++;
      }
      /* Tilt compensation reuses the cos(tilt) computed above: the world-down
       * component of the body-down axis both scales the slant range to a
       * vertical height and gates on how far off level we are. */
      agl_tof = tof.range_m * cos_tilt;
      tof_valid = (tof_age_steps < VERT_TOF_STALE_STEPS) &&
                  (cos_tilt > VERT_TOF_MAX_TILT_COS) &&
                  (agl_tof >= VERT_TOF_MIN_M) && (agl_tof <= VERT_TOF_MAX_M);
    } else {
      tof_valid = false;
      tof_fresh = false;
    }

    /* Rangefinder aiding for the accel-bias state. This is the ONLY place the
     * ToF touches the filter, and it moves nothing but the bias — see the block
     * comment in vertical_estimator.h. It matters because the bias appears at
     * throttle-up: learned from baro alone it needs ~7 s, which is the whole
     * lift-off, and a position measurement cannot do better because it only
     * reveals an accel bias after two integrations. Differentiating the ToF
     * gives a VELOCITY measurement, one integration closer, and the ToF is in
     * range exactly over the takeoff band. Feed only fresh samples that already
     * passed the publication gate; anything else breaks the chain so the next
     * pair is not differenced across the hole. */
    if (tof_valid && tof_fresh) {
      vert_est_correct_tof(&ve, agl_tof, tof_dt);
      tof_dt = 0.0f;
    } else if (!tof_valid) {
      vert_est_tof_gap(&ve);
    }

    /* Takeoff / landing detector + FC-owned AGL ground reference.
     * Only meaningful once the filter is seeded; until then the ground reference
     * has no absolute altitude to anchor to. `armed` (ARMED or IN_AIR) freezes
     * the ground reference; `in_air` selects the landing vs takeoff test. */
    sys_state_t st = system_state_get();
    bool in_air = (st == SYSTEM_STATE_IN_AIR);
    bool armed = (st == SYSTEM_STATE_ARMED) || in_air;
    if (ve.initialized) {
      flight_phase_event_t ev = flight_phase_update(
          &fp, armed, in_air, ve.altitude, baro_alt, agl_tof, tof_valid,
          ve.climb_rate, angle_controller_last_throttle(), in.dt);
      if (ev == FLIGHT_PHASE_EVENT_TAKEOFF) {
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_IN_AIR));
      } else if (ev == FLIGHT_PHASE_EVENT_LAND) {
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
      }
    }

    /* Publish the fused state (OVERWRITE ring — consumers read the latest). */
    vertical_state_t out = {
        .altitude = ve.altitude,
        .climb_rate = ve.climb_rate,
        .vertical_accel = ve.vertical_accel,
        .baro_altitude = baro_alt,
        .agl = fp.agl,
        .agl_tof = agl_tof,
        .tof_valid = tof_valid,
        .hover_est = hov.estimate,
        .hover_measured = hov.measured,
        .accel_bias = ve.accel_bias,
        .accel_unhealthy = ve.accel_unhealthy,
        .valid = ve.initialized,
        .timestamp = in.timestamp,
    };
    vertical_state_queue_push(&out);

    /* High-speed SD stream, "vrt": the estimator's own view, on the same
     * timebase as the vibration that corrupts it. accel_bias is the most
     * direct read-out of how badly motor noise is rectifying the accelerometer,
     * and it is meaningless without the raw accel beside it. Decimated
     * internally to 20 Hz, and a no-op unless armed. */
    {
      hsl_vert_sample_t hv = {
          .baro_altitude = out.baro_altitude,
          .agl = out.agl,
          .agl_tof = out.agl_tof,
          .altitude = out.altitude,
          .climb_rate = out.climb_rate,
          .accel_bias = out.accel_bias,
          .flags = vert_log_flags(&out, tof_fresh, tof_age_steps, cos_tilt),
      };
      imu_hs_log_vert(&hv, out.timestamp);
    }
  }
}
