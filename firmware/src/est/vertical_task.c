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
#include "control/angle_controller.h" /* angle_controller_last_throttle */
#include "control/height_controller.h" /* HEIGHT_HOVER_GUESS (initial seed) */
#include "est/flight_phase.h"
#include "est/hover_estimate.h"
#include "storage/hover_store.h"
#include "est/vertical_estimator.h"
#include "maths/linalg.h" /* m_quat_rotate */
#include "sensor/bme280.h"
#include "sensor/vl53l0x.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h" /* system_state_get/set, SYSTEM_STATE_* */
#include "vaios.h"
#include "vaios_app_config.h"
#include "variables.h" /* MS_TO_TICKS via the task/config chain */
#include "vayu_tasks.h"
#include <stdbool.h>

/* Cap the blocking wait so the task keeps publishing (and the baro path keeps
 * correcting) even if attitude input stalls; normally it is sample-driven. */
#define VERT_MAX_PERIOD_MS 50u

/* Rangefinder acceptance. The VL53L0X measures along the body-down axis, so its
 * reading is the VERTICAL height only after multiplying by cos(tilt); past ~30
 * deg both that correction and the beam footprint stop being trustworthy (the
 * cone is looking sideways at whatever the craft is banked toward). */
#define VERT_TOF_MAX_TILT_COS 0.866f /* cos(30 deg) */
/* Usable band. The floor is the DEVICE floor (VL53L0X_RANGE_MIN_MM, 30 mm) and
 * not a comfort margin above it: the sensor sits ~45 mm off the ground on its
 * feet, so a floor of 0.05 made tof_valid go false exactly when the craft was
 * landed — blinding the touchdown detector at the one moment it needs the ToF
 * most. The mounting offset is removed downstream by flight_phase's own ToF
 * ground reference, so a low reading here is data, not noise. The ceiling stays
 * under the sensor's limit so we hand back to baro before it starts reporting
 * its out-of-range sentinel. */
#define VERT_TOF_MIN_M 0.03f
#define VERT_TOF_MAX_M 1.50f
/* Consecutive predict steps (~250 Hz) tolerated without a fresh in-window
 * range before the ToF is declared stale. ~50 steps ~= 200 ms, an order above
 * the ~21 Hz ride-along cadence. */
#define VERT_TOF_STALE_STEPS 50u

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

    /* Correct: fold in the latest baro altitude when a new sample is ready.
     * bme280_read_all() returns the last published reading with its own
     * acquisition stamp; we correct only when that stamp advances so each baro
     * sample is used once (latest-wins). */
    bme280_reading_t baro;
    float baro_alt = ve.altitude; /* fallback for telemetry before first baro */
    if (bme280_read_all(&baro) == HAL_OK) {
      baro_alt = baro.altitude_m;
      if (!have_baro_stamp || baro.timestamp != last_baro_stamp) {
        last_baro_stamp = baro.timestamp;
        have_baro_stamp = true;
        vert_est_correct(&ve, baro.altitude_m);
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
    vl53l0x_reading_t tof;
    if (vl53l0x_read_all(&tof) == HAL_OK) {
      if (!have_tof_stamp || tof.timestamp != last_tof_stamp) {
        last_tof_stamp = tof.timestamp;
        have_tof_stamp = true;
        tof_age_steps = 0;
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
  }
}
