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
#include "est/flight_phase.h"
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
/* Usable band. Below the device floor the reading is unreliable; the ceiling is
 * kept under the sensor's own limit so we hand back to baro before it starts
 * reporting its out-of-range sentinel. */
#define VERT_TOF_MIN_M 0.05f
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
      /* Tilt compensation: rotate the body-down axis into the world; its
       * world-down component IS cos(tilt), so it both scales the slant range to
       * a vertical height and gates on how far off level we are. */
      const float body_down[3] = {0.0f, 0.0f, 1.0f};
      float w_down[3];
      m_quat_rotate(&in.q, body_down, w_down);
      float cos_tilt = w_down[2];
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
      flight_phase_event_t ev =
          flight_phase_update(&fp, armed, in_air, ve.altitude, baro_alt,
                              ve.climb_rate, angle_controller_last_throttle(),
                              in.dt);
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
        .valid = ve.initialized,
        .timestamp = in.timestamp,
    };
    vertical_state_queue_push(&out);
  }
}
