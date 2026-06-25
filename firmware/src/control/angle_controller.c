#include "control/angle_controller.h"
#include "comm/comm.h"
#include "control/angle_rate_controller.h"
#include "control/flight_mode.h"
#include "control/pid_config.h"
#include "maths/maths_interface.h"
#include "control/pid.h"
#include "est/est.h"
#include "sensor/sensor.h"
#include "structure.h"
#include "sys/state.h"
#include "vaios.h"
#include "variables.h"

#define ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE 4

static angle_controller_outputs_t
    angle_controller_out_buf[ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE] = {
        0};
static spsc_fifo_t angle_controller_fifo;
static void init_fifo(void) {
  spsc_init(&angle_controller_fifo, angle_controller_out_buf,
            ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE,
            sizeof(angle_controller_outputs_t));
  spsc_set_policy(&angle_controller_fifo, SPSC_POLICY_OVERWRITE);
}
static void fifo_push(angle_controller_outputs_t *outputs) {
  spsc_write(&angle_controller_fifo, outputs, 1);
}

bool angle_controller_get_outputs(angle_controller_outputs_t *outputs) {
  return spsc_read(&angle_controller_fifo, outputs, 1);
}

/* Latest commanded throttle, mirrored out-of-band so non-consuming observers
 * (the takeoff/landing detector) don't steal from the rate-controller FIFO. */
static volatile float _last_throttle = 0.0f;
float angle_controller_last_throttle(void) { return _last_throttle; }

static angle_controller_t angle_controller = {
    .pid = {
        {
            .Kp = DEAFULT_ROLL_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_ROLL_ANGLE_OUT_MAX,
            .out_max = DEAFULT_ROLL_ANGLE_OUT_MAX,
            .initialized = false,
        },
        {
            .Kp = DEAFULT_PITCH_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_PITCH_ANGLE_OUT_MAX,
            .out_max = DEAFULT_PITCH_ANGLE_OUT_MAX,
            .initialized = false,
        },
        {
            .Kp = DEAFULT_YAW_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_YAW_ANGLE_OUT_MAX,
            .out_max = DEAFULT_YAW_ANGLE_OUT_MAX,
            .initialized = false,
        },
    }};

void angle_controller_init(void) {
  init_fifo();
  for (int i = 0; i < NUM_AXES; i++) {
    v_pid_init(&angle_controller.pid[i], angle_controller.pid[i].Kp,
               angle_controller.pid[i].Ki, angle_controller.pid[i].Kd,
               angle_controller.pid[i].Kff, angle_controller.pid[i].i_max,
               angle_controller.pid[i].d_max, angle_controller.pid[i].d_lpf_rc,
               angle_controller.pid[i].out_min,
               angle_controller.pid[i].out_max);

    /* COMM-CMD-003: override compiled defaults with any persisted tune. */
    float kp, ki, kd, kff;
    if (pid_config_get_angle((uint8_t)i, &kp, &ki, &kd, &kff)) {
      v_pid_set_gains(&angle_controller.pid[i], kp, ki, kd, kff);
    }
  }
}

bool angle_controller_set_gains(uint8_t axis, float kp, float ki, float kd,
                                float kff) {
  if (axis >= NUM_AXES) {
    return false;
  }
  v_pid_set_gains(&angle_controller.pid[axis], kp, ki, kd, kff);
  return true;
}

typedef struct {
  // normal rc channels
  float channels[4];
  //
} rc_data_t;

static inline rc_data_t normalize_rc_data(ibus_data_t rc_data) {
  rc_data_t normalized_rc_data;
  for (int i = 0; i < 4; i++) {
    if (i != 2) {
      // Apply deadband: linear outside the ±PID_RC_DEADBAND band around
      // centre (1500), zero inside. (Both sides map identically, so the
      // former separate if/else-if branches are merged — bugprone-branch-clone.)
      if (rc_data.channels[i] > 1500 + PID_RC_DEADBAND ||
          rc_data.channels[i] < 1500 - PID_RC_DEADBAND) {
        normalized_rc_data.channels[i] =
            ((float)rc_data.channels[i] - 1500.0f) / 500.0f;
      } else {
        normalized_rc_data.channels[i] = 0;
      }
    } else {
      // Throttle is not deaband at 1500
      normalized_rc_data.channels[i] =
          ((float)rc_data.channels[i] - 1000.0f) / 1000.0f;
    }
  }

  // Clamp to the valid normalized range BEFORE any expo shaping. A missing or
  // out-of-range RC reading — e.g. channel == 0 at startup, or a glitch frame —
  // maps to (0-1500)/500 = -3, which the cubic expo blows up to -27 -> a
  // -2700 deg angle command. Clamping caps every stick at its intended ±full
  // throw (throttle 0..1), so a bad sample can't inject a runaway setpoint.
  for (int i = 0; i < 4; i++) {
    const float lo = (i == 2) ? 0.0f : -1.0f;
    if (normalized_rc_data.channels[i] > 1.0f) normalized_rc_data.channels[i] = 1.0f;
    if (normalized_rc_data.channels[i] < lo) normalized_rc_data.channels[i] = lo;
  }

  switch (PID_RC2ANGLE_RATE_MODE) {
  case NORMALIZED_RC2ANGLE_RATE_LINEAR:
    break;
  case NORMALIZED_RC2ANGLE_RATE_CUBIC:
    for (int i = 0; i < 4; i++) {
      if (i != 2) {
        normalized_rc_data.channels[i] = normalized_rc_data.channels[i] *
                                         normalized_rc_data.channels[i] *
                                         normalized_rc_data.channels[i];
      }
    }
    break;
  default:
    /* LINEAR mapping (no shaping) — also the safe fallback. */
    break;
  }
  return normalized_rc_data;
}

void angle_controller_task(void *arg) {
  (void)arg;
  angle_controller_init();
  static attitude_t attitude;
  static attitude_t last_attitude;
  static angle_controller_outputs_t angle_controller_outputs;
  static ibus_data_t rc_data;
  static ibus_data_t prev_rc_data;
  /* Hard-pinned outer-loop rate (OUTER_LOOP_FREQ_HZ = INNER_LOOP_FREQ_HZ /
   * OUTER_LOOP_DECIM): drift-free periodic schedule, constant integration
   * timestep. The estimator runs faster, so each tick we take the freshest
   * attitude (drained below). */
  uint32_t last_wake = v_get_ticks();
  const float dt = OUTER_LOOP_DT;
  while (1) {
    /* Drift-free periodic wait. Ticks even if the estimator stalls so the
     * bank-angle failsafe / motor path keep updating (the attitude drain below
     * then reuses the last estimate). */
    task_delay_until(&last_wake, OUTER_LOOP_PERIOD_TICKS);

    if (rc_queue_control_pop(&rc_data)) {
      prev_rc_data = rc_data;
    } else {
      rc_data = prev_rc_data;
    }
    // Normalizing the rc data
    rc_data_t normalized_rc_data = normalize_rc_data(rc_data);
    float target_angles[NUM_AXES];

    target_angles[0] =
        normalized_rc_data.channels[0] * DEAFULT_ROLL_ANGLE_TARGET_MAX;
    target_angles[1] =
        -normalized_rc_data.channels[1] *
        DEAFULT_PITCH_ANGLE_TARGET_MAX; // Forward push if acieved by tilting
                                        // back motors up which is negative
                                        // pitch
    float target_throttle = normalized_rc_data.channels[2];
    target_angles[2] =
        normalized_rc_data.channels[3] * DEAFULT_YAW_ANGLE_TARGET_MAX;

    /* Acro (rate) mode toggle on RC channel ACRO_SWITCH_CH (ch6, 0-based 5).
     * High -> sticks command body rate directly, attitude loop bypassed. The
     * RC request is arbitrated with any GCS CMD_SET_FLIGHT_MODE override (GCS
     * wins while active); the resolved mode is published as telemetry. */
    bool rc_acro = (ACRO_SWITCH_CH < IBUS_MAX_CHANNELS) &&
                   (rc_data.channels[ACRO_SWITCH_CH] > ACRO_SWITCH_US);
    bool acro_mode = flight_mode_resolve_acro(rc_acro);

    /* Freshest attitude estimate (drain the OVERWRITE ring; the estimator runs
     * faster than this loop). dt is the constant OUTER_LOOP_DT declared above. */
    bool got_att = false;
    while (attitude_queue_control_pop(&attitude)) {
      got_att = true;
    }
    if (!got_att) {
      attitude = last_attitude; // estimator stalled — hold last estimate
    }
    last_attitude = attitude;
    /* Bank-angle failsafe applies in angle mode only. In acro the airframe is
     * meant to exceed MAX_ANGLE_CUTOFF (flips/rolls), so it must not trip. */
    if (!acro_mode &&
        (m_fabsf(attitude.roll) > MAX_ANGLE_CUTOFF ||
         m_fabsf(attitude.pitch) > MAX_ANGLE_CUTOFF)) {
      // Yaw is intentionally excluded from the failsafe condition:
      // a drone can rotate freely around its vertical axis without
      // being in danger, and on a sim build without working mag
      // correction the mahony filter's yaw drifts past 45 deg over
      // tens of seconds while the airframe is sitting still. That
      // false-tripped FAILSAFE the moment the integrator's small
      // bias accumulated. Real hardware with a calibrated mag would
      // be largely immune, but the check is unnecessary either way.
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
    }
    // Calculate target rates
    float current_angles[NUM_AXES];
    current_angles[0] = attitude.roll;
    current_angles[1] = attitude.pitch;
    current_angles[2] = attitude.yaw;
    if (acro_mode) {
      /* Stick -> body-rate setpoint [deg/s] directly (attitude PID bypassed).
       * Pitch keeps the same sign convention as angle mode (forward = nose
       * down = negative). */
      angle_controller_outputs.angle_rates[0] =
          normalized_rc_data.channels[0] * DEAFULT_ROLL_ACRO_RATE_MAX;
      angle_controller_outputs.angle_rates[1] =
          -normalized_rc_data.channels[1] * DEAFULT_PITCH_ACRO_RATE_MAX;
      angle_controller_outputs.angle_rates[2] =
          normalized_rc_data.channels[3] * DEAFULT_YAW_ACRO_RATE_MAX;
    } else {
      /* Angle (stabilise) mode: roll & pitch are attitude-controlled (stick ->
       * bank angle, self-levels when centered). YAW, however, is ALWAYS rate-
       * controlled like acro — the stick commands a yaw RATE and a centered
       * stick holds the current heading. A quad must never snap back to an
       * absolute heading of 0, and the mag-less sim yaw estimate drifts anyway,
       * so heading-hold-to-zero is both wrong and unstable. */
      for (int i = 0; i < 2; i++) { /* roll, pitch */
        angle_controller_outputs.angle_rates[i] = v_pid_update(
            &angle_controller.pid[i], target_angles[i], current_angles[i], 0, dt);
      }
      angle_controller_outputs.angle_rates[2] =
          normalized_rc_data.channels[3] * DEAFULT_YAW_ACRO_RATE_MAX;
    }
    /* Telemetry continuity: report the stick angle command vs measured angle in
     * both modes (in acro it's informational only — the rate setpoint above is
     * what actually drives the inner loop). */
    for (int i = 0; i < NUM_AXES; i++) {
      angle_controller_outputs.angle_sp[i] = target_angles[i];
      angle_controller_outputs.angle_curr[i] = current_angles[i];
    }
    angle_controller_outputs.throttle = target_throttle;
    angle_controller_outputs.dt = dt;
    _last_throttle = target_throttle;
    fifo_push(&angle_controller_outputs);
    /* No v_delay here — task_delay_until() (above) drives a drift-free periodic
     * schedule at OUTER_LOOP_PERIOD_TICKS; the loop drains the freshest attitude
     * sample each tick rather than blocking on queue arrival. */
  }
}
