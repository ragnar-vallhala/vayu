#include "control/angle_controller.h"
#include "comm/comm.h"
#include "control/angle_rate_controller.h"
#include "control/flight_mode.h"
#include "control/height_controller.h"
#include "control/throttle_curve.h"
#include "est/vertical_estimator.h"
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
/** @noreq static outputs-FIFO init (infrastructure). */
static void init_fifo(void) {
  spsc_init(&angle_controller_fifo, angle_controller_out_buf,
            ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE,
            sizeof(angle_controller_outputs_t));
  spsc_set_policy(&angle_controller_fifo, SPSC_POLICY_OVERWRITE);
}
/** @noreq static outputs-FIFO push (infrastructure). */
static void fifo_push(angle_controller_outputs_t *outputs) {
  spsc_write(&angle_controller_fifo, outputs, 1);
}

/** @noreq outputs-FIFO read (infrastructure). */
bool angle_controller_get_outputs(angle_controller_outputs_t *outputs) {
  return spsc_read(&angle_controller_fifo, outputs, 1);
}

/* Latest commanded throttle, mirrored out-of-band so non-consuming observers
 * (the takeoff/landing detector) don't steal from the rate-controller FIFO. */
static volatile float _last_throttle = 0.0f;
static volatile uint8_t _height_state = 0;
/** @noreq latest-throttle accessor (non-destructive observer). */
float angle_controller_last_throttle(void) { return _last_throttle; }

/** @noreq Packed height-mode status for telemetry (see the header). */
uint8_t angle_controller_height_state(void) { return _height_state; }

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

/**
 * Init the angle-loop PIDs from the compiled defaults, then override each axis
 * with any tune persisted to SD and restored at boot (pid_config_get_angle).
 *
 * @implements COMM-CMD-003
 */
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
} rc_data_t;

/**
 * Normalise the RC sticks: a ±PID_RC_DEADBAND deadband around 1500, linear map
 * to the normalised range (throttle 0..1, others ±1) with a pre-shape clamp,
 * then the PID_RC2ANGLE_RATE_MODE expo curve (CUBIC default) — the stick
 * mapping of CTRL-ANGLE-103 (per-axis ° scaling is applied by the caller).
 *
 * @implements CTRL-ANGLE-103
 */
static inline rc_data_t normalize_rc_data(ibus_data_t rc_data) {
  rc_data_t normalized_rc_data;
  for (int i = 0; i < 4; i++) {
    // Fail SAFE to centre on an implausible reading. A valid RC pulse is
    // ~1000..2000 us; a dropout / uninitialised channel (0 at boot, or a glitch
    // frame) is not a stick position. Without this guard a 0 reading maps to
    // (0-1500)/500 = -3 and clamps to FULL deflection -> a railed angle setpoint:
    // the airframe self-commands a full front/right lean and won't self-level
    // toward it (corrections stop firing for those directions). Treat it as
    // centred: roll/pitch/yaw -> level, throttle -> min.
    if (rc_data.channels[i] < 900 || rc_data.channels[i] > 2100) {
      normalized_rc_data.channels[i] = 0.0f;
      continue;
    }
    if (i != 2) {
      // Apply deadband: linear outside the ±PID_RC_DEADBAND band around
      // centre (1500), zero inside. Both sides of centre map identically.
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

/**
 * Outer angle/stabilise loop: a drift-free ~500 Hz periodic task that reads the
 * latest RC sticks and attitude estimate and writes rate setpoints to the rate
 * loop (CTRL-ANGLE-101). Roll/pitch are pure-P attitude-controlled with the
 * output saturated (CTRL-ANGLE-102); the RC ACRO switch selects acro (rate,
 * SYS-CTRL-001) vs stabilise (angle, SYS-CTRL-002) mode (SYS-CTRL-003). In
 * angle mode |roll|/|pitch| beyond MAX_ANGLE_CUTOFF requests FAILSAFE
 * (CTRL-FAIL-001 / SYS-SAFE-004); yaw is excluded by design.
 *
 * @implements CTRL-ANGLE-101, CTRL-ANGLE-102, SYS-CTRL-001, SYS-CTRL-002, SYS-CTRL-003, CTRL-FAIL-001, SYS-SAFE-004
 */
void angle_controller_task(void *arg) {
  (void)arg;
  angle_controller_init();
  static attitude_t attitude;
  static attitude_t last_attitude;
  static angle_controller_outputs_t angle_controller_outputs;
  static ibus_data_t rc_data;
  static ibus_data_t prev_rc_data;
  /* Bank-angle recovery + height-hold state (task-local, single writer). */
  static bool s_recovering = false;
  static uint32_t s_recovery_ticks = 0;
  static bool s_mode_seen_centre = false;
  static height_ctrl_t s_height;
  height_ctrl_reset(&s_height);
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
    /* Collective stick shaping: hover at mid-stick, linear in THRUST, expo
     * derived from hover. The raw map put hover at 38% of travel on this
     * airframe and made mid-stick a +0.7 g climb; see control/throttle_curve.h.
     * Roll/pitch/yaw are untouched.
     *
     * Centre the curve on the MEASURED hover once the estimator has one — both
     * references do this (PX4 slews its curve centre to the hover-thrust
     * estimate; ArduPilot learns MOT_THST_HOVER). HEIGHT_HOVER_GUESS is only
     * the seed, so the stick re-centres itself on the airframe's real hover
     * instead of trusting a number derived from assumed thrust. */
    vertical_state_t vs;
    bool have_vs = vertical_state_queue_peek(&vs) && vs.valid;
    float hover_now =
        (have_vs && vs.hover_measured) ? vs.hover_est : HEIGHT_HOVER_GUESS;
    float target_throttle =
        throttle_curve(normalized_rc_data.channels[2], hover_now);
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
    /* Bank-angle upset handling, angle mode only. In acro the airframe is
     * meant to exceed MAX_ANGLE_CUTOFF (flips/rolls), so it must not trip.
     *
     * IN FLIGHT this recovers instead of cutting the motors: level demand at
     * hover collective until the craft is back under MAX_ANGLE_RECOVER. Cutting
     * thrust mid-air (what this did before) is unrecoverable by construction —
     * motor.c zeroes all four motors outside ARMED/IN_AIR and the state table
     * has no way back. ON THE GROUND the cut is kept: there, stopping the props
     * is right. See the MAX_ANGLE_RECOVER block in variables.h for the incident
     * this came from.
     *
     * Yaw is intentionally excluded from the condition: a drone can rotate
     * freely around its vertical axis without being in danger, and on a sim
     * build without working mag correction the mahony filter's yaw drifts past
     * 45 deg over tens of seconds while the airframe is sitting still. That
     * false-tripped FAILSAFE the moment the integrator's small bias
     * accumulated. Real hardware with a calibrated mag would be largely immune,
     * but the check is unnecessary either way. */
    const uint32_t recovery_timeout_ticks =
        (uint32_t)((RECOVERY_TIMEOUT_MS * 0.001f) / OUTER_LOOP_DT);
    float abs_roll = m_fabsf(attitude.roll);
    float abs_pitch = m_fabsf(attitude.pitch);
    float tilt = (abs_roll > abs_pitch) ? abs_roll : abs_pitch;
    bool in_air = (system_state_get() == SYSTEM_STATE_IN_AIR);
    if (acro_mode) {
      s_recovering = false;
    } else if (in_air) {
      if (!s_recovering) {
        if (tilt > MAX_ANGLE_CUTOFF) {
          s_recovering = true;
          s_recovery_ticks = 0;
        }
      } else if (tilt < MAX_ANGLE_RECOVER) {
        s_recovering = false; /* flown out of it */
      } else if (++s_recovery_ticks > recovery_timeout_ticks) {
        /* Not coming back (inverted, dead motor, broken prop). Holding hover
         * thrust from here only drives it in harder — stop the props. */
        s_recovering = false;
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
      }
    } else {
      s_recovering = false;
      if (tilt > MAX_ANGLE_CUTOFF) {
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
      }
    }
    if (s_recovering) {
      /* FC flies it out: wings level, no yaw demand, hover collective. This
       * overrides the pilot — including a chopped throttle stick, which is what
       * removes the rate loop's authority in the first place. */
      target_angles[0] = 0.0f;
      target_angles[1] = 0.0f;
      target_angles[2] = 0.0f;
      normalized_rc_data.channels[3] = 0.0f;
      /* Hover, measured — see the note where RECOVERY_THROTTLE used to live. */
      target_throttle = hover_now;
    }

    /* Height mode — 3-position switch, collective only. The sticks keep doing
     * exactly what they always did; this takes the collective (and only the
     * collective) while the switch is off centre:
     *
     *   centre -> OFF   pilot's stick passes through
     *   up     -> HOLD  lift to 1 m and hold, or hold current height in flight
     *   down   -> LAND  descend and settle, then idle
     *
     * Interlocks: angle mode only, never while recovering from an upset
     * (recovery outranks it), and the switch must have been seen CENTRED since
     * arming — otherwise arming with it already up would fly the craft off the
     * ground unprompted.
     *
     * Height source: the tilt-compensated rangefinder AGL when it is in range
     * and the craft is near level, else the baro-derived AGL. The controller
     * re-anchors its setpoint across that handoff (height_controller.h). */
    height_mode_t hmode = HEIGHT_MODE_OFF;
    if (ALT_MODE_CH < IBUS_MAX_CHANNELS) {
      uint16_t mode_us = rc_data.channels[ALT_MODE_CH];
      /* Note the deliberate asymmetry: HOLD (the position that can fly the
       * craft off the ground) additionally requires a PLAUSIBLE RC value, so a
       * dead/unmapped channel reading 0 can never be mistaken for it. LAND
       * needs no such guard — failing safe toward the ground is fine. */
      if (mode_us >= ALT_MODE_VALID_MIN_US && mode_us < ALT_MODE_HOLD_US) {
        hmode = HEIGHT_MODE_HOLD;
      } else if (mode_us > ALT_MODE_LAND_US) {
        hmode = HEIGHT_MODE_LAND;
      }
    }
    /* Disarmed clears the interlock memory, so every flight has to pass through
     * centre again before a mode can take the collective. */
    sys_state_t hstate = system_state_get();
    if (hstate != SYSTEM_STATE_ARMED && hstate != SYSTEM_STATE_IN_AIR) {
      s_mode_seen_centre = false;
    } else if (hmode == HEIGHT_MODE_OFF) {
      s_mode_seen_centre = true;
    }
    /* ...and never on a climb_rate we know is wrong. The height controller's
     * inner loop IS climb_rate, so a corrupted one does not degrade the mode,
     * it inverts it: a phantom descent is answered with more collective, which
     * makes more vibration, which deepens the phantom. The estimator raises
     * this once its bias state runs out of authority to cancel the error
     * (vertical_estimator.h). Vetoing hands the pilot back the collective
     * through the normal OFF path, with its throttle re-sync. */
    bool hmode_vetoed = (acro_mode || s_recovering || !s_mode_seen_centre ||
                         (have_vs && vs.accel_unhealthy));
    height_mode_t hmode_req = hmode;
    if (hmode_vetoed) {
      hmode = HEIGHT_MODE_OFF;
    }

    if (have_vs) {
      float alt = vs.tof_valid ? vs.agl_tof : vs.agl;
      target_throttle =
          height_ctrl_update(&s_height, hmode, in_air, target_throttle, alt,
                             vs.tof_valid, vs.climb_rate, dt, hover_now);
    } else {
      /* No usable vertical estimate — the pilot gets the collective back, but
       * through the same OFF path (and so the same throttle re-sync) rather
       * than a cold reset that would dump it on a possibly-idle stick. */
      target_throttle =
          height_ctrl_update(&s_height, HEIGHT_MODE_OFF, in_air, target_throttle,
                             0.0f, false, 0.0f, dt, hover_now);
    }

    /* Recovery outranks everything, and has to be applied LAST to actually mean
     * it. The height block above still runs (its state must stay coherent), but
     * with the mode vetoed to OFF it can return an armed hand-back value of up
     * to HEIGHT_BASE_MAX — which would quietly replace the recovery collective
     * mid-upset. Re-assert it here so the override is final. */
    if (s_recovering) {
      target_throttle = hover_now;
    }

    /* Publish what the mode is doing, so "I flipped the switch and nothing
     * happened" is answerable from telemetry instead of a log hunt. */
    _height_state =
        (uint8_t)(((uint8_t)hmode_req & HEIGHT_STATE_MODE_MASK) |
                  (s_height.engaged ? HEIGHT_STATE_ENGAGED : 0u) |
                  (s_height.failed ? HEIGHT_STATE_FAILED : 0u) |
                  (s_height.landed ? HEIGHT_STATE_LANDED : 0u) |
                  (s_height.handback ? HEIGHT_STATE_HANDBACK : 0u) |
                  (s_mode_seen_centre ? HEIGHT_STATE_ARMED_OK : 0u) |
                  ((hmode_vetoed && hmode_req != HEIGHT_MODE_OFF)
                       ? HEIGHT_STATE_BLOCKED : 0u));
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
