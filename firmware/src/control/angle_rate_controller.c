#include "control/angle_rate_controller.h"
#include "actuator/actuator.h"
#include "comm/comm.h"
#include "control/angle_controller.h"
#include "control/pid_config.h"
#include "control/control_buffer.h"
#include "control/pid.h"
#include "control/mixer.h"       /* dedicated control-allocation mixer */
#include "control/rate_indi.h"   /* optional INDI inner loop (RATE_CTRL_ALGO_USED) */
#include "control/sysid.h"
#include "dsp/gyro_notch.h"      /* FFT-driven dynamic gyro notch (off by default) */
#include "memory.h"   /* v_memcpy */
#include "navhal.h"
#include "sensor/sensor.h"
#include "sys/state.h"
#include "vaios.h"
#include "variables.h"

static AngleRateController angle_rate_controller = {
    .pid = {// roll angle rate gains
            {
                .Kp = DEAFULT_ROLL_ANGLE_RATE_KP,
                .Ki = DEAFULT_ROLL_ANGLE_RATE_KI,
                .Kd = DEAFULT_ROLL_ANGLE_RATE_KD,
                .Kff = DEAFULT_ROLL_ANGLE_RATE_KFF,
                .i_max = DEAFULT_ROLL_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_ROLL_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_ROLL_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_ROLL_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_ROLL_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            },
            // pitch angle rate gains
            {
                .Kp = DEAFULT_PITCH_ANGLE_RATE_KP,
                .Ki = DEAFULT_PITCH_ANGLE_RATE_KI,
                .Kd = DEAFULT_PITCH_ANGLE_RATE_KD,
                .Kff = DEAFULT_PITCH_ANGLE_RATE_KFF,
                .i_max = DEAFULT_PITCH_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_PITCH_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_PITCH_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_PITCH_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_PITCH_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            },
            // yaw angle rate gains
            {
                .Kp = DEAFULT_YAW_ANGLE_RATE_KP,
                .Ki = DEAFULT_YAW_ANGLE_RATE_KI,
                .Kd = DEAFULT_YAW_ANGLE_RATE_KD,
                .Kff = DEAFULT_YAW_ANGLE_RATE_KFF,
                .i_max = DEAFULT_YAW_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_YAW_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_YAW_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_YAW_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_YAW_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            }}};

/* Per-axis first-order low-pass on the gyro (rate measurement) feeding the
 * rate PID. rc is the filter time constant [s]; rc <= 0 disables (passthrough).
 * Co-tuned with the gains: heavier filtering raises the gain ceiling before the
 * loop hunts around the gyro deadband. Defaults off so flight behavior is
 * unchanged until a tune sets it. */
static float s_gyro_lpf_rc[NUM_AXES]    = {0.0f, 0.0f, 0.0f};
static float s_gyro_lpf_state[NUM_AXES] = {0.0f, 0.0f, 0.0f};

/* INDI inner-loop state (used only when RATE_CTRL_ALGO_USED == RATE_CTRL_INDI;
 * a few floats per axis otherwise idle). Configured in angle_rate_controller_init. */
static rate_indi_t s_indi[NUM_AXES];

/** @implements CTRL-RATE-105 gyro-LPF tuning setter. */
bool angle_rate_controller_set_gyro_lpf(uint8_t axis, float rc) {
  if (axis >= NUM_AXES) {
    return false;
  }
  s_gyro_lpf_rc[axis] = (rc > 0.0f) ? rc : 0.0f;
  return true;
}

/** @noreq gyro-LPF getter. */
float angle_rate_controller_get_gyro_lpf(uint8_t axis) {
  return (axis < NUM_AXES) ? s_gyro_lpf_rc[axis] : 0.0f;
}

/** @noreq D-term LPF tuning setter (the D-LPF behaviour is CTRL-PID-101). */
bool angle_rate_controller_set_d_lpf(uint8_t axis, float rc) {
  if (axis >= NUM_AXES) {
    return false;
  }
  /* Single 32-bit field; a concurrent hot-loop read sees old-or-new RC, which
   * is harmless (R8.6: no lock in the rate loop), same as the gain setter. */
  v_pid_set_d_lpf_rc(&angle_rate_controller.pid[axis], rc);
  return true;
}

/** @noreq D-term LPF getter. */
float angle_rate_controller_get_d_lpf(uint8_t axis) {
  return (axis < NUM_AXES) ? angle_rate_controller.pid[axis].d_lpf_rc : 0.0f;
}

/* Per-motor mix signs derived from the airframe geometry (position + spin), so
 * the roll/pitch/yaw -> motor mixing matches whatever motor layout the sim
 * physics / real airframe actually uses, instead of a hardcoded numbering.
 *   out_i = throttle + roll*(-sign y_i) + pitch*(sign x_i) + yaw*(spin_i)
 * Defaults are the on-hardware rig geometry reconciled from
 * firmware/docs/store/rig_tune.json (2026-06-22): pos_x/pos_y give the X-quad roll/pitch
 * mix (FR=M1, RR=M2, RL=M3, FL=M4), and the yaw spin signs give negative (stable)
 * yaw feedback. ONE geometry source (the GCS vehicle / loaded .vveh) drives both
 * the sim physics and this mix, keeping firmware + sim consistent and stable for
 * any quad layout. */
static float s_mix_roll[4]  = {-1.f, -1.f, +1.f, +1.f};  /* -sign(y) */
static float s_mix_pitch[4] = {+1.f, -1.f, -1.f, +1.f};  /*  sign(x) */
static float s_mix_yaw[4]   = {-1.f, +1.f, -1.f, +1.f};  /*  spin    */

/* Control-allocation mixer (firmware/src/control/mixer.c): pseudo-inverse mix +
 * airmode desaturation + clamp + idle floor. Kept in sync with the s_mix_* signs
 * by mixer_sync(). Airmode DISABLED applies the uniform anti-saturation scaler
 * (hold throttle, scale attitude to fit); MIXER_AIRMODE_RP preserves roll/pitch
 * authority instead. */
static mixer_t s_mixer;
static mixer_airmode_t s_airmode = MIXER_AIRMODE_DISABLED;

/* (Re)build the mixer from the current sign arrays + idle floor. The mixer only
 * needs the geometry sign, so we recover valid pos/spin inputs from s_mix_*
 * (their single source of truth) -- they can never drift apart. */
/** @noreq rebuilds the control-allocation mixer from the mix-sign arrays (glue). */
static void mixer_sync(void) {
  float px[4], py[4];
  int sp[4];
  for (int i = 0; i < 4; i++) {
    px[i] = s_mix_pitch[i];                      /* sign(x)            */
    py[i] = -s_mix_roll[i];                      /* sign(y) = -(-sign y) */
    sp[i] = (s_mix_yaw[i] >= 0.0f) ? 1 : -1;     /* spin               */
  }
  mixer_set_geometry(&s_mixer, px, py, sp, 4);
  mixer_set_airmode(&s_mixer, s_airmode);
  mixer_set_idle_floor(&s_mixer, MOTOR_IDLE_FLOOR);
}

/** @noreq derives per-motor mix signs from geometry, then rebuilds the mixer
 *  (glue; the mixing layout itself is owned by mixer_set_geometry / CTRL-MIX-001). */
void angle_rate_controller_set_motor_geometry(const float pos_x[4],
                                              const float pos_y[4],
                                              const int spin[4]) {
  for (int i = 0; i < 4; i++) {
    s_mix_roll[i]  = (pos_y[i] >= 0.0f) ? -1.0f : +1.0f;
    s_mix_pitch[i] = (pos_x[i] >= 0.0f) ? +1.0f : -1.0f;
    s_mix_yaw[i]   = (spin[i] >= 0)     ? +1.0f : -1.0f;
  }
  mixer_sync(); /* rebuild the allocator for the new geometry */
}

/**
 * Parse a CMD_SET_MOTOR_GEOMETRY payload, validating argc/length before reading
 * the 12 float args (COMM-CMD-002), then apply the geometry.
 *
 * @implements COMM-CMD-002
 */
bool angle_rate_controller_apply_geometry_command(const uint8_t *payload,
                                                  uint16_t len) {
  /* payload: [cmd:2][argc:1][x0..x3, y0..y3, spin0..spin3] (12 floats). */
  if (payload == NULL || len < 3) return false;
  uint8_t argc = payload[2];
  if (argc < 12 || len < (uint16_t)argc * 4u + 3u) return false;
  float a[12];
  for (int i = 0; i < 12; i++) v_memcpy(&a[i], &payload[3 + i * 4], 4);
  const float x[4] = {a[0], a[1], a[2], a[3]};
  const float y[4] = {a[4], a[5], a[6], a[7]};
  const int spin[4] = {a[8] >= 0 ? 1 : -1, a[9] >= 0 ? 1 : -1,
                       a[10] >= 0 ? 1 : -1, a[11] >= 0 ? 1 : -1};
  angle_rate_controller_set_motor_geometry(x, y, spin);
  return true;
}

/**
 * Init the rate-loop PIDs from the compiled defaults, then override each axis
 * with any gains / gyro-LPF / D-LPF tune persisted to SD and restored at boot
 * (pid_config_get_*). Also seeds the optional INDI state and builds the mixer.
 *
 * @implements COMM-CMD-003
 */
void angle_rate_controller_init(void) {
  for (int i = 0; i < NUM_AXES; i++) {
    v_pid_init(&angle_rate_controller.pid[i], angle_rate_controller.pid[i].Kp,
               angle_rate_controller.pid[i].Ki, angle_rate_controller.pid[i].Kd,
               angle_rate_controller.pid[i].Kff,
               angle_rate_controller.pid[i].i_max,
               angle_rate_controller.pid[i].d_max,
               angle_rate_controller.pid[i].d_lpf_rc,
               angle_rate_controller.pid[i].out_min,
               angle_rate_controller.pid[i].out_max);

    /* COMM-CMD-003: override compiled defaults with any tune persisted
     * to SD (loaded by pid_config_init() before the scheduler started). */
    float kp, ki, kd, kff;
    if (pid_config_get_rate((uint8_t)i, &kp, &ki, &kd, &kff)) {
      v_pid_set_gains(&angle_rate_controller.pid[i], kp, ki, kd, kff);
    }
    /* Restore any persisted gyro LPF time constant (co-tuned with the gains). */
    float rc;
    if (pid_config_get_gyro_lpf((uint8_t)i, &rc)) {
      s_gyro_lpf_rc[i] = (rc > 0.0f) ? rc : 0.0f;
    }
    /* Restore any persisted D-term LPF time constant (overrides the compiled
     * DEAFULT_*_RATE_D_LPF_RC for this axis). */
    float d_rc;
    if (pid_config_get_d_lpf((uint8_t)i, &d_rc)) {
      v_pid_set_d_lpf_rc(&angle_rate_controller.pid[i], d_rc);
    }
  }

  /* Configure the optional INDI inner loop. Always initialised (cheap) so the
   * RATE_CTRL_ALGO_USED switch is the only thing that selects it; the PID is
   * untouched. Effectiveness b is per-axis (sysid K); k/lpf are shared seeds. */
  const float indi_b[NUM_AXES] = {DEAFULT_ROLL_INDI_B, DEAFULT_PITCH_INDI_B,
                                  DEAFULT_YAW_INDI_B};
  for (int i = 0; i < NUM_AXES; i++) {
    rate_indi_init(&s_indi[i], indi_b[i], DEAFULT_RATE_INDI_K,
                   DEAFULT_RATE_INDI_LPF, angle_rate_controller.pid[i].out_min,
                   angle_rate_controller.pid[i].out_max);
  }

  /* Build the control-allocation mixer from the default geometry. */
  mixer_sync();

  /* Allocate the FFT dynamic-notch banks on the heap (keeps .bss flat). Stays
   * disabled until explicitly enabled, so the gyro stream is untouched here. */
  gyro_notch_init();
}

bool angle_rate_controller_set_gains(uint8_t axis, float kp, float ki, float kd,
                                     float kff) {
  if (axis >= NUM_AXES) {
    return false;
  }
  /* Each field is a 32-bit scalar; a control-loop iteration concurrent
   * with this update may read a one-cycle mix of old/new gains, which is
   * harmless for PID gains (R8.6: no lock in the hot loop). */
  v_pid_set_gains(&angle_rate_controller.pid[axis], kp, ki, kd, kff);
  return true;
}

bool angle_rate_controller_get_gains(uint8_t axis, float *kp, float *ki,
                                     float *kd, float *kff) {
  if (axis >= NUM_AXES) {
    return false;
  }
  const struct PID *p = &angle_rate_controller.pid[axis];
  *kp = p->Kp;
  *ki = p->Ki;
  *kd = p->Kd;
  *kff = p->Kff;
  return true;
}

/**
 * Inner body-rate loop: a drift-free 1 kHz periodic task (CTRL-RATE-001) that
 * drives the per-axis rate PID. On the disarmed->ARMED edge it resets the PID
 * state (CTRL-PID-102); it holds the integrator at zero below
 * RATE_PID_INTEGRATE_THROTTLE (CTRL-PID-103); it ramps PID authority from
 * MIN_ARMED_THROTTLE to PID_FULL_AUTHORITY_THROTTLE (CTRL-ARM-002); and it
 * feeds the motor FIFO only while ARMED/IN_AIR (CTRL-MIX-004). The chirp
 * injection (sysid) and optional INDI inner loop are not yet covered by
 * requirements (see proposed CTRL-SID-* and CTRL-RATE-104).
 *
 * @implements CTRL-RATE-001, CTRL-PID-102, CTRL-PID-103, CTRL-ARM-002, CTRL-MIX-004
 */
void angle_rate_controller_task(void *arg) {
  (void)arg;
  angle_rate_controller_init();
  static bmx160_all_reading_t imu_data = {0};
  static bmx160_all_reading_t prev_imu_data = {0};
  /* Previous sample's acquisition cycle stamp; dt is the delta of these (the
   * true inter-sample interval), not a DWT read at loop time. */
  static motor_outputs_t motor_outputs = {0};
  static angle_controller_outputs_t angle_controller_outputs;
  static angle_controller_outputs_t last_angle_controller_outputs;
  static sys_state_t prev_state = SYSTEM_STATE_UNINITIALIZED;

  /* Throttle threshold below which we hold the rate PIDs in reset.
   * The drone can't actually rotate while it's sitting on the ground
   * with idle / near-idle motors, so the I-term would otherwise wind up
   * against an unsatisfiable rate error and saturate one motor on
   * takeoff. 0.3 is well above MIN_ARMED_THROTTLE (0.1) and below the
   * X3's ~0.55 hover throttle, so the integrator wakes up just before
   * the drone leaves the ground. */
#define RATE_PID_INTEGRATE_THROTTLE 0.3f

  set_motor_ready(true);
  /* Hard-pinned inner-loop rate (INNER_LOOP_FREQ_HZ): the loop runs on a
   * drift-free periodic schedule (task_delay_until), not on IMU-sample arrival.
   * The IMU produces faster than the control rate, so each tick we drain the
   * OVERWRITE control ring to the freshest sample and integrate with a constant
   * dt = INNER_LOOP_DT. If the IMU stalls the loop still ticks (failsafe / motor
   * path stays alive) and reuses the last sample. */
  uint32_t last_wake = v_get_ticks();
  const float dt = INNER_LOOP_DT;
  while (1) {
    task_delay_until(&last_wake, INNER_LOOP_PERIOD_TICKS);

    // Get the freshest IMU data (drain the ring; producer runs faster).
    bool got_sample = false;
    while (imu_queue_control_pop(&imu_data)) {
      prev_imu_data = imu_data;
      got_sample = true;
    }
    if (!got_sample) {
      imu_data = prev_imu_data; // IMU stalled — hold last sample
    }

    // Applying deadband to the gyro data
    for (int i = 0; i < NUM_AXES; i++) {
      if (m_fabsf(imu_data.converted.gyr[i]) < PID_GYRO_DEADBAND) {
        imu_data.converted.gyr[i] = 0;
      }
    }

    // Per-axis gyro low-pass (input filter on the rate measurement). Smooths
    // the P path so higher gains don't limit-cycle around the deadband.
    // rc <= 0 = passthrough. Co-tuned with the PID gains.
    for (int i = 0; i < NUM_AXES; i++) {
      float rc = s_gyro_lpf_rc[i];
      if (rc > 1e-6f && dt > 0.0f) {
        float alpha = dt / (dt + rc);
        s_gyro_lpf_state[i] += alpha * (imu_data.converted.gyr[i] - s_gyro_lpf_state[i]);
        imu_data.converted.gyr[i] = s_gyro_lpf_state[i];
      } else {
        s_gyro_lpf_state[i] = imu_data.converted.gyr[i];
      }
    }

    // FFT-driven dynamic notch on the rate measurement, after the LPF and just
    // before the rate is handed to the PID. Observes the (post-LPF) gyro and,
    // when enabled, notches out the tracked prop peaks; a no-op passthrough
    // until gyro_notch_set_enabled(true). The heavy FFT retune is amortised to
    // one axis per tick by the gyro_notch_service() call below.
    for (int i = 0; i < NUM_AXES; i++) {
      imu_data.converted.gyr[i] =
          gyro_notch_apply((uint8_t)i, imu_data.converted.gyr[i]);
    }
    gyro_notch_service();

    // Get rates from the angle controller
    if (angle_controller_get_outputs(&angle_controller_outputs)) {
      last_angle_controller_outputs = angle_controller_outputs;
    } else {
      angle_controller_outputs = last_angle_controller_outputs;
    }
    float target_rates[NUM_AXES] = {angle_controller_outputs.angle_rates[0],
                                    angle_controller_outputs.angle_rates[1],
                                    angle_controller_outputs.angle_rates[2]};
    float current_rates[NUM_AXES] = {imu_data.converted.gyr[0],
                                     imu_data.converted.gyr[1],
                                     imu_data.converted.gyr[2]};

    /* SYS-ID: inject the chirp excitation. Zero unless a run is active.
     * Two injection points (sysid_inject_mode()):
     *   RATE_SP — add to the rate SETPOINT here, before the PID; the closed loop
     *             tracks it (safe, self-stabilising, closed-loop ID).
     *   U       — leave the setpoint untouched and add to the rate-PID OUTPUT
     *             below, after the PID, i.e. straight onto the plant input
     *             (cleaner near-open-loop excitation; more aggressive).
     * Either way motors move only via the ARMED-gated motor_set_outputs() that
     * runs downstream of the capture, so both modes are safe DISARMED (props off).
     * sysid_inject persists to the post-PID block below for the U-mode add. */
    float sysid_inject[NUM_AXES] = {0.0f, 0.0f, 0.0f};
    sysid_step(dt, angle_controller_outputs.angle_curr, current_rates,
               sysid_inject);
    int sysid_mode = sysid_inject_mode();
    if (sysid_mode == SYSID_INJECT_RATE_SP) {
      for (int i = 0; i < NUM_AXES; i++)
        target_rates[i] += sysid_inject[i];
    }

    /* (B) Hard reset when ENTERING the armed group (disarmed -> ARMED) so no
     * windup carries from the previous arm cycle. The ARMED<->IN_AIR internal
     * transitions (takeoff/touchdown) must NOT reset — they're one continuous
     * flight, and a mid-air reset would dump the rate integrators. */
    sys_state_t state = system_state_get();
    bool armed_now =
        (state == SYSTEM_STATE_ARMED || state == SYSTEM_STATE_IN_AIR);
    bool armed_prev = (prev_state == SYSTEM_STATE_ARMED ||
                       prev_state == SYSTEM_STATE_IN_AIR);
    if (armed_now && !armed_prev) {
      for (int i = 0; i < NUM_AXES; i++) {
        v_pid_reset(&angle_rate_controller.pid[i]);
        rate_indi_reset(&s_indi[i]);    /* re-seed INDI filters/feedback too */
        s_gyro_lpf_state[i] = 0.0f;     /* clear the input filter too */
      }
    }
    prev_state = state;

    /* Clamp target_throttle to [0, 1]. The RC normalize step maps
     * pulse 1000..2000 us to 0..1, but a remote with imperfect
     * calibration / trim can dip below 1000 us at "idle", giving a
     * tiny negative target_throttle. The anti-saturation block below
     * then computes `k = target_throttle / (target_throttle - min_m)`
     * with min_m == target_throttle (since PID outputs are zeroed at
     * low throttle), divides by zero, and poisons all four motor
     * outputs with NaN. NaN propagates through esc_set_throttle (the
     * 0..1 clamp lets NaN through since `NaN < 0` is false), down the
     * PWM FIFO as the literal string "nan", and into the Gazebo
     * wrench bridge -- which then applies NaN force/torque to the
     * airframe and physics explodes. */
    float target_throttle = angle_controller_outputs.throttle;
    if (target_throttle < 0.0f) target_throttle = 0.0f;
    if (target_throttle > 1.0f) target_throttle = 1.0f;

    /* (A) Hold the integrator at zero while throttle is below the
     * gate. This prevents windup-on-the-ground: the drone can't rotate
     * with motors at idle, so a non-zero rate setpoint would otherwise
     * accumulate forever in the I-term and saturate one motor the
     * moment we cross hover. */
    if (target_throttle < RATE_PID_INTEGRATE_THROTTLE) {
      for (int i = 0; i < NUM_AXES; i++)
        v_pid_set_integral(&angle_rate_controller.pid[i], 0.0f);
    }

    // Inner rate loop. Algorithm selected by RATE_CTRL_ALGO_USED (variables.h),
    // mirroring the SF_FILTER_USED estimator switch: a runtime compare on a
    // compile-time constant, so -O2 drops the dead branch (enum values aren't
    // preprocessor-visible, so #if can't be used here). Same per-axis contract
    // either way: (rate_sp, rate_meas, dt) -> normalized u.
    float outputs[NUM_AXES] = {0};
    if (RATE_CTRL_ALGO_USED == RATE_CTRL_INDI) {
      for (int i = 0; i < NUM_AXES; i++) {
        outputs[i] = rate_indi_update(&s_indi[i], target_rates[i],
                                      current_rates[i], dt);
      }
    } else {
      for (int i = 0; i < NUM_AXES; i++) {
        outputs[i] = v_pid_update(&angle_rate_controller.pid[i], target_rates[i],
                                  current_rates[i], 0, dt);
      }
    }

    /* SYS-ID U mode: add the chirp directly onto the control effort u (the plant
     * input), AFTER the inner rate loop (PID or INDI). The captured u below is then
     * the total command actually applied to the mixer — exactly the plant-fit
     * input. Zero on the non-excited axes and when idle, so this is a no-op
     * outside a U-mode run. */
    if (sysid_mode == SYSID_INJECT_U) {
      for (int i = 0; i < NUM_AXES; i++)
        outputs[i] += sysid_inject[i];
    }

    /* SYS-ID capture: record the excited axis's rate-PID OUTPUT u (the control
     * effort the plant fit needs) and the measured gyro rate, decimated to
     * ~500 Hz into RAM; dumped after the run via CMD_SYSID_DUMP. Taken pre-throttle
     * -gating so the chirp response is visible even disarmed; at hover throttle the
     * gating is full-authority, so this equals the command actually applied. */
    sysid_capture(outputs, current_rates);

    /* PID authority ramp. Below MIN_ARMED_THROTTLE the airframe is
     * either disarmed-ish or so lightly throttled that the motors can't
     * meaningfully rotate it; the only effect of a PID correction at
     * that point is to fight whatever ground contact / friction is
     * holding the drone down, which creates a feedback loop with the
     * mahony filter (drone wobbles a degree -> PID asks for big rate ->
     * motors deflect -> drone tips further against the ground
     * constraint -> filter sees more rotation -> ...). Ramp from
     * MIN_ARMED_THROTTLE up to PID_FULL_AUTHORITY_THROTTLE so the loop
     * is fully gated until the pilot is nearly at hover (the SITL X3
     * hovers at ~0.55), at which point the drone is light on the ground
     * or already lifting and the PID actually has authority over attitude. */
    if (target_throttle < MIN_ARMED_THROTTLE) {
      for (int i = 0; i < NUM_AXES; i++) outputs[i] = 0.0f;
    } else if (target_throttle < PID_FULL_AUTHORITY_THROTTLE) {
      float ramp = (target_throttle - MIN_ARMED_THROTTLE) /
                   (PID_FULL_AUTHORITY_THROTTLE - MIN_ARMED_THROTTLE);
      for (int i = 0; i < NUM_AXES; i++) outputs[i] *= ramp;
    }
    /* Control allocation. The mixer (firmware/src/control/mixer.c) owns the
     * per-motor mix (pseudo-inverse of the airframe geometry), saturation
     * handling, the [0,1] clamp, the NaN guard and the idle floor.
     *
     * With airmode DISABLED (the default) the worst-violating PID excursion is
     * scaled so every motor fits in [0,1] without changing the commanded
     * throttle. MIXER_AIRMODE_RP instead preserves roll/pitch authority under
     * saturation (moves collective / sacrifices yaw). The realised differential
     * the INDI block below reads back from these motor commands is the same
     * either way. */
    {
      const float w[MIX_NW] = {outputs[0], outputs[1], outputs[2],
                               target_throttle};
      float m[4];
      mixer_allocate(&s_mixer, w, m, NULL);
      motor_outputs.m1 = m[0];
      motor_outputs.m2 = m[1];
      motor_outputs.m3 = m[2];
      motor_outputs.m4 = m[3];
    }

    /* INDI saturation-aware feedback: hand each axis the differential the motors
     * ACTUALLY delivered after anti-sat clipping, not the demanded u. The mix is
     * orthogonal (sum mix_a*mix_b = 0, sum mix_a^2 = 4), so the realized per-axis
     * command is sum(m_i * mix_a_i)/4 and the common throttle/idle-floor cancels.
     * This is what stops INDI from integrating against thrust it never got — the
     * relay limit cycle it otherwise hits at low throttle. No-op for PID. */
    if (RATE_CTRL_ALGO_USED == RATE_CTRL_INDI) {
      const float m[4] = {motor_outputs.m1, motor_outputs.m2, motor_outputs.m3,
                          motor_outputs.m4};
      float a_roll = 0.0f, a_pitch = 0.0f, a_yaw = 0.0f;
      for (int i = 0; i < 4; i++) {
        a_roll  += m[i] * s_mix_roll[i];
        a_pitch += m[i] * s_mix_pitch[i];
        a_yaw   += m[i] * s_mix_yaw[i];
      }
      rate_indi_set_applied(&s_indi[0], a_roll  * 0.25f);
      rate_indi_set_applied(&s_indi[1], a_pitch * 0.25f);
      rate_indi_set_applied(&s_indi[2], a_yaw   * 0.25f);
    }

    /* Only feed the motor FIFO while the airframe is armed. The
     * motor_task already zero-overrides outputs in non-ARMED states, so
     * whatever it pulls from the queue is discarded. But the queue
     * itself is OVERWRITE policy: a pre-arm asymmetric mix sits in the
     * slot until the next read consumes it. The instant the state
     * machine flips to ARMED, motor_task's "override to 0" path stops
     * triggering and the very next FIFO read serves up the pre-arm
     * value -- so the ESCs get a step kick of (target_throttle +/-
     * PID outputs computed against gyro noise while the drone was
     * disarmed) before angle_rate_controller has even had a chance to
     * push a fresh, reset-PID value. Drone flips before throttle is
     * touched. Pushing only while armed eliminates that stale slot. */
    if (state == SYSTEM_STATE_ARMED || state == SYSTEM_STATE_IN_AIR) {
      motor_set_outputs(motor_outputs);
    }

    control_telemetry_t telemetry = {
        .roll_angle_sp = angle_controller_outputs.angle_sp[0],
        .pitch_angle_sp = angle_controller_outputs.angle_sp[1],
        .yaw_angle_sp = angle_controller_outputs.angle_sp[2],
        .roll_angle_curr = angle_controller_outputs.angle_curr[0],
        .pitch_angle_curr = angle_controller_outputs.angle_curr[1],
        .yaw_angle_curr = angle_controller_outputs.angle_curr[2],
        .roll_rate_sp = target_rates[0],
        .pitch_rate_sp = target_rates[1],
        .yaw_rate_sp = target_rates[2],
        .roll_rate_curr = current_rates[0],
        .pitch_rate_curr = current_rates[1],
        .yaw_rate_curr = current_rates[2],
        .roll_out = outputs[0],
        .pitch_out = outputs[1],
        .yaw_out = outputs[2],
        .thro_out = target_throttle,
        .outer_dt = angle_controller_outputs.dt,
        .inner_dt = dt};
    control_telemetry_queue_push(&telemetry);
    /* CTRL-RATE-101: no v_delay() here — task_delay_until() (above) drives a
     * drift-free periodic schedule at INNER_LOOP_PERIOD_TICKS; the loop drains
     * the freshest IMU sample each tick rather than blocking on queue arrival. */
  }
}
