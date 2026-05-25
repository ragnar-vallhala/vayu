#include "control/angle_rate_controller.h"
#include "actuator/motor.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "control/angle_controller.h"
#include "maths/control_buffer.h"
#include "maths/pid.h"
#include "navhal.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
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

static inline float get_dt(void) {
  static uint32_t last_time = 0;
  uint32_t current_time = hal_cycle_counter_get();
  float dt = (float)(current_time - last_time) / SYS_CLOCK_FREQ;
  last_time = current_time;
  return dt;
}
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
  }
}

void angle_rate_controller_task(void *arg) {
  angle_rate_controller_init();
  static bmx160_all_reading_t imu_data = {0};
  static bmx160_all_reading_t prev_imu_data = {0};
  static float prev_target_rates[NUM_AXES] = {0};
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
  while (1) {
    // Getting all the data

    // Get IMU data
    if (imu_queue_control_pop(&imu_data)) {
      prev_imu_data = imu_data;
    } else {
      imu_data = prev_imu_data;
    }

    float dt = get_dt();

    // Applying deadband to the gyro data
    for (int i = 0; i < NUM_AXES; i++) {
      if (m_fabsf(imu_data.converted.gyr[i]) < PID_GYRO_DEADBAND) {
        imu_data.converted.gyr[i] = 0;
      }
    }

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

    /* (B) Hard reset on every STANDBY -> ARMED transition. Carries no
     * windup from the previous arm cycle into the new one. */
    sys_state_t state = system_state_get();
    if (state == SYSTEM_STATE_ARMED && prev_state != SYSTEM_STATE_ARMED) {
      for (int i = 0; i < NUM_AXES; i++)
        v_pid_reset(&angle_rate_controller.pid[i]);
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

    // Apply PID to each axis
    float outputs[NUM_AXES] = {0};
    for (int i = 0; i < NUM_AXES; i++) {
      float dot_sp = (target_rates[i] - prev_target_rates[i]) / dt;
      outputs[i] = v_pid_update(&angle_rate_controller.pid[i], target_rates[i],
                                current_rates[i], 0, dt);
      prev_target_rates[i] = target_rates[i];
    }

    /* PID authority ramp. Below MIN_ARMED_THROTTLE the airframe is
     * either disarmed-ish or so lightly throttled that the motors can't
     * meaningfully rotate it; the only effect of a PID correction at
     * that point is to fight whatever ground contact / friction is
     * holding the drone down, which creates a feedback loop with the
     * mahony filter (drone wobbles a degree -> PID asks for big rate ->
     * motors deflect -> drone tips further against the ground
     * constraint -> filter sees more rotation -> ...). The original
     * piecewise that only ramped between 0 and MIN_ARMED_THROTTLE left
     * the PID at full authority for any pilot throttle above 10%, which
     * is well below the X3's ~0.55 hover point in sim. Ramp from
     * MIN_ARMED_THROTTLE up to PID_FULL_AUTHORITY_THROTTLE so the loop
     * is fully gated until the pilot is nearly at hover, at which point
     * the drone is light on the ground or already lifting and the PID
     * actually has authority over attitude. */
    if (target_throttle < MIN_ARMED_THROTTLE) {
      for (int i = 0; i < NUM_AXES; i++) outputs[i] = 0.0f;
    } else if (target_throttle < PID_FULL_AUTHORITY_THROTTLE) {
      float ramp = (target_throttle - MIN_ARMED_THROTTLE) /
                   (PID_FULL_AUTHORITY_THROTTLE - MIN_ARMED_THROTTLE);
      for (int i = 0; i < NUM_AXES; i++) outputs[i] *= ramp;
    }
    // Your layout:
    // Front Left  = M4
    // Front Right = M1
    // Rear Left   = M3
    // Rear Right  = M2
    // Motor               Throttle         Roll           Pitch          Yaw
    motor_outputs.m1 = target_throttle - outputs[0] + outputs[1] + outputs[2];
    motor_outputs.m2 = target_throttle - outputs[0] - outputs[1] - outputs[2];
    motor_outputs.m3 = target_throttle + outputs[0] - outputs[1] + outputs[2];
    motor_outputs.m4 = target_throttle + outputs[0] + outputs[1] - outputs[2];

    // Saturation handling: scale the PID differential (deviation from
    // target_throttle) so every motor fits in [0, 1] WITHOUT changing
    // the pilot's commanded throttle.
    //
    // The previous "shift-all-up-by-|min|" pattern silently added thrust
    // the pilot never asked for: e.g. at target_throttle=0.14 with a
    // PID asking for a big roll torque, m4 would come out at -0.30; the
    // shift then added +0.30 to all four motors, average thrust jumped
    // from 14% to 44%, drone took off uncommanded ("shot up") with the
    // residual differential still tilting it ("rolled down"). The
    // subsequent divide-by-max for positives further re-scaled, but the
    // total energy bump from the lift step had already happened.
    //
    // Correct anti-saturation: find the worst-violating PID excursion
    // and scale ALL PID outputs by the same factor (<=1) so the worst
    // motor sits exactly at the limit (0 or 1) while throttle stays
    // intact. Authority over attitude is reduced when limits bite, but
    // the pilot keeps the throttle they asked for.
    {
      float min_m = motor_outputs.m1, max_m = motor_outputs.m1;
      if (motor_outputs.m2 < min_m) min_m = motor_outputs.m2;
      if (motor_outputs.m3 < min_m) min_m = motor_outputs.m3;
      if (motor_outputs.m4 < min_m) min_m = motor_outputs.m4;
      if (motor_outputs.m2 > max_m) max_m = motor_outputs.m2;
      if (motor_outputs.m3 > max_m) max_m = motor_outputs.m3;
      if (motor_outputs.m4 > max_m) max_m = motor_outputs.m4;

      float scale = 1.0f;
      if (min_m < 0.0f) {
        // need to shrink (target_throttle - min_m) to (target_throttle - 0)
        float k = (target_throttle) / (target_throttle - min_m);
        if (k < scale) scale = k;
      }
      if (max_m > 1.0f) {
        float k = (1.0f - target_throttle) / (max_m - target_throttle);
        if (k < scale) scale = k;
      }
      if (scale < 1.0f) {
        motor_outputs.m1 = target_throttle + scale * (motor_outputs.m1 - target_throttle);
        motor_outputs.m2 = target_throttle + scale * (motor_outputs.m2 - target_throttle);
        motor_outputs.m3 = target_throttle + scale * (motor_outputs.m3 - target_throttle);
        motor_outputs.m4 = target_throttle + scale * (motor_outputs.m4 - target_throttle);
      }
      // Final clip in case throttle itself is out of range (shouldn't be
      // - normalized RC is [0, 1] - but cheap insurance).
      if (motor_outputs.m1 < 0) motor_outputs.m1 = 0;
      if (motor_outputs.m2 < 0) motor_outputs.m2 = 0;
      if (motor_outputs.m3 < 0) motor_outputs.m3 = 0;
      if (motor_outputs.m4 < 0) motor_outputs.m4 = 0;
      if (motor_outputs.m1 > 1) motor_outputs.m1 = 1;
      if (motor_outputs.m2 > 1) motor_outputs.m2 = 1;
      if (motor_outputs.m3 > 1) motor_outputs.m3 = 1;
      if (motor_outputs.m4 > 1) motor_outputs.m4 = 1;
      /* NaN guard. The clamps above use ordered comparisons (`< 0`,
       * `> 1`), which return false for NaN -- so a stray NaN slips
       * through unchanged. Force NaN/Inf to a safe 0. `x != x` is the
       * standard NaN check; (x > -INF) is false for NaN too, but the
       * self-comparison is portable across compilers without needing
       * math.h's isnan. */
      if (motor_outputs.m1 != motor_outputs.m1) motor_outputs.m1 = 0;
      if (motor_outputs.m2 != motor_outputs.m2) motor_outputs.m2 = 0;
      if (motor_outputs.m3 != motor_outputs.m3) motor_outputs.m3 = 0;
      if (motor_outputs.m4 != motor_outputs.m4) motor_outputs.m4 = 0;

      /* Idle thrust floor. Every motor is held at >= MOTOR_IDLE_FLOOR
       * while armed. Two reasons:
       *   1. It matches what a real ESC does in modern flight stacks
       *      ("MOTOR_STOP=false"): the props keep spinning slowly while
       *      armed, so the next throttle command doesn't have to cold-
       *      start the motor.
       *   2. In SITL it's how we signal "armed-and-alive" to the
       *      bridge. With the floor at 0 the FIFO produces
       *      indistinguishable motors=0 in two cases (disarmed, and
       *      armed-at-idle-throttle); the bridge's gravity-
       *      cancellation logic needs to tell them apart so it knows
       *      whether to hold the drone in mid-air or let it fall onto
       *      the skid. motor_task still zeros the outputs in non-ARMED
       *      states, so the floor only takes effect when armed. */
      if (motor_outputs.m1 < MOTOR_IDLE_FLOOR) motor_outputs.m1 = MOTOR_IDLE_FLOOR;
      if (motor_outputs.m2 < MOTOR_IDLE_FLOOR) motor_outputs.m2 = MOTOR_IDLE_FLOOR;
      if (motor_outputs.m3 < MOTOR_IDLE_FLOOR) motor_outputs.m3 = MOTOR_IDLE_FLOOR;
      if (motor_outputs.m4 < MOTOR_IDLE_FLOOR) motor_outputs.m4 = MOTOR_IDLE_FLOOR;
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
    if (state == SYSTEM_STATE_ARMED) {
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

    v_delay(1);
  }
}