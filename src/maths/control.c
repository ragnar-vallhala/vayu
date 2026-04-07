#include "maths/control.h"
#include "actuator/esc.h"
#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "comm/serializer.h"
#include "core/cortex-m4/dwt.h"
#include "maths/control_buffer.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include <math.h>
#include <stdint.h>

// PID Controllers
static pid_controller_t pid_roll_angle, pid_roll_rate;
static pid_controller_t pid_pitch_angle, pid_pitch_rate;
static pid_controller_t pid_yaw_angle, pid_yaw_rate;
static ESC_Handle motors[4];

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->i_limit = i_limit;
  pid->output_limit = out_limit;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  lpf_init(&pid->lpf_d, PID_ROLL_RATE_KD_LPF_ALPHA);
}
static float expo(float x, float expo) {
  return x * (1.0f - expo) + x * x * x * expo;
}
void pid_reset(pid_controller_t *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float current_value,
                    float dt) {

  if (!isfinite(setpoint) || !isfinite(current_value) || !isfinite(dt) ||
      dt <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint - current_value;

  float p_out = pid->kp * error;

  pid->integral += error * dt;
  if (pid->integral > pid->i_limit)
    pid->integral = pid->i_limit;
  else if (pid->integral < -pid->i_limit)
    pid->integral = -pid->i_limit;

  float i_out = pid->ki * pid->integral;

  float derivative = (error - pid->prev_error) / dt;
  derivative = lpf_apply(&pid->lpf_d, derivative);
  //  CRITICAL CLAMP
  if (!isfinite(derivative) || fabsf(derivative) > 10000.0f) {
    derivative = 0.0f;
  }

  float d_out = pid->kd * derivative;
  pid->prev_error = error;

  float output = p_out + i_out + d_out;

  //  CRITICAL SANITY
  if (!isfinite(output)) {
    return 0.0f;
  }

  if (output > pid->output_limit)
    output = pid->output_limit;
  else if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}

void control_init(void) {
  // Angle Loops (Outer)
  pid_init(&pid_roll_angle, PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI,
           PID_ROLL_ANGLE_KD, PID_ROLL_ANGLE_I_LIMIT, MAX_CONTROL_RATE);
  pid_init(&pid_pitch_angle, PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI,
           PID_PITCH_ANGLE_KD, PID_PITCH_ANGLE_I_LIMIT, MAX_CONTROL_RATE);
  pid_init(&pid_yaw_angle, PID_YAW_ANGLE_KP, PID_YAW_ANGLE_KI, PID_YAW_ANGLE_KD,
           PID_YAW_ANGLE_I_LIMIT, MAX_CONTROL_RATE);

  // Rate Loops (Inner)
  pid_init(&pid_roll_rate, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
           PID_ROLL_RATE_I_LIMIT, PID_ROLL_RATE_OUT_LIMIT);
  pid_init(&pid_pitch_rate, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI,
           PID_PITCH_RATE_KD, PID_PITCH_RATE_I_LIMIT, PID_PITCH_RATE_OUT_LIMIT);
  pid_init(&pid_yaw_rate, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
           PID_YAW_RATE_I_LIMIT, PID_YAW_RATE_OUT_LIMIT);
}

void control_task(void *args) {
  (void)args;

  control_init();

  // Setup ESCs (Mapping from motor_task.c)
  esc_init(&motors[0], TIM1, 1, GPIO_PA08); // Motor 1
  esc_init(&motors[1], TIM1, 2, GPIO_PA09); // Motor 2
  esc_init(&motors[2], TIM1, 3, GPIO_PA10); // Motor 3
  esc_init(&motors[3], TIM1, 4, GPIO_PA11); // Motor 4

  for (int i = 0; i < 4; i++) {
    esc_arm(&motors[i]);
    v_delay(4);
  }
  v_delay(100);

  static bmx160_all_reading_t imu_data = {0};
  static attitude_t attitude = {0};
  static attitude_t last_attitude = {0};
  static bmx160_all_reading_t last_valid = {0};
  static ibus_data_t rc_data;
  static uint16_t rc_channels[IBUS_MAX_CHANNELS];

  int count = 0;
  uint32_t last = dwt_get_cycles();
  while (1) {
    count++;
    uint32_t n = dwt_get_cycles();
    float dt = ((float)(n - last)) / (float)SYS_CLOCK_FREQ;
    last = n;
    if (dt <= 1e-6f || dt > 0.05f) { // reject anything > 50ms as bogus
      last = dwt_get_cycles();
      // vayu_log("Rejected control loop dt: %f\n", dt);
      v_delay(1);
      continue;
    }

    // 1. Get latest sensor data
    // Use averaged gyro data from the buffer if available

    if (!imu_queue_control_pop(&imu_data)) {
      // Fallback: use last valid data OR zero
      imu_data = last_valid;
    } else {
      // vayu_log("Failed to pop IMU data in control loop");
      // Save last good sample
      last_valid = imu_data;
    }

    //  Sanitize gyro (CRITICAL)
    for (int i = 0; i < 3; i++) {
      if (!isfinite(imu_data.converted.gyr[i]) ||
          fabsf(imu_data.converted.gyr[i]) > 2000.0f) {
        imu_data.converted.gyr[i] = 0.0f;
      }
    }
    // Get current attitude (updated by fusion task)
    if (attitude_queue_control_pop(&attitude)) {
      last_attitude = attitude;
    } else {
      attitude = last_attitude;
    }

    // 2. Get RC setpoints and map to physical units
    if (rc_queue_control_pop(&rc_data)) {
      v_memcpy(rc_channels, rc_data.channels,
               sizeof(rc_channels)); // Dangerous memcopy from struct
    }
    float throttle = ((float)rc_channels[2] - 1000.0f) / 1000.0f;

    if (system_state_get() == SYSTEM_STATE_ARMED &&
        throttle < MOTOR_MIN_THROTTLE) {
      throttle = MOTOR_MIN_THROTTLE;
    }

    // Clamp throttle
    if (throttle < 0.0f)
      throttle = 0.0f;
    if (throttle > 1.0f)
      throttle = 1.0f;

    // 3. Map RC sticks to Target Angles (Roll/Pitch) or Rate (Stick)
    float target_angle_roll =
        (expo(((float)rc_channels[0] - 1500.0f) / 500.0f, PID_ROLL_RATE_EXPO)) *
        MAX_CONTROL_ANGLE;

    float target_angle_pitch = (expo(((float)rc_channels[1] - 1500.0f) / 500.0f,
                                     PID_PITCH_RATE_EXPO)) *
                               MAX_CONTROL_ANGLE;

    // For Angle Mode, Yaw is typically still rate controlled by the pilot.
    float target_rate_yaw_stick =
        -(expo(((float)rc_channels[3] - 1500.0f) / 500.0f, PID_YAW_RATE_EXPO)) *
        MAX_CONTROL_RATE;

    // 4. Outer Loop (Angle Control)
    // Convert angle error to target rate
    float target_rate_roll =
        pid_calculate(&pid_roll_angle, target_angle_roll, attitude.roll, dt);
    float target_rate_pitch = pid_calculate(
        &pid_pitch_angle, target_angle_pitch, -attitude.pitch, dt);

    float target_rate_yaw = target_rate_yaw_stick;

    // 5. Inner Loop (Rate Control)
    float out_roll = pid_calculate(&pid_roll_rate, target_rate_roll,
                                   imu_data.converted.gyr[0], dt);
    float out_pitch = pid_calculate(&pid_pitch_rate, target_rate_pitch,
                                    imu_data.converted.gyr[1], dt);
    float out_yaw = pid_calculate(&pid_yaw_rate, target_rate_yaw,
                                  imu_data.converted.gyr[2], dt);

    if (system_state_get() == SYSTEM_STATE_ARMED) {
      if (attitude.roll > 30 || attitude.roll < -30 || attitude.pitch > 30 ||
          attitude.pitch < -30) {
        system_state_set(SYSTEM_STATE_FAILSAFE);
      }
    }
    // Your layout:
    // Front Left  = M4
    // Front Right = M1
    // Rear Left   = M3
    // Rear Right  = M2
    // M1 = Front Right
    if (throttle <= 0.06f) {
      out_roll = 0;
      out_pitch = 0;
      out_yaw = 0;
    }
    float m1 = throttle - out_roll - out_pitch - out_yaw;

    // M2 = Rear Right
    float m2 = throttle - out_roll + out_pitch + out_yaw;

    // M3 = Rear Left
    float m3 = throttle + out_roll + out_pitch - out_yaw;

    // M4 = Front Left
    float m4 = throttle + out_roll - out_pitch + out_yaw;

    if (system_state_get() != SYSTEM_STATE_ARMED) {
      m1 = 0;
      m2 = 0;
      m3 = 0;
      m4 = 0;
    }
    float motor_cmds[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; i++) {
      if (motor_cmds[i] < 0.0f)
        motor_cmds[i] = 0.0f;
      if (motor_cmds[i] > 1.0f)
        motor_cmds[i] = 1.0f;
      esc_set_throttle(&motors[i], motor_cmds[i]);
    }

    // Push motor PWMs and PID errors to SPSC queues for telemetry
    motor_pwm_data_t m_data;
    v_memcpy(m_data.motors, motor_cmds, sizeof(m_data.motors));
    motor_queue_push(&m_data);

    pid_error_data_t e_data = {.errors = {pid_roll_angle.prev_error,
                                          pid_pitch_angle.prev_error,
                                          pid_yaw_angle.prev_error}};
    pid_error_queue_push(&e_data);
    v_delay(1);
  }
}