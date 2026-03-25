#include "maths/control.h"
#include "actuator/esc.h"
#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/serializer.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include <math.h>
// PID Controllers
static pid_controller_t pid_roll_angle, pid_roll_rate;
static pid_controller_t pid_pitch_angle, pid_pitch_rate;
static pid_controller_t pid_yaw_rate;
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

  // Rate Loops (Inner)
  pid_init(&pid_roll_rate, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
           PID_ROLL_RATE_I_LIMIT, 1.0f);
  pid_init(&pid_pitch_rate, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI,
           PID_PITCH_RATE_KD, PID_PITCH_RATE_I_LIMIT, 1.0f);
  pid_init(&pid_yaw_rate, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
           PID_YAW_RATE_I_LIMIT, 1.0f);
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
  }

  static bmx160_all_reading_t imu_data = {0};
  static attitude_t attitude = {0};
  static bmx160_all_reading_t last_valid = {0};

  while (1) {

    static uint32_t last = 0;
    uint32_t n = v_get_ticks();
    float dt = (n - last) / 1000.0f;
    last = n;
    if (dt <= 0.0f || dt > 0.05f) { // reject anything > 50ms as bogus
      last = v_get_ticks();
      v_delay(2);
      continue;
    }

    // 1. Get latest sensor data
    // Use averaged gyro data from the buffer if available
    bool valid = imu_buffer_peek(&imu_data);

    if (!valid) {
      // Fallback: use last valid data OR zero
      imu_data = last_valid;
    } else {
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
    bmx160_get_attitude(&attitude);

    // 2. Get RC setpoints and map to physical units
    float target_roll =
        ((float)rc_channels[0] - 1500.0f) / 500.0f * MAX_CONTROL_ANGLE;
    float target_pitch =
        ((float)rc_channels[1] - 1500.0f) / 500.0f * MAX_CONTROL_ANGLE;
    float throttle = ((float)rc_channels[2] - 1000.0f) / 1000.0f;

    // Clamp throttle
    if (throttle < 0.0f)
      throttle = 0.0f;
    if (throttle > 1.0f)
      throttle = 1.0f;

    // 3. Pose Control (Outer Angle Loop)
    // Target Angle -> Angle PID -> Desired Rate
    float target_rate_roll =
        pid_calculate(&pid_roll_angle, target_roll, attitude.roll, dt);
    float target_rate_pitch =
        pid_calculate(&pid_pitch_angle, target_pitch, attitude.pitch, dt);

    float target_yaw_rate =
        ((float)rc_channels[3] - 1500.0f) / 500.0f * MAX_CONTROL_RATE;

    // 4. Rate Control (Inner Rate Loop)
    float out_roll = pid_calculate(&pid_roll_rate, target_rate_roll,
                                   imu_data.converted.gyr[0], dt);
    float out_pitch = pid_calculate(&pid_pitch_rate, target_rate_pitch,
                                    imu_data.converted.gyr[1], dt);
    float out_yaw = pid_calculate(&pid_yaw_rate, target_yaw_rate,
                                  imu_data.converted.gyr[2], dt);
    
    // // 5. Motor Mixing (Quad-X configuration)
    float m1 = throttle - out_roll - out_pitch + out_yaw;
    float m2 = throttle - out_roll + out_pitch - out_yaw;
    float m3 = throttle + out_roll + out_pitch + out_yaw;
    float m4 = throttle + out_roll - out_pitch - out_yaw;
     if(system_state_get()==SYSTEM_STATE_CALIBRATING || system_state_get()==SYSTEM_STATE_FAILSAFE){
      m1=0;
      m2=0;
      m3=0;
      m4=0;
     }
    float motor_cmds[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; i++) {
      if (motor_cmds[i] < 0.0f)
        motor_cmds[i] = 0.0f;
      if (motor_cmds[i] > 1.0f)
        motor_cmds[i] = 1.0f;
      esc_set_throttle(&motors[i], motor_cmds[i]);
    }

    static uint32_t last_telemetry_time = 0;
    uint32_t now = v_get_ticks();
    if (now - last_telemetry_time >= 20) { // 50 Hz
      if (g_telemetry_channel.handle != NULL) {
        send_packet(&g_telemetry_channel, PACKET_TYPE_MOTOR_TELEMETRY,
                    (uint8_t *)&motor_cmds, sizeof(motor_cmds));
      }
      last_telemetry_time = now;
    }
    v_delay(2);
  }
}
