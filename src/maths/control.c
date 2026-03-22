#include "maths/control.h"
#include "actuator/esc.h"
#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/serializer.h"
#include "sensor/bmx160.h"
#include "sys/state.h"
#include "utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"

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
  float error = setpoint - current_value;

  // Proportional
  float p_out = pid->kp * error;

  // Integral with anti-windup
  pid->integral += error * dt;
  if (pid->integral > pid->i_limit)
    pid->integral = pid->i_limit;
  else if (pid->integral < -pid->i_limit)
    pid->integral = -pid->i_limit;
  float i_out = pid->ki * pid->integral;

  // Derivative
  float derivative = (error - pid->prev_error) / dt;
  float d_out = pid->kd * derivative;
  pid->prev_error = error;

  float output = p_out + i_out + d_out;

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
  while (1) {
    v_delay(100);
  }
}
