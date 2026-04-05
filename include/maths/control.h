#ifndef VAYU_CONTROL_H
#define VAYU_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "maths/lpf.h"

// Angle Loop Gains
#define PID_ROLL_ANGLE_KP 1.0f
#define PID_ROLL_ANGLE_KI 0.0f
#define PID_ROLL_ANGLE_KD 0.0f
#define PID_ROLL_ANGLE_I_LIMIT 0.0f

#define PID_PITCH_ANGLE_KP 1.0f
#define PID_PITCH_ANGLE_KI 0.0f
#define PID_PITCH_ANGLE_KD 0.0f
#define PID_PITCH_ANGLE_I_LIMIT 0.0f

#define PID_YAW_ANGLE_KP 1.0f
#define PID_YAW_ANGLE_KI 0.0f
#define PID_YAW_ANGLE_KD 0.0f
#define PID_YAW_ANGLE_I_LIMIT 0.0f

// Rate Loop Gains
#define PID_ROLL_RATE_KP 0.12f
#define PID_ROLL_RATE_KI 0.00f
#define PID_ROLL_RATE_KD 0.00f
#define PID_ROLL_RATE_KD_LPF_ALPHA 0.7f
#define PID_ROLL_RATE_I_LIMIT 0.2f
#define PID_ROLL_RATE_OUT_LIMIT 0.3f
#define PID_ROLL_RATE_EXPO 0.7f

#define PID_PITCH_RATE_KP 0.12f
#define PID_PITCH_RATE_KI 0.00f
#define PID_PITCH_RATE_KD 0.00f
#define PID_PITCH_RATE_KD_LPF_ALPHA 0.7f
#define PID_PITCH_RATE_I_LIMIT 0.2f
#define PID_PITCH_RATE_OUT_LIMIT 0.3f
#define PID_PITCH_RATE_EXPO 0.7f

#define RADIO_AVOID_BAND 10

#define PID_YAW_RATE_KP 0.0f
#define PID_YAW_RATE_KI 0.0f
#define PID_YAW_RATE_KD 0.0f
#define PID_YAW_RATE_I_LIMIT 0.5f
#define PID_YAW_RATE_OUT_LIMIT 0.8f
#define PID_YAW_RATE_EXPO 0.7f

// Control Limits
#define MAX_CONTROL_ANGLE 40.0f  // max tilt in degrees
#define MAX_CONTROL_RATE 40.0f  // max target rate in deg/s
#define MOTOR_MIN_THROTTLE 0.05f // minimum for armed motors

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float i_limit;
  float output_limit;
  lpf_t lpf_d;
} pid_controller_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float throttle;
} control_setpoint_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit);
float pid_calculate(pid_controller_t *pid, float setpoint, float current_value,
                    float dt);
void pid_reset(pid_controller_t *pid);

void control_get_pid_errors(float errors[3]);

void control_init(void);
void control_task(void *args);

#endif // VAYU_CONTROL_H
