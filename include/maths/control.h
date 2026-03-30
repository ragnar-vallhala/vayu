#ifndef VAYU_CONTROL_H
#define VAYU_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

// Angle Loop Gains
#define PID_ROLL_ANGLE_KP 3.0f
#define PID_ROLL_ANGLE_KI 0.0f
#define PID_ROLL_ANGLE_KD 0.0f
#define PID_ROLL_ANGLE_I_LIMIT 0.0f

#define PID_PITCH_ANGLE_KP 3.0f
#define PID_PITCH_ANGLE_KI 0.0f
#define PID_PITCH_ANGLE_KD 0.0f
#define PID_PITCH_ANGLE_I_LIMIT 0.0f

// Rate Loop Gains
#define PID_ROLL_RATE_KP 0.08f
#define PID_ROLL_RATE_KI 0.08f
#define PID_ROLL_RATE_KD 0.002f
#define PID_ROLL_RATE_I_LIMIT 1.0f

#define PID_PITCH_RATE_KP PID_ROLL_RATE_KP
#define PID_PITCH_RATE_KI PID_ROLL_RATE_KI
#define PID_PITCH_RATE_KD PID_ROLL_RATE_KD
#define PID_PITCH_RATE_I_LIMIT PID_ROLL_RATE_I_LIMIT

#define PID_YAW_RATE_KP 0.12f
#define PID_YAW_RATE_KI 0.05f
#define PID_YAW_RATE_KD 0.0f
#define PID_YAW_RATE_I_LIMIT 0.4f

// Control Limits
#define MAX_CONTROL_ANGLE 30.0f  // max tilt in degrees
#define MAX_CONTROL_RATE 180.0f  // max target rate in deg/s
#define MOTOR_MIN_THROTTLE 0.05f // minimum for armed motors

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float i_limit;
  float output_limit;
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
