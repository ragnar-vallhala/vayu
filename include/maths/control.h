#ifndef VAYU_CONTROL_H
#define VAYU_CONTROL_H

#include "maths/lpf.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
  float kp;
  float ki;
  float kd;
  float i_limit;
  float out_limit;
  float d_lpf_alpha;
  float expo;
} pid_params_t;

typedef struct __attribute__((packed)) {
  pid_params_t roll_angle;
  pid_params_t pitch_angle;
  pid_params_t yaw_angle;
  pid_params_t roll_rate;
  pid_params_t pitch_rate;
  pid_params_t yaw_rate;
} control_config_t;

extern control_config_t g_control_config;

#define RADIO_AVOID_BAND 10

// Control Limits
#define MAX_CONTROL_ANGLE 40.0f  // max tilt in degrees
#define MAX_CONTROL_RATE 40.0f   // max target rate in deg/s
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

typedef struct {
  float roll_angle_error;
  float pitch_angle_error;
  float yaw_angle_error;
  float roll_rate_error;
  float pitch_rate_error;
  float yaw_rate_error;
  float dt;
  float roll_angle_setpoint;
  float pitch_angle_setpoint;
  float yaw_angle_setpoint;
  float roll_rate_setpoint;
  float pitch_rate_setpoint;
  float yaw_rate_setpoint;
  float roll_output;
  float pitch_output;
  float yaw_output;
  float throttle_output;
} control_loop_data_t;

void pid_init(pid_controller_t *pid, const pid_params_t *params);
float pid_calculate(pid_controller_t *pid, float setpoint, float current_value,
                    float dt);
void pid_reset(pid_controller_t *pid);

void control_get_pid_errors(float errors[3]);

void control_init(void);
void control_task(void *args);

void control_loop_fifo_init(void);
bool control_loop_fifo_push(control_loop_data_t *data);
bool control_loop_fifo_pop(control_loop_data_t *data);
#endif // VAYU_CONTROL_H
