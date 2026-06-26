#ifndef VAYU_PID_H
#define VAYU_PID_H
#include <stdbool.h>

struct PID {
  float Kp, Ki, Kd, Kff;
  float i_max;            // integrator anti-windup clamp
  float d_max;            // derivative anti-windup clamp
  float d_lpf_rc;         // derivative low pass filter cutoff frequency
  float d_filtered;       // filtered derivative
  float integral;         // accumulated I term
  float prev_meas;        // previous measurement (D-on-meas)
  float out_min, out_max; // output saturation
  bool initialized;       // initialized flag
};

void v_pid_init(struct PID *pid, float Kp, float Ki, float Kd, float Kff,
                float i_max, float d_max, float d_lpf_rc, float out_min,
                float out_max);

float v_pid_update(struct PID *pid, float sp, float meas, float sp_dot,
                   float dt);
void v_pid_reset(struct PID *pid);

void v_pid_set_gains(struct PID *pid, float Kp, float Ki, float Kd, float Kff);
void v_pid_set_limits(struct PID *pid, float out_min, float out_max);
void v_pid_set_i_max(struct PID *pid, float i_max);
void v_pid_set_d_lpf_rc(struct PID *pid, float d_lpf_rc);
void v_pid_set_prev_meas(struct PID *pid, float prev_meas);
void v_pid_set_integral(struct PID *pid, float integral);
#endif // VAYU_PID_H
