/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
  /* How much of the last output the ACTUATOR could not deliver, in output
   * units and signed the same way: positive means more positive effort was
   * asked for than arrived. The PID's own out_min/out_max cannot see this --
   * a rate axis saturates at the MIXER, which sums three axes onto the
   * collective and runs out long before any single axis reaches +-1. Fed back
   * by the caller that owns the allocation; 0 means "no limit reported". */
  float sat_excess;
  bool initialized; // initialized flag
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
/* Report what the actuator could not deliver on the previous output, so the
 * next update stops integrating further INTO that limit (it may still unwind
 * out of it). Signed in output units: commanded - realised. */
void v_pid_set_sat_excess(struct PID *pid, float excess);
#endif // VAYU_PID_H
