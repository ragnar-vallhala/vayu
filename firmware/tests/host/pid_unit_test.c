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
/* pid_unit_test.c — host unit test for the rate/attitude PID core (pid.c).
 *
 * Pure function under test: v_pid_update(sp, meas, sp_dot, dt). No HAL/RTOS —
 * links only pid.c + maths_interface.c. Proves the control-critical behaviours:
 * P/I/D/FF terms, integral clamp + anti-windup freeze, derivative-on-measurement
 * (no derivative kick), the D-term LPF, output saturation, and dt/ reset guards.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/pid_unit_test.c src/control/pid.c \
 *       src/maths/maths_interface.c -lm -o /tmp/pt && /tmp/pt
 */
#include "control/pid.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}
static int near(float a, float b, float tol) { return fabsf(a - b) < tol; }

/* generous limits / no D-LPF unless a test sets them */
static void mk(struct PID *p, float kp, float ki, float kd, float kff) {
  v_pid_init(p, kp, ki, kd, kff, /*i_max*/ 1e6f, /*d_max*/ 1e6f,
             /*d_lpf_rc*/ 0.0f, /*out_min*/ -1e6f, /*out_max*/ 1e6f);
}

int main(void) {
  /* 1. Pure proportional: out = Kp*(sp-meas). */
  printf("Test 1: proportional term\n");
  {
    struct PID p;
    mk(&p, 2.0f, 0, 0, 0);
    float u = v_pid_update(&p, 3.0f, 1.0f, 0, 0.01f); /* error 2 -> 4 */
    check("out == Kp*error", near(u, 4.0f, 1e-4f));
  }

  /* 2. Integral accumulates error*dt, then clamps at +/-i_max. */
  printf("Test 2: integral accumulation + i_max clamp\n");
  {
    struct PID p;
    v_pid_init(&p, 0, 1.0f, 0, 0, /*i_max*/ 5.0f, 1e6f, 0, -1e6f, 1e6f);
    float u = 0;
    for (int i = 0; i < 3; i++)
      u = v_pid_update(&p, 1.0f, 0.0f, 0, 1.0f);
    check("I = Ki*error*dt*3 = 3", near(u, 3.0f, 1e-4f));
    for (int i = 0; i < 100; i++)
      u = v_pid_update(&p, 1.0f, 0.0f, 0, 1.0f);
    check("I clamps at i_max = 5", near(u, 5.0f, 1e-4f));
  }

  /* 3. Anti-windup: integrator FROZEN while P+I saturates the output. */
  printf("Test 3: anti-windup integrator freeze on saturation\n");
  {
    struct PID p;
    v_pid_init(&p, 1.0f, 1.0f, 0, 0, /*i_max*/ 1e6f, 1e6f, 0,
               /*out_min*/ -1.0f, /*out_max*/ 1.0f);
    for (int i = 0; i < 50; i++)
      v_pid_update(&p, 5.0f, 0.0f, 0, 1.0f);
    /* P=5 alone exceeds out_max=1, so output_pi is always saturated and the
     * integrator must never have accumulated. */
    check("integral stays 0 while saturated", near(p.integral, 0.0f, 1e-6f));
  }

  /* 4. Derivative-on-measurement: a SETPOINT step causes no derivative kick. */
  printf("Test 4: no derivative kick on setpoint change\n");
  {
    struct PID p;
    mk(&p, 0, 0, 1.0f, 0);
    v_pid_update(&p, 0.0f, 0.0f, 0, 0.1f);            /* init: prev_meas=0 */
    float u = v_pid_update(&p, 10.0f, 0.0f, 0, 0.1f); /* sp jumps, meas same */
    check("D == 0 when only sp changes", near(u, 0.0f, 1e-4f));
  }

  /* 5. Derivative responds to MEASUREMENT rate: D = -Kd*(dmeas)/dt. */
  printf("Test 5: derivative on measurement\n");
  {
    struct PID p;
    mk(&p, 0, 0, 1.0f, 0);
    v_pid_update(&p, 0.0f, 0.0f, 0, 0.1f);           /* init prev_meas=0 */
    float u = v_pid_update(&p, 0.0f, 1.0f, 0, 0.1f); /* meas 0->1 over 0.1s */
    check("D == -Kd*1/0.1 == -10", near(u, -10.0f, 1e-3f));
  }

  /* 6. D-term LPF: alpha = dt/(dt+rc); rc==dt -> alpha 0.5 halves a fresh step. */
  printf("Test 6: derivative LPF smoothing\n");
  {
    struct PID p;
    v_pid_init(&p, 0, 0, 1.0f, 0, 1e6f, 1e6f, /*d_lpf_rc*/ 0.1f, -1e6f, 1e6f);
    v_pid_update(&p, 0.0f, 0.0f, 0, 0.1f);           /* init, d_filtered=0 */
    float u = v_pid_update(&p, 0.0f, 1.0f, 0, 0.1f); /* D_raw=-10, alpha=0.5 */
    check("filtered D == 0.5*(-10) == -5", near(u, -5.0f, 1e-3f));
  }

  /* 7. Output saturation clamps to [out_min, out_max]. */
  printf("Test 7: output saturation\n");
  {
    struct PID p;
    v_pid_init(&p, 100.0f, 0, 0, 0, 1e6f, 1e6f, 0, /*min*/ -10.0f,
               /*max*/ 10.0f);
    float hi = v_pid_update(&p, 1.0f, 0.0f, 0, 0.01f); /* P=100 -> clamp 10 */
    float lo =
        v_pid_update(&p, -1.0f, 0.0f, 0, 0.01f); /* P=-100 -> clamp -10 */
    check("clamps to out_max", near(hi, 10.0f, 1e-4f));
    check("clamps to out_min", near(lo, -10.0f, 1e-4f));
  }

  /* 8. Feedforward: FF = Kff*sp_dot. */
  printf("Test 8: feedforward\n");
  {
    struct PID p;
    mk(&p, 0, 0, 0, 2.0f);
    float u = v_pid_update(&p, 0.0f, 0.0f, 3.0f, 0.01f);
    check("out == Kff*sp_dot == 6", near(u, 6.0f, 1e-4f));
  }

  /* 9. Guards: dt<=0 returns 0; reset zeroes state. */
  printf("Test 9: dt guard + reset\n");
  {
    struct PID p;
    mk(&p, 1.0f, 1.0f, 1.0f, 0);
    check("dt<=1e-6 returns 0",
          near(v_pid_update(&p, 1, 0, 0, 0.0f), 0.0f, 1e-9f));
    for (int i = 0; i < 5; i++)
      v_pid_update(&p, 1.0f, 0.0f, 0, 0.1f);
    v_pid_reset(&p);
    check("reset zeroes integral", near(p.integral, 0.0f, 1e-9f));
    check("reset clears initialized", p.initialized == false);
  }

  /* 10. setters: gains, limits (incl. reversed order), i_max, integral clamp,
   *     d_lpf_rc, prev_meas. */
  printf("Test 10: parameter setters\n");
  {
    struct PID p;
    mk(&p, 1.0f, 0, 0, 0);
    v_pid_set_gains(&p, 4.0f, 0, 0, 0);
    check("set_gains applies new Kp",
          near(v_pid_update(&p, 1.0f, 0.0f, 0, 0.01f), 4.0f, 1e-4f));

    v_pid_set_limits(&p, /*min*/ 5.0f, /*max*/ -5.0f); /* reversed -> sorted */
    check("set_limits sorts min<max", p.out_min == -5.0f && p.out_max == 5.0f);

    v_pid_set_i_max(&p, 2.0f);
    v_pid_set_integral(&p, 100.0f);
    check("set_integral clamps to i_max", near(p.integral, 2.0f, 1e-6f));
    v_pid_set_integral(&p, -100.0f);
    check("set_integral clamps to -i_max", near(p.integral, -2.0f, 1e-6f));

    v_pid_set_d_lpf_rc(&p, 0.05f);
    check("set_d_lpf_rc stores rc", near(p.d_lpf_rc, 0.05f, 1e-6f));
    v_pid_set_d_lpf_rc(&p, -1.0f); /* negative -> 0 */
    check("set_d_lpf_rc rejects negative", near(p.d_lpf_rc, 0.0f, 1e-6f));

    v_pid_set_prev_meas(&p, 1.5f);
    check("set_prev_meas stores + de-inits",
          near(p.prev_meas, 1.5f, 1e-6f) && p.initialized == false);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  /* N. Actuator-saturation anti-windup. The PID's own out_min/out_max is not
   * the only limit: on a rate axis authority runs out at the MIXER, which sums
   * three axes onto the collective and saturates while each PID is nowhere
   * near its own rail. The caller that owns the allocation reports the
   * shortfall, and integration must stop in THAT direction only. PX4 does the
   * same from its control allocator (rate_control.cpp, saturation_positive/
   * negative clamping the rate error); ArduPilot from the motors library
   * (AC_PID::update_i with motors.limit.roll/pitch/yaw). */
  printf("Test 13: anti-windup against the actuator limit\n");
  {
    struct PID p;
    v_pid_init(&p, 0, 1.0f, 0, 0, /*i_max*/ 100.0f, 1e6f, 0, -1e6f, 1e6f);
    /* A sustained positive error the airframe cannot null. Unreported, the
     * integrator winds -- this is the behaviour measured on the aircraft. */
    for (int k = 0; k < 100; k++)
      (void)v_pid_update(&p, 1.0f, 0.0f, 0, 0.01f);
    float wound = p.integral;
    check("unreported saturation still winds up (the old behaviour)",
          wound > 0.9f);

    /* Now report that the actuator could not deliver the positive demand. */
    v_pid_reset(&p);
    for (int k = 0; k < 100; k++) {
      v_pid_set_sat_excess(&p, +0.5f); /* asked for more + than arrived */
      (void)v_pid_update(&p, 1.0f, 0.0f, 0, 0.01f);
    }
    check("reported saturation stops the wind-up",
          near(p.integral, 0.0f, 1e-6f));

    /* Unwinding out of the limit must still be allowed, or it could never
     * recover once the error reverses. */
    v_pid_reset(&p);
    v_pid_set_integral(&p, 0.5f);
    for (int k = 0; k < 50; k++) {
      v_pid_set_sat_excess(&p, +0.5f); /* still saturated positive */
      (void)v_pid_update(&p, 0.0f, 1.0f, 0, 0.01f); /* error now NEGATIVE */
    }
    check("but unwinding out of the limit is still allowed", p.integral < 0.4f);

    /* Saturation on the opposite side must not block a positive error. */
    v_pid_reset(&p);
    for (int k = 0; k < 50; k++) {
      v_pid_set_sat_excess(&p, -0.5f);              /* saturated NEGATIVE */
      (void)v_pid_update(&p, 1.0f, 0.0f, 0, 0.01f); /* error POSITIVE */
    }
    check("the opposite-side limit does not block", p.integral > 0.4f);
  }

  return fails ? 1 : 0;
}
