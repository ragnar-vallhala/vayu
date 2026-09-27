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
#include "vayu_board.h"
#include "actuator/motor.h"
#include "actuator/esc_calib.h"
#include "driver/esc.h"
#include "structure.h"
#include "sys/state.h"
#include "vaios.h"

static spsc_fifo_t motor_angle_rate2motor_queue;
static motor_outputs_t motor_outputs_buffer[MOTOR_QUEUE_SIZE];
static spsc_fifo_t motor_telemetry_queue;
static motor_outputs_t motor_telemetry_buffer[MOTOR_QUEUE_SIZE];
static ESC_Handle motors[NUM_MOTORS];
static bool motor_ready = false;
void set_motor_ready(bool ready) { motor_ready = ready; }
bool get_motor_ready(void) { return motor_ready; }
void motor_init(void) {
  spsc_init(&motor_angle_rate2motor_queue, motor_outputs_buffer,
            MOTOR_QUEUE_SIZE, sizeof(motor_outputs_t));
  spsc_set_policy(&motor_angle_rate2motor_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&motor_telemetry_queue, motor_telemetry_buffer, MOTOR_QUEUE_SIZE,
            sizeof(motor_outputs_t));
  spsc_set_policy(&motor_telemetry_queue, SPSC_POLICY_OVERWRITE);
  // Setup ESCs
  /* The shared timer is configured ONCE, by its owner, before any channel
   * asks for anything (F7). */
  esc_group_init(BOARD_ESC_TIMER, VAYU_ESC_PWM_FREQ);

  esc_init(&motors[0], 1, BOARD_ESC_M1_PIN, BOARD_ESC_AF); // Motor 1
  esc_init(&motors[1], 2, BOARD_ESC_M2_PIN, BOARD_ESC_AF); // Motor 2
  esc_init(&motors[2], 3, BOARD_ESC_M3_PIN, BOARD_ESC_AF); // Motor 3
  esc_init(&motors[3], 4, BOARD_ESC_M4_PIN, BOARD_ESC_AF); // Motor 4

  for (int i = 0; i < NUM_MOTORS; i++) {
    esc_arm(&motors[i]);
    v_delay(4);
  }
  v_delay(100);
}

void motor_set_outputs(motor_outputs_t motor_outputs) {
  spsc_write(&motor_angle_rate2motor_queue, &motor_outputs, 1);
}

void motor_task(void *arg) {
  (void)arg;
  motor_init();
  static motor_outputs_t motor_outputs;
  static motor_outputs_t prev_motor_outputs;
  while (1) {
    if (!get_motor_ready()) {
      v_delay(3);
      continue;
    }
    if (!spsc_read(&motor_angle_rate2motor_queue, &motor_outputs, 1)) {
      motor_outputs = prev_motor_outputs;
    }
    /* Motors may spin while ARMED *or* IN_AIR — IN_AIR is armed-and-flying, not a
     * disarm. Anything else (STANDBY/FAILSAFE/...) forces them to zero. */
    sys_state_t mstate = system_state_get();

    if (mstate == SYSTEM_STATE_ESC_CALIB) {
      /* ESC endpoint calibration: every motor gets the same endpoint, from the
       * calibration state machine rather than the mixer. This is the ONLY
       * state besides ARMED/IN_AIR in which an output may leave zero, and the
       * mixer is bypassed entirely so no attitude term can perturb an endpoint
       * the ESC is trying to learn. */
      const float lvl = esc_calib_output();
      motor_outputs.m1 = lvl;
      motor_outputs.m2 = lvl;
      motor_outputs.m3 = lvl;
      motor_outputs.m4 = lvl;
    } else if (mstate != SYSTEM_STATE_ARMED && mstate != SYSTEM_STATE_IN_AIR) {
      motor_outputs.m1 = 0;
      motor_outputs.m2 = 0;
      motor_outputs.m3 = 0;
      motor_outputs.m4 = 0;
    }
    prev_motor_outputs = motor_outputs;
    esc_set_throttle(&motors[0], motor_outputs.m1);
    esc_set_throttle(&motors[1], motor_outputs.m2);
    esc_set_throttle(&motors[2], motor_outputs.m3);
    esc_set_throttle(&motors[3], motor_outputs.m4);
    spsc_write(&motor_telemetry_queue, &motor_outputs, 1);
    v_delay(2);
  }
}

bool motor_telemetry_queue_pop(motor_outputs_t *out_data) {
  return spsc_read(&motor_telemetry_queue, out_data, 1) == 1;
}