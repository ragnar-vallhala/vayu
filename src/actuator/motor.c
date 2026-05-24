#include "actuator/motor.h"
#include "actuator/esc.h"
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
}

void motor_set_outputs(motor_outputs_t motor_outputs) {
  spsc_write(&motor_angle_rate2motor_queue, &motor_outputs, 1);
}

void motor_task(void *arg) {
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
    if (system_state_get() != SYSTEM_STATE_ARMED) {
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