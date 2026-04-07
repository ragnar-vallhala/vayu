#include "maths/control_buffer.h"
#include "structure.h"

#define CONTROL_BUFFER_INTERNAL_CAPACITY (CONTROL_BUFFER_SIZE + 1)

static motor_pwm_data_t _motor_buffer[CONTROL_BUFFER_INTERNAL_CAPACITY];

static spsc_fifo_t _motor_queue;

void control_buffer_init(void) {
  spsc_init(&_motor_queue, _motor_buffer, CONTROL_BUFFER_INTERNAL_CAPACITY,
            sizeof(motor_pwm_data_t));
  spsc_set_policy(&_motor_queue, SPSC_POLICY_OVERWRITE);
}

bool motor_queue_push(const motor_pwm_data_t *data) {
  return spsc_write(&_motor_queue, data, 1) == 1;
}

bool motor_queue_pop(motor_pwm_data_t *out_data) {
  return spsc_read(&_motor_queue, out_data, 1) == 1;
}
