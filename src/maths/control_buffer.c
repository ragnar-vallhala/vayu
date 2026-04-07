#include "maths/control_buffer.h"
#include "structure.h"

#define CONTROL_BUFFER_INTERNAL_CAPACITY (CONTROL_BUFFER_SIZE + 1)

static motor_pwm_data_t _motor_buffer[CONTROL_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _motor_queue;

static control_config_t _t2c_buffer[CONTROL_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _t2c_queue;

static control_config_t _c2t_buffer[CONTROL_BUFFER_INTERNAL_CAPACITY];
static spsc_fifo_t _c2t_queue;

void control_buffer_init(void) {
  spsc_init(&_motor_queue, _motor_buffer, CONTROL_BUFFER_INTERNAL_CAPACITY,
            sizeof(motor_pwm_data_t));
  spsc_set_policy(&_motor_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_t2c_queue, _t2c_buffer, CONTROL_BUFFER_INTERNAL_CAPACITY,
            sizeof(control_config_t));
  spsc_set_policy(&_t2c_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_c2t_queue, _c2t_buffer, CONTROL_BUFFER_INTERNAL_CAPACITY,
            sizeof(control_config_t));
  spsc_set_policy(&_c2t_queue, SPSC_POLICY_OVERWRITE);
}

bool motor_queue_push(const motor_pwm_data_t *data) {
  return spsc_write(&_motor_queue, data, 1) == 1;
}

bool motor_queue_pop(motor_pwm_data_t *out_data) {
  return spsc_read(&_motor_queue, out_data, 1) == 1;
}

bool pid_config_t2c_push(const control_config_t *data) {
  return spsc_write(&_t2c_queue, data, 1) == 1;
}

bool pid_config_t2c_pop(control_config_t *out_data) {
  return spsc_read(&_t2c_queue, out_data, 1) == 1;
}

bool pid_config_c2t_push(const control_config_t *data) {
  return spsc_write(&_c2t_queue, data, 1) == 1;
}

bool pid_config_c2t_pop(control_config_t *out_data) {
  return spsc_read(&_c2t_queue, out_data, 1) == 1;
}
