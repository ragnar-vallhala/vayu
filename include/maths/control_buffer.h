#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include "maths/control.h"
#include <stdbool.h>
#include <stdint.h>

#define CONTROL_BUFFER_SIZE 5

typedef struct {
  float motors[4];
} motor_pwm_data_t;

void control_buffer_init(void);

bool motor_queue_push(const motor_pwm_data_t *data);
bool motor_queue_pop(motor_pwm_data_t *out_data);

bool pid_config_t2c_push(const control_config_t *data);
bool pid_config_t2c_pop(control_config_t *out_data);

bool pid_config_c2t_push(const control_config_t *data);
bool pid_config_c2t_pop(control_config_t *out_data);

#endif // VAYU_CONTROL_BUFFER_H
