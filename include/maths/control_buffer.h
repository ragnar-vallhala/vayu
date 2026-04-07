#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_BUFFER_SIZE 5

typedef struct {
  float motors[4];
} motor_pwm_data_t;

void control_buffer_init(void);

bool motor_queue_push(const motor_pwm_data_t *data);
bool motor_queue_pop(motor_pwm_data_t *out_data);

#endif // VAYU_CONTROL_BUFFER_H
