#ifndef VAYU_MOTOR_H
#define VAYU_MOTOR_H
#include "structure.h"
#include <stdint.h>
#define NUM_MOTORS 4

typedef struct {
  float m1;
  float m2;
  float m3;
  float m4;
} motor_outputs_t;
#define MOTOR_QUEUE_SIZE 4

void motor_init(void);
void set_motor_ready(bool ready);
bool get_motor_ready(void);
void motor_set_outputs(motor_outputs_t motor_outputs);
void motor_task(void *arg);
bool motor_telemetry_queue_pop(motor_outputs_t *out_data);
#endif // VAYU_MOTOR_H