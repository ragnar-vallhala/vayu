#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "maths/pid.h"
#include "variables.h"

typedef struct {
  struct PID pid[NUM_AXES];
} angle_controller_t;
typedef struct {
  float angle_rates[NUM_AXES];
  float throttle;
  float angle_sp[NUM_AXES];
  float angle_curr[NUM_AXES];
  float dt;
} angle_controller_outputs_t;

void angle_controller_init(void);
void angle_controller_task(void *arg);
bool angle_controller_get_outputs(angle_controller_outputs_t *outputs);
#endif // ANGLE_CONTROLLER_H