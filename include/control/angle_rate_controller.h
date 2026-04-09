#ifndef VAYU_ANGLE_RATE_CONTROLLER_H
#define VAYU_ANGLE_RATE_CONTROLLER_H

#include "maths/pid.h"
#include "variables.h"


typedef struct {
  struct PID pid[NUM_AXES];
} AngleRateController;

typedef enum {
  NORMALIZED_RC2ANGLE_RATE_LINEAR, // Linear mode
  NORMALIZED_RC2ANGLE_RATE_CUBIC,  // Cubic mode
} normalized_rc2angle_rate_mode_t;

void angle_rate_controller_init(void);
void angle_rate_controller_task(void *arg);

#endif // VAYU_ANGLE_RATE_CONTROLLER_H