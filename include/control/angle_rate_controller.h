#ifndef VAYU_ANGLE_RATE_CONTROLLER_H
#define VAYU_ANGLE_RATE_CONTROLLER_H

#include "control/pid.h"
#include "variables.h"
#include <stdbool.h>
#include <stdint.h>


typedef struct {
  struct PID pid[NUM_AXES];
} AngleRateController;

typedef enum {
  NORMALIZED_RC2ANGLE_RATE_LINEAR, // Linear mode
  NORMALIZED_RC2ANGLE_RATE_CUBIC,  // Cubic mode
} normalized_rc2angle_rate_mode_t;

void angle_rate_controller_init(void);
void angle_rate_controller_task(void *arg);

/**
 * @brief Set the live rate-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true on apply.
 * @implements COMM-CMD-003
 */
bool angle_rate_controller_set_gains(uint8_t axis, float kp, float ki,
                                     float kd, float kff);

/**
 * @brief Read the live rate-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true and fills out-params on ok.
 * @implements COMM-CMD-003
 */
bool angle_rate_controller_get_gains(uint8_t axis, float *kp, float *ki,
                                     float *kd, float *kff);

#endif // VAYU_ANGLE_RATE_CONTROLLER_H