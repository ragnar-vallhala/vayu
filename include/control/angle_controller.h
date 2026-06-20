#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "control/pid.h"
#include "variables.h"
#include <stdbool.h>
#include <stdint.h>

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

/**
 * @brief Latest commanded throttle (normalised 0..1), non-destructive.
 *
 * The outputs FIFO is consumed by the rate controller, so observers that must
 * not steal samples (e.g. the takeoff/landing detector) read the throttle here.
 * O(1) volatile load; 0 until the first control loop runs.
 */
float angle_controller_last_throttle(void);

/**
 * @brief Set the live angle-PID gains for one axis (0..NUM_AXES-1).
 * @return false if axis is out of range; true on apply.
 * @implements COMM-CMD-003
 */
bool angle_controller_set_gains(uint8_t axis, float kp, float ki, float kd,
                                float kff);
#endif // ANGLE_CONTROLLER_H