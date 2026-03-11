/**
 * @file motor_task.c
 * @brief Task for mapping RC throttle to motors.
 */

#include "actuator/esc.h"
#include "comm/ibus.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>

// Extern from rc_task.c
extern uint16_t rc_channels[IBUS_MAX_CHANNELS];

void motor_task(void *args) {
  (void)args;

  ESC_Handle motors[4];

  // Setup ESCs on TIM1 Channels 1-4 with PA8-PA11 pins
  esc_init(&motors[0], TIM1, 1, GPIO_PA08);
  esc_init(&motors[1], TIM1, 2, GPIO_PA09);
  esc_init(&motors[2], TIM1, 3, GPIO_PA10);
  esc_init(&motors[3], TIM1, 4, GPIO_PA11);

  // Initial arming (ensures ESCs see low throttle)
  for (int i = 0; i < 4; i++) {
    esc_arm(&motors[i]);
    esc_set_throttle(&motors[i], 0.0f);
  }

  while (1) {
    // Channel 3 is index 2 (1-indexed 3)
    uint16_t throttle_raw = rc_channels[2];

    // Map 1000-2000 range to 0.0-1.0
    float throttle = 0.0f;
    if (throttle_raw > 1000) {
      throttle = (float)(throttle_raw - 1000) / 1000.0f;
    }

    if (throttle > 1.0f)
      throttle = 1.0f;
    if (throttle < 0.0f)
      throttle = 0.0f;

    // Apply to all motors
    for (int i = 0; i < 4; i++) {
      esc_set_throttle(&motors[i], throttle);
    }

    // Run at 400Hz (matching ESC update rate)
    v_delay(2);
  }
}
