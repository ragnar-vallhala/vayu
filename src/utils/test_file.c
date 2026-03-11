#include "utils/test_file.h"
#include "actuator/esc.h"
#include "vaios.h"
#include <stdint.h>

#define THROTTLE_VAL 0.3f
void test_task(void *args) {
  (void)args;
  ESC_Handle motors[4];
  esc_init(&motors[0], TIM1, 1, GPIO_PA08);
  esc_init(&motors[1], TIM1, 2, GPIO_PA09);
  esc_init(&motors[2], TIM1, 3, GPIO_PA10);
  esc_init(&motors[3], TIM1, 4, GPIO_PA11);
  esc_arm(&motors[0]);
  esc_arm(&motors[1]);
  esc_arm(&motors[2]);
  esc_arm(&motors[3]);

  v_delay(2000);

  // esc_set_throttle(&motors[0],THROTTLE_VAL);
  // esc_set_throttle(&motors[1],THROTTLE_VAL);
  // esc_set_throttle(&motors[2],THROTTLE_VAL);
  // esc_set_throttle(&motors[3],THROTTLE_VAL);

  hal_pwm_set_duty_cycle(&motors[0].pwm, THROTTLE_VAL);
  hal_pwm_set_duty_cycle(&motors[1].pwm, THROTTLE_VAL);
  hal_pwm_set_duty_cycle(&motors[2].pwm, THROTTLE_VAL);
  hal_pwm_set_duty_cycle(&motors[3].pwm, THROTTLE_VAL);
  while (1)
    v_delay(1000);
}