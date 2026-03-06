#include "utils/physical_heartbeat.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

void physical_heartbeat(void *args) {
  uint32_t time_period_half =
      _HEARTBEAT_DEFAULT_TIMEPERIOD / 2; // 1000ms by default
  if (args) {
    time_period_half = (*(uint32_t *)args) / 2;
  }
  // clamp to 250ms for a max of 2Hz blink
  time_period_half = time_period_half >= 250 ? time_period_half : 250;
  // Configure pins
  hal_gpio_setmode(_HEARTBEAT_LED_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);
  while (1) {
    hal_gpio_digitalwrite(_HEARTBEAT_LED_PIN, GPIO_HIGH);
    v_delay(time_period_half);
    hal_gpio_digitalwrite(_HEARTBEAT_LED_PIN, GPIO_LOW);
    v_delay(time_period_half);
  }
}