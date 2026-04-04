#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "core/cortex-m4/gpio.h"
#include "core/cortex-m4/uart.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/gpio_types.h"
#include "utils/types.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
// State tracking for toggling (since reading output pins is unreliable)
static uint8_t _blue_led_state = 0;
static uint8_t _green_led_state = 0;
static uint8_t _red_led_state = 0;
static uint8_t _buzzer_state = 0;

static inline void _toggle_pin(hal_gpio_pin pin) {
  if (pin == _BLUE_LED_PIN) {
    _blue_led_state = !_blue_led_state;
    hal_gpio_digitalwrite(pin, _blue_led_state ? GPIO_HIGH : GPIO_LOW);
  } else if (pin == _GREEN_LED_PIN) {
    _green_led_state = !_green_led_state;
    hal_gpio_digitalwrite(pin, _green_led_state ? GPIO_HIGH : GPIO_LOW);
  } else if (pin == _RED_LED_PIN) {
    _red_led_state = !_red_led_state;
    hal_gpio_digitalwrite(pin, _red_led_state ? GPIO_HIGH : GPIO_LOW);
  } else if (pin == _BUZZER_PIN) {
    _buzzer_state = !_buzzer_state;
    hal_gpio_digitalwrite(pin, _buzzer_state ? GPIO_HIGH : GPIO_LOW);
  }
}
static inline void _heartbeat_peripheral_init(void) {
  hal_gpio_setmode(_BLUE_LED_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);
  hal_gpio_setmode(_GREEN_LED_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);
  hal_gpio_setmode(_RED_LED_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);
  hal_gpio_setmode(_BUZZER_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);
}

static inline void _system_init(void) {
  static uint8_t _first_time = 1;
  if (_first_time) {
    _first_time = 0;
    hal_gpio_digitalwrite(_BUZZER_PIN, GPIO_HIGH);
    v_delay(100);
    hal_gpio_digitalwrite(_BUZZER_PIN, GPIO_LOW);
  }
  _toggle_pin(_BLUE_LED_PIN);
}

static inline void _system_standby(void) { _toggle_pin(_GREEN_LED_PIN); }

static inline void _system_prearm(void) {
  _toggle_pin(_GREEN_LED_PIN);
  _toggle_pin(_BLUE_LED_PIN);
}

static inline void _system_armed(void) {
  _toggle_pin(_GREEN_LED_PIN);
  hal_gpio_digitalwrite(_RED_LED_PIN, GPIO_HIGH);
}

static inline void _system_in_air(void) {
  _toggle_pin(_GREEN_LED_PIN);
  _toggle_pin(_RED_LED_PIN);
}

static inline void _system_failsafe(void) {
  uint32_t boot_flags = (uint32_t)system_boot_check_state_get();

  // Master Failsafe Blink (Red + Buzzer)
  _toggle_pin(_RED_LED_PIN);
  static uint8_t buzz_cnt = 0;
  if (++buzz_cnt % 2 == 0) {
    _toggle_pin(_BUZZER_PIN);
  }

  // Diagnostic: Solid Blue = Clock Mismatch
  if (boot_flags & BOOT_CHECK_SYSTEM_CLOCK_CHECK_FAIL) {
    hal_gpio_digitalwrite(_BLUE_LED_PIN, GPIO_HIGH);
  } else {
    hal_gpio_digitalwrite(_BLUE_LED_PIN, GPIO_LOW);
  }

  // Diagnostic: Solid Green = SD Card Failure
  if (boot_flags & BOOT_CHECK_SD_CARD_CHECK_FAIL) {
    hal_gpio_digitalwrite(_GREEN_LED_PIN, GPIO_HIGH);
  } else {
    hal_gpio_digitalwrite(_GREEN_LED_PIN, GPIO_LOW);
  }
}

static inline void _system_terminated(void) {
  hal_gpio_digitalwrite(_RED_LED_PIN, GPIO_HIGH);
  hal_gpio_digitalwrite(_BUZZER_PIN, GPIO_HIGH);
}

static inline void _run_heartbeat(channel_t *channel, uint32_t period) {
  static uint32_t last_time = 0;
  uint32_t current_time = v_get_ticks();
  if (current_time - last_time < period) {
    return;
  }
  last_time = current_time;


  static sys_state_t last_state = SYSTEM_STATE_UNINITIALIZED;
  sys_state_t current_state = system_state_get();

  if (current_state != last_state) {
    // Clear all LEDs on transition to ensure a clean slate for the new state
    hal_gpio_digitalwrite(_BLUE_LED_PIN, GPIO_LOW);
    hal_gpio_digitalwrite(_GREEN_LED_PIN, GPIO_LOW);
    hal_gpio_digitalwrite(_RED_LED_PIN, GPIO_LOW);
    _blue_led_state = 0;
    _green_led_state = 0;
    _red_led_state = 0;
    last_state = current_state;
  }

  switch (current_state) {
  case SYSTEM_STATE_UNINITIALIZED:
    // Do nothing
    break;
  case SYSTEM_STATE_INIT:
    _system_init();
    break;
  case SYSTEM_STATE_STANDBY:
    _system_standby();
    break;
  case SYSTEM_STATE_PREARM:
    _system_prearm();
    break;
  case SYSTEM_STATE_ARMED:
    _system_armed();
    break;
  case SYSTEM_STATE_IN_AIR:
    _system_in_air();
    break;
  case SYSTEM_STATE_FAILSAFE:
    _system_failsafe();
    break;
  case SYSTEM_STATE_TERMINATED:
    _system_terminated();
    break;
  case SYSTEM_STATE_CALIBRATING:
    _toggle_pin(_BLUE_LED_PIN);
    _toggle_pin(_GREEN_LED_PIN);
    break;
  default:
    break;
  }
}
void heartbeat_task(void *args) {

  // Configure Physical Heartbeat
  _heartbeat_peripheral_init();
  while (g_telemetry_channel.handle == NULL) {
    v_delay(10);
  }
  uint32_t period = _HEARTBEAT_DEFAULT_TIMEPERIOD;
  if (args) {
    period = *(uint32_t *)args;
  }
  // clamp to 250ms for a max of 2Hz blink
  period = period >= 250 ? period : 250;

  while (1) {
    _run_heartbeat(&g_telemetry_channel, period);
    v_delay(20);
  }
}