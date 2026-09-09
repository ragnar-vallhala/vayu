/* Host SITL stub for NavHAL cortex-m4 port_gpio.h.
 * Inline hot-path GPIO accessors that target STM32 BSRR / IDR don't apply
 * to the host build - degrade them to no-ops. */
#ifndef VAYU_SIM_NAVHAL_PORT_GPIO_H
#define VAYU_SIM_NAVHAL_PORT_GPIO_H

#include "common/hal_gpio.h"

static inline void hal_gpio_write(hal_gpio_pin_t pin, hal_gpio_state_t state) {
  (void)pin;
  (void)state;
}

static inline hal_gpio_state_t hal_gpio_read(hal_gpio_pin_t pin) {
  (void)pin;
  return HAL_GPIO_LOW;
}

static inline void hal_gpio_toggle(hal_gpio_pin_t pin) { (void)pin; }

#endif /* VAYU_SIM_NAVHAL_PORT_GPIO_H */
