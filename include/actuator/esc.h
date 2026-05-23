/**
 * @file esc.h
 * @brief Electronic Speed Controller (ESC) driver using PWM.
 *
 * @copyright © NAVROBOTEC PVT. LTD.
 */

#ifndef ESC_H
#define ESC_H

#include "navhal.h"
#include <stdint.h>

/**
 * @brief ESC handle structure.
 */
typedef struct {
  hal_pwm_handle_t pwm;
  float min_pulse_ms;
  float max_pulse_ms;
  uint32_t frequency;
} ESC_Handle;

/**
 * @brief Initialize an ESC on a specific timer and channel.
 *
 * @param esc Pointer to the ESC handle.
 * @param timer Hardware timer.
 * @param channel PWM channel.
 * @param pin GPIO pin for PWM output.
 */
void esc_init(ESC_Handle *esc, hal_timer_t timer, uint32_t channel,
              hal_gpio_pin_t pin);

/**
 * @brief Arm the ESC (usually involves sending min throttle for a period).
 *
 * @param esc Pointer to the ESC handle.
 */
void esc_arm(ESC_Handle *esc);

/**
 * @brief Disarm the ESC (stops PWM or sends safe signal).
 *
 * @param esc Pointer to the ESC handle.
 */
void esc_disarm(ESC_Handle *esc);

/**
 * @brief Set the throttle level for the ESC.
 *
 * @param esc Pointer to the ESC handle.
 * @param throttle Throttle value from 0.0 to 1.0.
 */
void esc_set_throttle(ESC_Handle *esc, float throttle);

#endif // ESC_H
