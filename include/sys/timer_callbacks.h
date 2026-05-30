#ifndef VAYU_TIMER_CALLBACKS_H
#define VAYU_TIMER_CALLBACKS_H

#include "navhal.h"
#include <stdint.h>

// TIM5 is used for timer callbacks
/**
 * @brief Initialize the high-frequency timer callback system.
 * @param freq_hz Desired execution frequency in Hertz.
 */
void timer_callback_init(uint32_t freq_hz);
uint32_t timer_get_callback_frequency(void);

/**
 * @brief Register a function to be called at high frequency.
 * @param fn Function pointer to register.
 * @return 0 on success, -1 if array is full.
 */
int timer_callback_register(void (*fn)(void), uint32_t us_delay);

#endif // !VAYU_TIMER_CALLBACKS_H
