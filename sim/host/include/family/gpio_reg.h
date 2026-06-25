/* Host SITL stub for NavHAL's family/gpio_reg.h (cortex-m4 STM32 register
 * file). The host build never dereferences GPIO registers, so this only
 * needs to satisfy the #include. */
#ifndef VAYU_SIM_GPIO_REG_H
#define VAYU_SIM_GPIO_REG_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIO_GET_PORT(pin) ((GPIO_TypeDef *)0)
#define GPIO_GET_PIN(pin)  ((uint32_t)(pin) & 0xF)

#endif /* VAYU_SIM_GPIO_REG_H */
