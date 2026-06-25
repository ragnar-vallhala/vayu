/* Host SITL stub for NavHAL family/interrupt_reg.h. The firmware
 * references a handful of IRQn names by symbol when it would normally
 * register handlers with the NVIC. On host they're never fired - we
 * just need the symbols so channel.c and friends compile. */
#ifndef VAYU_SIM_FAMILY_INTERRUPT_REG_H
#define VAYU_SIM_FAMILY_INTERRUPT_REG_H

#include <stdint.h>

enum {
    USART1_IRQn = 37,
    USART2_IRQn = 38,
    USART6_IRQn = 71,

    DMA1_Stream0_IRQn = 11,
    DMA1_Stream1_IRQn = 12,
    DMA1_Stream2_IRQn = 13,
    DMA1_Stream3_IRQn = 14,
    DMA1_Stream4_IRQn = 15,
    DMA1_Stream5_IRQn = 16,
    DMA1_Stream6_IRQn = 17,
    DMA1_Stream7_IRQn = 47,

    DMA2_Stream0_IRQn = 56,
    DMA2_Stream1_IRQn = 57,
    DMA2_Stream2_IRQn = 58,
    DMA2_Stream3_IRQn = 59,
    DMA2_Stream4_IRQn = 60,
    DMA2_Stream5_IRQn = 68,
    DMA2_Stream6_IRQn = 69,
    DMA2_Stream7_IRQn = 70,

    SDIO_IRQn   = 49,

    I2C1_EV_IRQn = 31,
    I2C1_ER_IRQn = 32,
};

/* channel.c calls these directly - they live in NavHAL's compat header on
 * the real port, but we expose them here so the call sites get a
 * declaration when only family/interrupt_reg.h is in scope. */
uint32_t hal_interrupt_disable_global(void);
void     hal_interrupt_enable_global(uint32_t state);

#endif
