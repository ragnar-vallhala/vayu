#ifndef VAYU_SIM_NAVHAL_PORT_INTERRUPT_H
#define VAYU_SIM_NAVHAL_PORT_INTERRUPT_H

/* hal_irq_t would normally come from the cortex-m4 family/interrupt_reg.h
 * (IRQn_Type enum). For host SITL we don't fire real IRQs; just give the
 * type a definition so the shimmed hal_interrupt_* function signatures
 * type-check. */
typedef int hal_irq_t;

#endif
