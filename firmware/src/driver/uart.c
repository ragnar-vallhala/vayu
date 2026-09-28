/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* See driver/uart.h. A thin pass-through to NavHAL, with two jobs of its own:
 * it casts vuart_t to the HAL's instance type in one place, and it claims
 * every interrupt vector through the registry before attaching to it. */
#include "driver/uart.h"

#include "navhal.h"
#include "sys/irq_registry.h"

static inline hal_uart_t _u(vuart_t u) { return (hal_uart_t)u; }

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_init(vuart_t u, uint32_t baud) {
  hal_uart_config_t cfg = {.baudrate = baud};
  return hal_uart_init(_u(u), &cfg) == HAL_OK ? VAYU_OK : VAYU_ERR_FAULT;
}

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_attach_rx(vuart_t u, uint32_t vector, void (*cb)(void),
                              const char *owner) {
  if (cb == NULL) {
    return VAYU_ERR_INVALID;
  }
  irq_registry_claim(vector, owner);
  hal_interrupt_attach_callback((hal_irq_t)vector, cb);
  /* RX on, TX off: transmission is DMA or a polled write, never an IRQ. */
  hal_uart_enable_interrupt(_u(u), 1, 0);
  return VAYU_OK;
}

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_detach_rx(vuart_t u, uint32_t vector, const char *owner) {
  (void)u;
  /* Mask first. Detaching the callback while the vector is still enabled
   * leaves an interrupt pointing at nothing, so a failed mask must abort the
   * whole detach rather than carry on. */
  if (hal_interrupt_disable((hal_irq_t)vector) != HAL_OK) {
    return VAYU_ERR_FAULT;
  }
  hal_interrupt_detach_callback((hal_irq_t)vector);
  irq_registry_release(vector, owner);
  return VAYU_OK;
}

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_attach_tx_dma(uint32_t vector, void (*cb)(void),
                                  const char *owner) {
  if (cb == NULL) {
    return VAYU_ERR_INVALID;
  }
  irq_registry_claim(vector, owner);
  hal_interrupt_attach_callback((hal_irq_t)vector, cb);
  hal_interrupt_enable((hal_irq_t)vector);
  return VAYU_OK;
}

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_write_dma(vuart_t u, const uint8_t *data, uint16_t n) {
  return hal_uart_write_dma(_u(u), data, n) == HAL_OK ? VAYU_OK
                                                      : VAYU_ERR_FAULT;
}

/** @noreq thin HAL wrapper. */
void vuart_write_char(vuart_t u, char c) { hal_uart_write_char(_u(u), c); }

/** @noreq thin HAL wrapper. */
char vuart_read_char(vuart_t u) { return hal_uart_read_char(_u(u)); }

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_rx_dma_start(vuart_t u, uint8_t *buf, uint16_t n,
                                 void (*on_idle)(void)) {
  if (buf == NULL || n == 0) {
    return VAYU_ERR_INVALID;
  }
  if (hal_uart_init_dma_rx(_u(u), buf, n) != HAL_OK) {
    return VAYU_ERR_FAULT;
  }
  if (on_idle != NULL) {
    hal_uart_attach_idle_callback(_u(u), on_idle);
  }
  return VAYU_OK;
}

/** @noreq thin HAL wrapper. */
vayu_status_t vuart_rx_dma_index(vuart_t u, uint16_t *out_index) {
  if (out_index == NULL) {
    return VAYU_ERR_INVALID;
  }
  return hal_uart_dma_rx_index(_u(u), out_index) == HAL_OK ? VAYU_OK
                                                           : VAYU_ERR_FAULT;
}
