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
#include "comm/serializer.h"
#include "navhal.h" /* hal_uart_read_char, HAL_UART_6 */
#include <stdint.h>

/* Telemetry-UART RX path for NavLink v2 (encoded in navlink_tx.c, decoded by
 * navlink_router.c). This TU is the RX byte pipe: a lock-free SPSC ring the ISR
 * mirrors every received byte into, drained in task context by
 * comm_processor_task (the v2 parser's handlers apply commands / send frames and
 * are not ISR-safe). Size is a power of two so the mask wraps cheaply. */
#define RX_RAW_RING_SZ 512u
static volatile uint8_t _rx_raw_buf[RX_RAW_RING_SZ];
static volatile uint16_t _rx_raw_head; /* producer (ISR) */
static volatile uint16_t _rx_raw_tail; /* consumer (task) */

/** @noreq lock-free RX byte-ring producer (ISR->task transport glue) */
static inline void rx_raw_push(uint8_t b) {
  uint16_t h = _rx_raw_head;
  uint16_t nh = (uint16_t)((h + 1u) & (RX_RAW_RING_SZ - 1u));
  if (nh != _rx_raw_tail) { /* drop on full (consumer fell behind) */
    _rx_raw_buf[h] = b;
    _rx_raw_head = nh;
  }
}

/** @noreq RX byte-ring drain (transport glue for the v2 parser) */
uint16_t comm_rx_raw_drain(uint8_t *out, uint16_t max) {
  uint16_t n = 0;
  uint16_t t = _rx_raw_tail;
  while (n < max && t != _rx_raw_head) {
    out[n++] = _rx_raw_buf[t];
    t = (uint16_t)((t + 1u) & (RX_RAW_RING_SZ - 1u));
  }
  _rx_raw_tail = t;
  return n;
}

#ifdef VAYU_SIM
/* Test-only RX injection seam. The host navhal keeps its received byte in a
 * file-static that uart2_packet_recv_callback() reads through the HAL, so a
 * host test has no way to feed the parser without a pty and a sleep. This
 * pushes bytes straight into the same ring the ISR fills, making the router
 * deterministically testable. Compiled ONLY under VAYU_SIM — it is not present
 * in firmware builds.
 *
 * @noreq test seam (sim builds only) */
void comm_rx_raw_inject(const uint8_t *data, uint16_t n) {
  for (uint16_t i = 0; i < n; i++)
    rx_raw_push(data[i]);
}
#endif /* VAYU_SIM */

/* Telemetry-UART (USART6) RX ISR: read the byte that triggered the interrupt
 * and mirror it into the raw ring for the v2 parser. */
/** @noreq telemetry-UART RX ISR: mirrors the received byte into the ring */
void uart2_packet_recv_callback(void) {
  uint8_t b = (uint8_t)hal_uart_read_char(HAL_UART_6);
  rx_raw_push(b);
}
