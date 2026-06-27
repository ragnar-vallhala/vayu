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

/* Telemetry-UART (USART6) RX ISR: read the byte that triggered the interrupt
 * and mirror it into the raw ring for the v2 parser. */
/** @noreq telemetry-UART RX ISR: mirrors the received byte into the ring */
void uart2_packet_recv_callback(void) {
  uint8_t b = (uint8_t)hal_uart_read_char(HAL_UART_6);
  rx_raw_push(b);
}
