#ifndef VAYU_SERIALIZER_H
#define VAYU_SERIALIZER_H
#include "channel.h"
#include "comm/comm_types.h"
#include "common/hal_types.h"
#include "sys/types.h"
#include <stdint.h>

/* Telemetry-UART RX ISR (registered as the USART6 RX callback). */
void uart2_packet_recv_callback(void);

/* Drain up to `max` raw telemetry-UART RX bytes into `out`, returning the count.
 * The RX ISR mirrors every byte into a lock-free ring so the NavLink v2 parser
 * can run in task context — its handlers apply commands / send frames, which is
 * not ISR-safe. Single consumer only (comm_processor_task). */
uint16_t comm_rx_raw_drain(uint8_t *out, uint16_t max);

#ifdef VAYU_SIM
/* Test-only: push bytes into the RX ring as if the ISR had received them.
 * Sim builds only (see serializer.c). */
void comm_rx_raw_inject(const uint8_t *data, uint16_t n);
#endif
#endif // !VAYU_SERIALIZER_H