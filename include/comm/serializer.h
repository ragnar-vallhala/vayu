#ifndef VAYU_SERIALIZER_H
#define VAYU_SERIALIZER_H
#include "channel.h"
#include "comm/comm_types.h"
#include "common/hal_types.h"
#include "sys/types.h"
#include <stdint.h>

err_t send_packet(channel_t *channel, packet_type_t packet_type, byte *payload,
                  uint8_t payload_size);
void uart2_packet_recv_callback(void);
err_t get_next_rx_packet(packet_t *pkt);

/* Drain up to `max` raw telemetry-UART RX bytes into `out`, returning the count.
 * The RX ISR mirrors every byte into a lock-free ring (alongside the v1
 * deserializer) so the NavLink v2 parser can run in task context — its handlers
 * apply commands / send frames, which is not ISR-safe. Single consumer only
 * (comm_processor_task). See navlink/INTEGRATION.md (Phase 3 FC RX demux). */
uint16_t comm_rx_raw_drain(uint8_t *out, uint16_t max);
#endif // !VAYU_SERIALIZER_H