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
#endif // !VAYU_SERIALIZER_H