#include "comm/serializer.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/deserializer.h"
#include "common/hal_types.h"
#include "core/cortex-m4/crc.h"
#include "core/cortex-m4/uart.h"
#include "utils.h"
#include "utils/utils.h"
#include "variables.h"
#include <stdint.h>

static packet_t _rx_uart_pkt_buf[INCOMING_PACKET_BUFFER];
static uint32_t _rx_uart_pkt_drop = 0;
static deserializer_t _uart_recv_state;

static crc_config_t _crc_cfg = {.polynomial =
                                    CRC_POLY_CRC32, // Standard 0x04C11DB7
                                .init_value = 0xFFFFFFFF};
static uint8_t _initialized = 0;
static void init_serializer(void) {
  hal_crc_init(&_crc_cfg);
  deserializer_init(&_uart_recv_state);
}

static uint32_t calculate_crc(byte *payload, uint8_t size) {
  hal_crc_reset();
  uint32_t crc_value = hal_crc_accumulate(payload, size);
  return crc_value;
}

err_t send_packet(channel_t *channel, packet_type_t packet_type, byte *payload,
                  uint8_t payload_size) {
#if ENABLE_BINARY_NAVLINK_PKT == 1
  if (!_initialized) {
    init_serializer();
    _initialized = 1;
  }
  packet_t packet = {};
  packet.sync = SYNC_BYTE;
  packet.protocol_packet_type =
      ((packet_type & 0b1111) << 4) | (PROTOCOL_VERSION & 0b1111);
  packet.length = payload_size;
  packet.device_id = get_device_id();
  packet.timestamp = get_timestamp_unix();
  if (payload != NULL && payload_size > 0)
    v_memcpy(packet.payload, payload, payload_size);

  // Calculate total size of the final transmitted bytes
  // Header size is 8 bytes: sync (1) + type (1) + length (1) + dev_id (1) +
  // timestamp (4)
  uint8_t header_size = 8;
  uint8_t packet_size = header_size + payload_size + sizeof(packet.crc32);

  if (g_comm_mutex) {
    if (v_mutex_lock(g_comm_mutex, 100) != VA_PASS) {
      return ERROR;
    }
  }

  packet.crc32 =
      calculate_crc((uint8_t *)(&packet), header_size + payload_size);

  // The struct has a 256-byte payload buffer, so crc32 sits at the end of that
  // 256 bytes in memory! We can't just pass &packet and packet_size natively
  // because of the struct padding/layout. We must copy the crc immediately
  // after the payload and then send that total contiguous buffer.
  v_memcpy(packet.payload + payload_size, &packet.crc32, sizeof(packet.crc32));

  err_t ret = write_channel(*channel, (uint8_t *)&packet, packet_size);

  if (g_comm_mutex) {
    v_mutex_unlock(g_comm_mutex);
  }

  return ret;
#else
  // No binary navlink packets
  return NONE;
#endif
}

void uart2_packet_recv_callback(void) {
  // 1. Read the SINGLE available byte that triggered the interrupt
  uint8_t b = (uint8_t)uart2_read_char();
  // 2. Feed to non-blocking state machine
  if (deserializer_feed(&_uart_recv_state, b)) {
    // 3. Valid Packet Found! Find a free slot to store it
    int free_index = -1;
    for (int i = 0; i < INCOMING_PACKET_BUFFER; i++) {
      if (_rx_uart_pkt_buf[i].sync == 0) {
        free_index = i;
        break;
      }
    }

    if (free_index != -1) {
      v_memcpy(&_rx_uart_pkt_buf[free_index], &_uart_recv_state.packet,
               sizeof(packet_t));
    } else {
      _rx_uart_pkt_drop++;
    }

    // Note: deserializer_feed already calls deserializer_init on
    // success/failure resets
  }
}

err_t get_next_rx_packet(packet_t *pkt) {
  if (pkt == NULL)
    return USAGE;

  for (int i = 0; i < INCOMING_PACKET_BUFFER; i++) {
    if (_rx_uart_pkt_buf[i].sync == SYNC_BYTE) {
      v_memcpy(pkt, &_rx_uart_pkt_buf[i], sizeof(packet_t));
      _rx_uart_pkt_buf[i].sync = 0; // Mark slot as free
      return NONE;
    }
  }
  return ERROR;
}