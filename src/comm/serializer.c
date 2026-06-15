#include "comm/serializer.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/deserializer.h"
#include "navhal.h"
#include "utils.h"
#include "sys/sys_utils.h"
#include "variables.h"
#include <stdint.h>

static packet_t _rx_uart_pkt_buf[INCOMING_PACKET_BUFFER];
static uint32_t _rx_uart_pkt_drop = 0;
static deserializer_t _uart_recv_state;

/* Lock-free SPSC ring mirroring every RX byte for the NavLink v2 parser (drained
 * in task context by comm_processor_task). Producer: the RX ISR. Consumer: the
 * comm task. Size is a power of two so the mask wraps cheaply. */
#define RX_RAW_RING_SZ 512u
static volatile uint8_t _rx_raw_buf[RX_RAW_RING_SZ];
static volatile uint16_t _rx_raw_head; /* producer (ISR) */
static volatile uint16_t _rx_raw_tail; /* consumer (task) */

static inline void rx_raw_push(uint8_t b) {
  uint16_t h = _rx_raw_head;
  uint16_t nh = (uint16_t)((h + 1u) & (RX_RAW_RING_SZ - 1u));
  if (nh != _rx_raw_tail) { /* drop on full (consumer fell behind) */
    _rx_raw_buf[h] = b;
    _rx_raw_head = nh;
  }
}

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

static uint8_t _initialized = 0;
static void init_serializer(void) { deserializer_init(&_uart_recv_state); }

static uint32_t calculate_crc(byte *payload, uint8_t size) {
  return utils_try_compute_crc32(payload, size);
}

err_t send_packet(channel_t *channel, packet_type_t packet_type, byte *payload,
                  uint8_t payload_size) {
#if ENABLE_BINARY_NAVLINK_PKT == 1
  if (!_initialized) {
    init_serializer();
    _initialized = 1;
  }
  packet_t packet = {0};
  packet.sync = SYNC_BYTE;
  packet.protocol_packet_type =
      ((packet_type & 0xF) << 4) | (PROTOCOL_VERSION & 0xF);
  packet.length = payload_size;
  packet.device_id = get_device_id();
  packet.timestamp = get_timestamp_unix();
  if (payload != NULL && payload_size > 0)
    v_memcpy(packet.payload, payload, payload_size);

  // Calculate total size of the final transmitted bytes
  // Header size is 8 bytes: sync (1) + type (1) + length (1) + dev_id (1) +
  // timestamp (4)
  uint8_t header_size = 8;
  uint8_t packet_size = (uint8_t)(header_size + payload_size + sizeof(packet.crc32));

  // if (g_comm_mutex) {
  //   if (v_mutex_lock(g_comm_mutex, 0) != VA_PASS) {
  //     return ERROR;
  //   }
  // }
  // for (volatile uint32_t i = 0; i < 1000; i++) {

  // }

  packet.crc32 =
      calculate_crc((uint8_t *)(&packet), header_size + payload_size);

  // The struct has a 256-byte payload buffer, so crc32 sits at the end of
  // that
  // 256 bytes in memory! We can't just pass &packet and packet_size natively
  // because of the struct padding/layout. We must copy the crc immediately
  // after the payload and then send that total contiguous buffer.
  v_memcpy(packet.payload + payload_size, &packet.crc32, sizeof(packet.crc32));

  err_t ret = write_channel(*channel, (uint8_t *)&packet, packet_size);

  // if (g_comm_mutex) {
  //   v_mutex_unlock(g_comm_mutex);
  // }
  return ret;
#else
  // No binary navlink packets
  return NONE;
#endif
}

void uart2_packet_recv_callback(void) {
  // 1. Read the SINGLE available byte that triggered the interrupt. Must match
  // the telemetry channel's UART (USART6, PC6/PC7) — hal_uart_read_char()
  // busy-waits on that peripheral's RXNE.
  uint8_t b = (uint8_t)hal_uart_read_char(HAL_UART_6);
  // Mirror the byte to the NavLink v2 raw ring (drained in task context by
  // comm_processor_task). The v1 deserializer below is unchanged; v1 and v2
  // frames coexist on the stream (navlink/INTEGRATION.md, Phase 3).
  rx_raw_push(b);
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
