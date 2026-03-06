#include "comm/serializer.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "common/hal_types.h"
#include "core/cortex-m4/crc.h"
#include "utils.h"
#include "utils/utils.h"
#include <stdint.h>

static crc_config_t _crc_cfg = {.polynomial =
                                    CRC_POLY_CRC32, // Standard 0x04C11DB7
                                .init_value = 0xFFFFFFFF};
static uint8_t _initialized = 0;
static void init_serializer(void) { hal_crc_init(&_crc_cfg); }

static uint32_t calculate_crc(byte *payload, uint8_t size) {
  hal_crc_reset();
  uint32_t crc_value = hal_crc_accumulate(payload, size);
  return crc_value;
}

err_t send_packet(channel_t *channel, packet_type_t packet_type, byte *payload,
                  uint8_t payload_size) {
  if (!_initialized)
    init_serializer();
  packet_t packet = {};
  packet.sync = SYNC_BYTE;
  packet.protocol_packet_type =
      ((packet_type & 0b1111) << 4) | (PROTOCOL_VERSION & 0b1111);
  packet.length = payload_size;
  packet.device_id = get_device_id();
  packet.timestamp = get_timestamp();
  v_memcpy(packet.payload, payload, payload_size);

  // Calculate total size of the final transmitted bytes
  // Header size is 8 bytes: sync (1) + type (1) + length (1) + dev_id (1) +
  // timestamp (4)
  uint8_t header_size = 8;
  uint8_t packet_size = header_size + payload_size + sizeof(packet.crc32);

  packet.crc32 =
      calculate_crc((uint8_t *)(&packet), header_size + payload_size);

  // The struct has a 256-byte payload buffer, so crc32 sits at the end of that
  // 256 bytes in memory! We can't just pass &packet and packet_size natively
  // because of the struct padding/layout. We must copy the crc immediately
  // after the payload and then send that total contiguous buffer.
  v_memcpy(packet.payload + payload_size, &packet.crc32, sizeof(packet.crc32));

  return write_channel(*channel, (uint8_t *)&packet, packet_size);
}