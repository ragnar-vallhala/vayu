#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include "common/hal_types.h"
#include <stdint.h>

#define SYNC_BYTE 0x56
#define PROTOCOL_VERSION 0x1

typedef enum {
  PACKET_TYPE_HEARTBEAT = 0x0,
  PACKET_TYPE_IMU_DATA_FULL = 0x1,
  PACKET_TYPE_IMU_DATA_COMPRESSED = 0x2,
  PACKET_TYPE_COMMAND = 0x3,
  PACKET_TYPE_ATTITUDE = 0x4,
  PACKET_TYPE_RC_CHANNELS = 0x5,
  PACKET_TYPE_SYSTEM_STATUS = 0x6,
  PACKET_TYPE_LOG = 0x7,
} packet_type_t;

typedef struct __attribute__((packed)) {
  uint8_t sync;
  uint8_t protocol_packet_type; // 4 bits protocol version, 4 bits packet type
  uint8_t length;
  uint8_t device_id;
  uint32_t timestamp;
  uint8_t payload[256];
  uint32_t crc32;
} packet_t;

typedef union {
  byte b;
  char c;
} byte_char_u;

#endif // !COMMUNICATION_TYPES_H