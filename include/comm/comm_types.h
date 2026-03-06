#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include <stdint.h>

#define SYNC_BYTE 0x56
#define PROTOCOL_VERSION 0x1

typedef enum {
    PACKET_TYPE_HEARTBEAT = 0x0,
    PACKET_TYPE_IMU_DATA_FULL = 0x1,
    PACKET_TYPE_IMU_DATA_COMPRESSED = 0x2,
} packet_type_t;

typedef struct {
    uint8_t sync;
    uint8_t protocol_packet_type; // 4 bits protocol version, 4 bits packet type
    uint8_t length;
    uint8_t device_id;
    uint32_t timestamp;
    uint8_t payload[256];
    uint32_t crc32;
} packet_t;

#endif // !COMMUNICATION_TYPES_H