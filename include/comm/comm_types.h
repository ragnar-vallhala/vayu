#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include "common/hal_types.h"
#include <stdint.h>

#define SYNC_BYTE 0x56
#define PROTOCOL_VERSION 0x1

typedef enum {
  PACKET_TYPE_HEARTBEAT = 0x0,           // FC -> GCS
  PACKET_TYPE_IMU_DATA_FULL = 0x1,       // FC -> GCS
  PACKET_TYPE_IMU_DATA_COMPRESSED = 0x2, // FC -> GCS
  PACKET_TYPE_COMMAND = 0x3,             // GCS -> FC
  PACKET_TYPE_ATTITUDE = 0x4,            // FC -> GCS
  PACKET_TYPE_RC_CHANNELS = 0x5,         // FC -> GCS
  PACKET_TYPE_SYSTEM_STATUS = 0x6,       // FC -> GCS
  PACKET_TYPE_LOG = 0x7,                 // FC -> GCS
  PACKET_TYPE_MOTOR_TELEMETRY = 0x8,     // FC -> GCS
} packet_type_t;

/**
 * @brief Origins for PACKET_TYPE_SYSTEM_STATUS (FC -> GCS)
 */
typedef enum {
  SYSTEM_ORIGIN_CALIBRATION = 0x01,
  SYSTEM_ORIGIN_HEALTH = 0x02,
  SYSTEM_ORIGIN_MOTOR = 0x03,
  SYSTEM_ORIGIN_SYS_STATE = 0x04,
} system_status_origin_t;

/**
 * @brief Calibration status and operator instructions (FC -> GCS)
 */
typedef enum {
  CALIB_UPDATE_PROGRESS = 0x00,
  CALIB_UPDATE_NOSE_UP = 0x01,
  CALIB_UPDATE_NOSE_DOWN = 0x02,
  CALIB_UPDATE_RIGHT_DOWN = 0x03,
  CALIB_UPDATE_LEFT_DOWN = 0x04,
  CALIB_UPDATE_UPRIGHT = 0x05,
  CALIB_UPDATE_UPSIDE_DOWN = 0x06,
  CALIB_UPDATE_FREE_ROT = 0x07,
} calib_update_type_t;

/**
 * @brief Remote control command IDs (GCS -> FC)
 */
typedef enum {
  CMD_CALIBRATE_IMU = 0x0001,
} packet_command_type_t;

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