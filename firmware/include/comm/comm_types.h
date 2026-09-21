/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
  PACKET_TYPE_PERF_STATS = 0x9,          // FC -> GCS (kernel/observability)
  PACKET_TYPE_PERF_TASKNAME = 0xA, // GCS <-> FC (task id -> name, on demand)
  PACKET_TYPE_TIME_SYNC = 0xB,     // GCS <-> FC (NTP-style clock sync)
} packet_type_t;

/**
 * @brief Roles for PACKET_TYPE_TIME_SYNC (docs/telemetry/time_sync.md)
 */
typedef enum {
  TIME_SYNC_REQUEST = 0x00,  // GCS -> FC
  TIME_SYNC_RESPONSE = 0x01, // FC -> GCS
  TIME_SYNC_REQUEST_WIDE =
      0x02, // GCS -> FC, full 64-bit correction (large dev)
} time_sync_role_t;

/**
 * @brief PACKET_TYPE_TIME_SYNC payload (little-endian, 32 bytes).
 *
 * NTP four-timestamp handshake. The GCS sends a REQUEST stamping t1; the FC
 * replies with a RESPONSE echoing t1 and adding t2 (its receive time) and t3
 * (its send time). The GCS captures t4 on receipt and computes
 *   offset = ((t2-t1)+(t3-t4))/2,  delay = (t4-t1)-(t3-t2).
 * FC stamps are its get_timestamp_unix() ms widened to 64-bit; GCS stamps are
 * wall-clock ms. commanded_offset_ms carries the GCS's filtered correction back
 * to the FC (applied via time_sync_set_offset); INT32_MIN means "no command".
 */
typedef struct __attribute__((packed)) {
  uint8_t role;       // time_sync_role_t
  uint8_t seq;        // request sequence, echoed in the response
  uint8_t _pad[2];    // reserved, zero
  uint64_t t1_gcs_tx; // GCS send    (wall-clock ms)
  uint64_t t2_fc_rx;  // FC receive  (FC ms)
  uint64_t t3_fc_tx;  // FC send     (FC ms)
  int32_t
      commanded_offset_ms; // GCS->FC correction (low word if WIDE); INT32_MIN = none
  int32_t
      commanded_offset_hi_ms; // high 32 bits when role==REQUEST_WIDE, else 0
} time_sync_payload_t;

/**
 * @brief Origins for PACKET_TYPE_SYSTEM_STATUS (FC -> GCS)
 */
typedef enum {
  SYSTEM_ORIGIN_CALIBRATION = 0x01,
  SYSTEM_ORIGIN_HEALTH = 0x02,
  SYSTEM_ORIGIN_MOTOR = 0x03,
  SYSTEM_ORIGIN_SYS_STATE = 0x04,
  SYSTEM_ORIGIN_PID_ERROR = 0x05,
  SYSTEM_ORIGIN_PID_UPDATE = 0x06,
  SYSTEM_ORIGIN_FLIGHT_MODE = 0x07, // [origin][pad][mode:u8][source:u8]
  SYSTEM_ORIGIN_EST_PERF =
      0x08, // [origin][n=4][peak_us,mean_us,decim,rate_hz : f32]
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
  /* Per-axis mag coverage (FC -> GCS): coverage[0..2] carry the fraction (0..100)
   * of each axis's normalized-field range swept so far. Drives the GCS coverage
   * readout; the FC also derives overall PROGRESS from it. Wire: step 8, len>=15
   * (navlink_tx_calibration coverage path). */
  CALIB_UPDATE_MAG_AXIS_COVERAGE = 0x08,
  /* Terminal status (FC -> GCS): the routine ended. COMPLETE = persisted OK;
   * FAILED = aborted/fit failure/save failure. Lets the GCS wizard finish on an
   * explicit event instead of inferring it from a STANDBY heartbeat (which is
   * absent when calibration legitimately ends in FAILSAFE). Keep these aligned
   * with navlink calib_step. */
  CALIB_UPDATE_COMPLETE = 0x09,
  CALIB_UPDATE_FAILED = 0x0A,
  /* Edge/corner accel poses (FC -> GCS) for the pose-tolerant full-3x3 accel
   * calibration: in addition to the 6 face poses above, the operator rests the
   * board on ~6 edges/corners so gravity is shared between axes (the only way the
   * off-diagonal/misalignment terms of the 3x3 become observable). The fit is
   * magnitude-only, so the exact pose is advisory — the GCS just needs a distinct
   * illustration per code. NavLink is unchanged: these ride the existing
   * calibration status buffer, not a dialect message. */
  CALIB_UPDATE_EDGE_1 = 0x0B,
  CALIB_UPDATE_EDGE_2 = 0x0C,
  CALIB_UPDATE_EDGE_3 = 0x0D,
  CALIB_UPDATE_EDGE_4 = 0x0E,
  CALIB_UPDATE_EDGE_5 = 0x0F,
  CALIB_UPDATE_EDGE_6 = 0x10,
  /* Board-level / trim (FC -> GCS): "hold the frame level and still" prompt for
   * the mounting-tilt calibration (imu_id 4). One level hold; the FC records the
   * gravity-derived roll/pitch as board_trim. Keep aligned with navlink
   * calib_step BOARD_LEVEL (17) and the GCS CalibUpdateType::BoardLevel. */
  CALIB_UPDATE_BOARD_LEVEL = 0x11,
} calib_update_type_t;

/**
 * @brief Remote control command IDs (GCS -> FC)
 */
typedef enum {
  CMD_CALIBRATE_IMU = 0x0001,
  CMD_ARM = 0x0002, // GCS-initiated arm request (sets the software-arm latch)
  CMD_DISARM = 0x0003, // GCS-initiated disarm (clears the software-arm latch)
  CMD_SET_PID = 0x000A,
  CMD_SET_GYRO_LPF = 0x000B,       // rate-loop gyro low-pass time constant
  CMD_SET_MOTOR_GEOMETRY = 0x000C, // per-motor x,y,spin -> mixer signs
  CMD_SET_FLIGHT_MODE =
      0x000D, // arg0: 0=stabilise/angle, 1=acro, 2=release to RC
  CMD_SET_D_LPF =
      0x000E, // rate-loop derivative (D-term) low-pass time constant
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