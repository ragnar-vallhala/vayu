#include "DroneProtocol.h"
#include "../core/crc.h"
#include <QDateTime>
#include <QStringList>
#include <cstdint>
#include <cstring>
#include <variant>

DroneProtocol::DroneProtocol(QObject *parent) : QObject(parent) {
  navlink_parser_init(&m_v2Parser);
  m_v2Handlers = {};                              // zero every slot
  m_v2Handlers.ctx = this;                        // member assignment: C++ has
  m_v2Handlers.on_attitude_euler = &DroneProtocol::onV2AttitudeEuler; // no
                                                  // out-of-order designated init
}

void DroneProtocol::onV2AttitudeEuler(void *ctx, const navlink_frame_hdr_t *,
                                      const navlink_attitude_euler_t *msg) {
  auto *self = static_cast<DroneProtocol *>(ctx);
  // v2 ATTITUDE_EULER is radians; AttitudeData is degrees (core/Types.h). Convert
  // so the signal / VehicleState / UI contract is unchanged (INTEGRATION.md §3).
  constexpr float kRad2Deg = 57.29577951308232f;
  AttitudeData att;
  att.roll = msg->roll * kRad2Deg;
  att.pitch = msg->pitch * kRad2Deg;
  att.yaw = msg->yaw * kRad2Deg;
  emit self->attitudeReceived(att);
}

void DroneProtocol::processData(const QByteArray &data) {
  m_buffer.append(data);
  parseBuffer();
}

void DroneProtocol::parseBuffer() {
  while (true) {
    if (m_buffer.isEmpty())
      break;

    // 1. Scan for the sync byte (0x56)
    int syncIdx = m_buffer.indexOf(0x56);
    if (syncIdx == -1) {
      // Forward everything else as an unknown packet (could be plain text logs)
      emit unknownPacket(m_buffer);
      m_buffer.clear();
      break;
    }

    if (syncIdx > 0) {
      emit unknownPacket(m_buffer.left(syncIdx));
      m_buffer.remove(0, syncIdx);
    }

    // A packet header is 8 bytes:
    // sync (1) + type (1) + length (1) + dev_id (1) + timestamp (4)
    if (m_buffer.size() < 8)
      break;

    // 2. Validate protocol version & extract type
    uint8_t type_byte = m_buffer[1];
    uint8_t protocol_version = type_byte & 0x0F;
    uint8_t packet_type = (type_byte >> 4) & 0x0F;

    // NavLink v2 frames share the 0x56 sync but byte 1 == 0x02 (vs v1's low
    // nibble == 1), so they demux cleanly (INTEGRATION.md §2). Consume the whole
    // v2 frame and hand it to the generated parser (CRC + typed dispatch) so the
    // v1 scanner below never nibbles at it byte-by-byte.
    if (type_byte == NAVLINK_VERSION) {
      if (m_buffer.size() < NAVLINK_HEADER_LEN)
        break;  // wait for the full 10-byte v2 header (payload_len at [2])
      int v2_total =
          NAVLINK_HEADER_LEN + static_cast<uint8_t>(m_buffer[2]) + 2; // + CRC-16
      if (m_buffer.size() < v2_total)
        break;  // wait for the rest of the frame
      const QByteArray frame = m_buffer.left(v2_total);
      emit packetReceived(frame);
      navlink_parser_push(
          &m_v2Parser, &m_v2Handlers,
          reinterpret_cast<const uint8_t *>(frame.constData()),
          static_cast<size_t>(v2_total));
      m_buffer.remove(0, v2_total);
      continue;
    }

    if (protocol_version != 0x1) {
      // Invalid protocol version, discard the false sync byte and continue
      emit unknownPacket(m_buffer.left(1));
      m_buffer.remove(0, 1);
      continue;
    }

    // 3. Extract length (payload size)
    uint8_t payload_length = m_buffer[2];
    int total_packet_size = 8 + payload_length + 4; // header + payload + CRC32

    if (m_buffer.size() < total_packet_size)
      break;

    // 4. Validate CRC32
    uint32_t computed_crc = CRC32::calculate(
        reinterpret_cast<const uint8_t *>(m_buffer.constData()),
        8 + payload_length);

    uint32_t received_crc;
    memcpy(&received_crc, m_buffer.constData() + 8 + payload_length, 4);

    if (m_checkCrc && computed_crc != received_crc) {
      // CRC mismatch, discard false sync byte (unless CRC checking is off, the
      // Advanced debug toggle, in which case we accept the frame as-is).
      emit unknownPacket(m_buffer.left(1));
      m_buffer.remove(0, 1);
      continue;
    }

    // 5. Valid packet! Parse contents.
    emit packetReceived(m_buffer.left(total_packet_size));

    // Time-sync RESPONSE (0xB): a fixed binary payload the decoder doesn't
    // model. Capture t4 here (worker thread, closest to receipt) and hand the
    // four timestamps to the estimator. (docs/telemetry/time_sync.md)
    if (packet_type == 0x0B) {
      if (payload_length >= 32) {
        const char *p = m_buffer.constData() + 8;  // payload base
        if (static_cast<quint8>(p[0]) == 0x01) {   // role == RESPONSE
          const quint8 seq = static_cast<quint8>(p[1]);
          quint64 t1, t2, t3;
          memcpy(&t1, p + 4, 8);
          memcpy(&t2, p + 12, 8);
          memcpy(&t3, p + 20, 8);
          const quint64 t4 =
              static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
          emit timeSyncResponse(seq, t1, t2, t3, t4);
        }
      }
      m_buffer.remove(0, total_packet_size);
      continue;
    }

    DecodedPacket decoded = m_decoder.decode(m_buffer.left(total_packet_size));
    if (decoded.valid) {
      if (std::holds_alternative<ImuData>(decoded.payload)) {
        emit imuReceived(std::get<ImuData>(decoded.payload));
      } else if (std::holds_alternative<CalibrationUpdate>(decoded.payload)) {
        emit calibrationUpdateReceived(
            std::get<CalibrationUpdate>(decoded.payload));
      } else if (std::holds_alternative<AttitudeData>(decoded.payload)) {
        emit attitudeReceived(std::get<AttitudeData>(decoded.payload));
      } else if (std::holds_alternative<RcData>(decoded.payload)) {
        emit rcReceived(std::get<RcData>(decoded.payload));
      } else if (std::holds_alternative<MotorData>(decoded.payload)) {
        emit motorReceived(std::get<MotorData>(decoded.payload));
      } else if (std::holds_alternative<ControlLoopData>(decoded.payload)) {
        emit controlLoopDataReceived(
            std::get<ControlLoopData>(decoded.payload));
      } else if (std::holds_alternative<FlightModeStatus>(decoded.payload)) {
        const auto &fm = std::get<FlightModeStatus>(decoded.payload);
        emit flightModeReceived(fm.mode, fm.source);
      } else if (std::holds_alternative<EstPerfData>(decoded.payload)) {
        emit estPerfReceived(std::get<EstPerfData>(decoded.payload));
      } else if (std::holds_alternative<PerfReport>(decoded.payload)) {
        emit perfReceived(std::get<PerfReport>(decoded.payload));
      } else if (std::holds_alternative<TaskNameInfo>(decoded.payload)) {
        const auto &tn = std::get<TaskNameInfo>(decoded.payload);
        emit taskNameReceived(tn.id, tn.name);
      } else if (std::holds_alternative<QString>(decoded.payload)) {
        if (packet_type == 0x6) {
          emit statusReceived(std::get<QString>(decoded.payload));
        } else if (packet_type == 0x7) {
          emit logReceived(std::get<QString>(decoded.payload));
        } else {
          emit logReceived(std::get<QString>(decoded.payload));
        }
      } else if (packet_type == 0x0) { // Keep heartbeat logic if decoder
                                       // doesn't handle it fully
        uint8_t device_id = m_buffer[3];
        uint32_t timestamp;
        memcpy(&timestamp, m_buffer.constData() + 4, 4);
        emit heartbeatReceived(timestamp, device_id);
      }
    }

    // Remove parsed packet from the buffer
    m_buffer.remove(0, total_packet_size);
  }
}
