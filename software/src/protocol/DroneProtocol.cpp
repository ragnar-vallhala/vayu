#include "DroneProtocol.h"
#include "../core/crc.h"
#include <QStringList>
#include <cstdint>
#include <cstring>
#include <variant>

DroneProtocol::DroneProtocol(QObject *parent) : QObject(parent) {}

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

    if (computed_crc != received_crc) {
      // CRC mismatch, discard false sync byte
      emit unknownPacket(m_buffer.left(1));
      m_buffer.remove(0, 1);
      continue;
    }

    // 5. Valid packet! Parse contents.
    emit packetReceived(m_buffer.left(total_packet_size));

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
