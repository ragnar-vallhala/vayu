#include "DroneProtocol.h"
#include "../core/crc.h"
#include <QStringList>
#include <cstdint>
#include <cstring>

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

    uint8_t device_id = m_buffer[3];
    uint32_t timestamp;
    memcpy(&timestamp, m_buffer.constData() + 4, 4);

    if (packet_type == 0x0) {
      // HEARTBEAT
      emit heartbeatReceived(timestamp, device_id);
    } else if (packet_type == 0x1) {
      // IMU_DATA_FULL (Payload: 10 x int16_t = 20 bytes)
      if (payload_length == 20) {
        int16_t raw[10];
        memcpy(raw, m_buffer.constData() + 8, 20);

        ImuData data;
        // Accel range ±8g: 4096 LSB/g (32768 / 8), 1g = 9.80665 m/s²
        const float acc_scale = 9.80665f * 8.0f / 32768.0f;
        data.acc[0] = static_cast<float>(raw[0]) * acc_scale;
        data.acc[1] = static_cast<float>(raw[1]) * acc_scale;
        data.acc[2] = static_cast<float>(raw[2]) * acc_scale;

        // Gyro range ±1000 dps: 32.768 LSB/dps (32768 / 1000)
        const float gyr_scale = 1000.0f / 32768.0f;
        data.gyr[0] = static_cast<float>(raw[3]) * gyr_scale;
        data.gyr[1] = static_cast<float>(raw[4]) * gyr_scale;
        data.gyr[2] = static_cast<float>(raw[5]) * gyr_scale;

        // Mag range: 0.3 LSB/µT (from bmx160.c)
        const float mag_scale = 0.3f;
        data.mag[0] = static_cast<float>(raw[6]) * mag_scale;
        data.mag[1] = static_cast<float>(raw[7]) * mag_scale;
        data.mag[2] = static_cast<float>(raw[8]) * mag_scale;

        // Temperature: 1/512 K/LSB, offset 23.0°C at 0
        data.tempC = static_cast<float>(raw[9]) / 512.0f + 23.0f;

        data.timestamp = timestamp;
        emit imuReceived(data);
      }
    }

    // Remove parsed packet from the buffer
    m_buffer.remove(0, total_packet_size);
  }
}
