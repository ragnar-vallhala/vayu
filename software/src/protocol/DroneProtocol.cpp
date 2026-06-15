#include "DroneProtocol.h"
#include "../core/crc.h"
#include <QDateTime>
#include <QStringList>
#include <cstdint>
#include <cstring>
#include <variant>

DroneProtocol::DroneProtocol(QObject *parent) : QObject(parent) {
  // Wire the v2 router's leaf hooks to our existing Qt signals. Adding a leaf is
  // a one-liner here; the generated codec stays inside NavlinkRouter.
  m_v2Router.onAttitude = [this](const AttitudeData &a) {
    emit attitudeReceived(a);
  };
  m_v2Router.onImu = [this](const ImuData &d) { emit imuReceived(d); };
  m_v2Router.onRc = [this](const RcData &d) { emit rcReceived(d); };
  m_v2Router.onMotor = [this](const MotorData &d) { emit motorReceived(d); };
  m_v2Router.onControlLoop = [this](const ControlLoopData &d) {
    emit controlLoopDataReceived(d);
  };
  m_v2Router.onEstPerf = [this](const EstPerfData &d) {
    emit estPerfReceived(d);
  };
  m_v2Router.onFlightMode = [this](uint8_t mode, uint8_t source) {
    emit flightModeReceived(mode, source);
  };
  // onSystemHealth intentionally left unset: no GCS consumer (parity with v1).
  m_v2Router.onDefault = [this](uint32_t msgid, int len) {
    emit logReceived(
        QStringLiteral("[navlink] v2 msgid %1 (%2 B)").arg(msgid).arg(len));
  };
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
    if (type_byte == 0x02) {  // NavLink v2 version byte
      constexpr int kV2HeaderLen = 10;        // sync,ver,len,flags,seq,sys,comp,msgid(3)
      if (m_buffer.size() < kV2HeaderLen)
        break;  // wait for the full v2 header (payload_len at [2])
      int v2_total =
          kV2HeaderLen + static_cast<uint8_t>(m_buffer[2]) + 2; // + CRC-16
      if (m_buffer.size() < v2_total)
        break;  // wait for the rest of the frame
      const QByteArray frame = m_buffer.left(v2_total);
      emit packetReceived(frame);
      m_v2Router.feed(frame);   // decode + dispatch (codec lives in the router)
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
