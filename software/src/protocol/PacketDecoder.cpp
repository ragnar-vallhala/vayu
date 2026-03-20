#include "PacketDecoder.h"
#include "core/MathUtils.h"
#include "core/Types.h"
#include <QString>
#include <cstring>

DecodedPacket PacketDecoder::decode(const QByteArray &data) {
  DecodedPacket result;
  if (data.size() < 8)
    return result;

  const uint8_t *raw = reinterpret_cast<const uint8_t *>(data.constData());

  if (raw[0] != 0x56)
    return result;

  uint8_t type_byte = raw[1];
  result.version = type_byte & 0x0F;
  result.type = (type_byte >> 4) & 0x0F;
  result.length = raw[2]; // Wait, I need to add length to DecodedPacket or just
                          // use data.size()
  result.deviceId = raw[3];
  memcpy(&result.timestamp, raw + 4, 4);

  if (data.size() < 8 + result.length + 4)
    return result;

  result.valid = true;

  if (result.type == 0x0) {
    // HEARTBEAT - Handled by DroneProtocol for signal emission
  } else if (result.type == 0x1) {
    // IMU_DATA_FULL
    if (result.length == 40) {
      float vals[10];
      memcpy(vals, raw + 8, 40);

      ImuData imu;
      for (int i = 0; i < 3; i++) {
        imu.acc[i] = vals[i];
        imu.gyr[i] = vals[i + 3];
        imu.mag[i] = vals[i + 6];
      }
      imu.tempC = vals[9];
      imu.timestamp = result.timestamp;
      result.payload = imu;

      // Update state for delta
      m_lastImu = imu;
      m_hasLastImu = true;
    }
  } else if (result.type == 0x2) {
    // IMU_DATA_DELTA (COMPRESSED)
    if (result.length == 20 && m_hasLastImu) {
      uint16_t deltas[10];
      memcpy(deltas, raw + 8, 20);

      ImuData imu;
      for (int i = 0; i < 3; i++) {
        imu.acc[i] =
            m_lastImu.acc[i] + MathUtils::float16_to_float32(deltas[i]);
        imu.gyr[i] =
            m_lastImu.gyr[i] + MathUtils::float16_to_float32(deltas[i + 3]);
        imu.mag[i] =
            m_lastImu.mag[i] + MathUtils::float16_to_float32(deltas[i + 6]);
      }
      imu.tempC = m_lastImu.tempC + MathUtils::float16_to_float32(deltas[9]);
      imu.timestamp = result.timestamp;
      result.payload = imu;

      // Update state
      m_lastImu = imu;
    }
  } else if (result.type == 0x4) {
    // ATTITUDE
    if (result.length == 12) {
      float vals[3];
      memcpy(vals, raw + 8, 12);

      AttitudeData att;
      att.roll = vals[0];
      att.pitch = vals[1];
      att.yaw = vals[2];
      result.payload = att;
    }
  } else if (result.type == 0x5) {
    // RC_CHANNELS
    if (result.length == 28) {
      RcData rc;
      memcpy(rc.channels, raw + 8, 28);
      rc.timestamp = result.timestamp;
      result.payload = rc;
    }
  } else if (result.type == 0x6) {
    // SYSTEM_STATUS
    if (result.length >= 2 && raw[8] == 0x04) {
      // SYSTEM_ORIGIN_SYS_STATE: [origin] [n] [float32_state]
      if (result.length == 6) {
        float f_state;
        memcpy(&f_state, raw + 10, 4);
        result.payload = sysStateToName(static_cast<uint16_t>(f_state));
      } else {
        result.payload =
            QString("INVALID SYSTEM_STATE LEN: %1").arg(result.length);
      }
    } else if (result.length == 18 && raw[8] == 0x01) {
      // SYSTEM_ORIGIN_CALIBRATION: [origin] [n] [float32 progress] [float32
      // bias_x] [float32 bias_y] [float32 bias_z]
      float progress;
      memcpy(&progress, raw + 10, 4);
      result.payload =
          QString("CALIBRATION PROGRESS: %1%").arg(progress, 0, 'f', 2);
    } else {
      // Legacy or other status origin: treat as string if small, or generic
      // format
      result.payload = QString::fromLatin1(data.mid(8, result.length));
    }
  } else if (result.type == 0x7) {
    // LOG
    result.payload = QString::fromLatin1(data.mid(8, result.length));
  }

  return result;
}

QString PacketDecoder::typeToString(uint8_t type) {
  switch (type) {
  case 0x0:
    return "HEARTBEAT";
  case 0x1:
    return "IMU_DATA_FULL";
  case 0x2:
    return "IMU_DATA_COMPRESSED";
  case 0x4:
    return "ATTITUDE";
  case 0x5:
    return "RC_CHANNELS";
  case 0x6:
    return "SYSTEM_STATUS";
  case 0x7:
    return "LOG";
  default:
    return QString("UNKNOWN (0x%1)").arg(type, 1, 16, QChar('0')).toUpper();
  }
}

QString PacketDecoder::sysStateToName(uint16_t state) {
  switch (state) {
  case 0x01:
    return "UNINITIALIZED";
  case 0x02:
    return "INIT";
  case 0x04:
    return "STANDBY";
  case 0x08:
    return "PREARM";
  case 0x10:
    return "ARMED";
  case 0x20:
    return "IN_AIR";
  case 0x40:
    return "FAILSAFE";
  case 0x80:
    return "TERMINATED";
  case 0x100:
    return "CALIBRATING";
  default:
    return QString("STATE: 0x%1").arg(state, 2, 16, QChar('0')).toUpper();
  }
}
