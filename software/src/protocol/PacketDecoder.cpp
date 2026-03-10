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
  default:
    return QString("UNKNOWN (0x%1)").arg(type, 1, 16, QChar('0')).toUpper();
  }
}
