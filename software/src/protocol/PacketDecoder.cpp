#include "PacketDecoder.h"
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
    // HEARTBEAT - no payload usually or just empty
    result.payload = QString("Heartbeat from Device %1").arg(result.deviceId);
  } else if (result.type == 0x1) {
    // IMU_DATA_FULL
    if (result.length == 20) {
      int16_t vals[10];
      memcpy(vals, raw + 8, 20);

      ImuData imu;
      const float acc_scale = 9.80665f * 8.0f / 32768.0f;
      imu.acc[0] = static_cast<float>(vals[0]) * acc_scale;
      imu.acc[1] = static_cast<float>(vals[1]) * acc_scale;
      imu.acc[2] = static_cast<float>(vals[2]) * acc_scale;

      const float gyr_scale = 1000.0f / 32768.0f;
      imu.gyr[0] = static_cast<float>(vals[3]) * gyr_scale;
      imu.gyr[1] = static_cast<float>(vals[4]) * gyr_scale;
      imu.gyr[2] = static_cast<float>(vals[5]) * gyr_scale;

      const float mag_scale = 0.3f;
      imu.mag[0] = static_cast<float>(vals[6]) * mag_scale;
      imu.mag[1] = static_cast<float>(vals[7]) * mag_scale;
      imu.mag[2] = static_cast<float>(vals[8]) * mag_scale;

      imu.tempC = static_cast<float>(vals[9]) / 512.0f + 23.0f;
      imu.timestamp = result.timestamp;
      result.payload = imu;
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
  default:
    return QString("UNKNOWN (0x%1)").arg(type, 1, 16, QChar('0')).toUpper();
  }
}
