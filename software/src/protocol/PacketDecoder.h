#pragma once

#include "../core/MathUtils.h"
#include "../core/Types.h"
#include <QByteArray>
#include <QString>
#include <cstdint>
#include <variant>

struct DecodedPacket {
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint8_t deviceId;
  uint32_t timestamp;
  std::variant<std::monostate, ImuData, QString> payload;
  bool valid = false;
};

class PacketDecoder {
public:
  DecodedPacket decode(const QByteArray &data);
  static QString typeToString(uint8_t type);

private:
  ImuData m_lastImu;
  bool m_hasLastImu = false;
};
