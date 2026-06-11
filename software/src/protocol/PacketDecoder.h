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
  std::variant<std::monostate, ImuData, QString, AttitudeData, RcData,
               CalibrationUpdate, MotorData, ControlLoopData, FlightModeStatus,
               PerfReport, TaskNameInfo>
      payload;
  bool valid = false;
};

class PacketDecoder {
public:
  DecodedPacket decode(const QByteArray &data);
  static QString typeToString(uint8_t type);

private:
  QString sysStateToName(uint16_t state);
  // Reassemble a PerfReport from its GLOBAL/TASKS/FIFOS fragments. Returns true
  // (and fills `out`) only once a report is complete (all rows received).
  bool decodePerf(const uint8_t *raw, uint8_t length, PerfReport &out);
  ImuData m_lastImu;
  bool m_hasLastImu = false;
  PerfReport m_perfAccum;
  bool m_perfHaveGlobal = false;
  int m_pendingTasks = 0;
  int m_pendingFifos = 0;
};
