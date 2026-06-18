#include "DroneProtocol.h"
#include <QString>
#include <cstdint>

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
  m_v2Router.onBaro = [this](const BaroData &d) { emit baroReceived(d); };
  m_v2Router.onFlightMode = [this](uint8_t mode, uint8_t source) {
    emit flightModeReceived(mode, source);
  };
  m_v2Router.onHeartbeat = [this](uint8_t navState, uint64_t ts, uint8_t dev) {
    // nav_state enum index -> the state name TelemetryEngine matches on (same
    // order as the firmware's one-hot sys_state, folded into HEARTBEAT).
    static const char *const kStateNames[] = {
        "UNINITIALIZED", "INIT",     "STANDBY",    "PREARM",
        "ARMED",         "IN_AIR",   "FAILSAFE",   "TERMINATED",
        "CALIBRATING"};
    if (navState < 9)
      emit statusReceived(QString::fromLatin1(kStateNames[navState]));
    emit heartbeatReceived(ts, dev);
  };
  m_v2Router.onLog = [this](const QString &line) { emit logReceived(line); };
  m_v2Router.onCalibration = [this](const CalibrationUpdate &u) {
    emit calibrationUpdateReceived(u);
  };
  m_v2Router.onPerf = [this](const PerfReport &r) { emit perfReceived(r); };
  m_v2Router.onTaskName = [this](int id, const QString &name) {
    emit taskNameReceived(id, name);
  };
  m_v2Router.onTimeSync = [this](uint8_t seq, uint64_t t1, uint64_t t2,
                                 uint64_t t3, uint64_t t4) {
    emit timeSyncResponse(seq, t1, t2, t3, t4);
  };
  m_v2Router.onCommandAck = [this](uint32_t command, uint8_t reqSeq,
                                   uint8_t result) {
    static const char *const kRes[] = {"ACCEPTED",    "TEMP_REJECTED",
                                       "DENIED",      "UNSUPPORTED",
                                       "FAILED",      "IN_PROGRESS"};
    const QString res =
        result < 6 ? QString::fromLatin1(kRes[result]) : QString::number(result);
    emit commandAckReceived(command, reqSeq, result);
    emit logReceived(QStringLiteral("[ack] cmd %1 #%2 → %3")
                         .arg(command)
                         .arg(reqSeq)
                         .arg(res));
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
  // Pure NavLink v2: every frame is sync(0x56) + version(0x02) + 10-byte header
  // + payload + CRC-16. Slice whole frames and hand them to the router, which
  // CRC-checks and dispatches to the typed hooks wired in the ctor. Anything
  // that isn't a v2 frame is surfaced as an unknown packet. The v1 wire path is
  // retired.
  while (true) {
    if (m_buffer.isEmpty())
      break;

    int syncIdx = m_buffer.indexOf(0x56);
    if (syncIdx == -1) {
      emit unknownPacket(m_buffer);
      m_buffer.clear();
      break;
    }
    if (syncIdx > 0) {
      emit unknownPacket(m_buffer.left(syncIdx));
      m_buffer.remove(0, syncIdx);
    }

    constexpr int kV2HeaderLen = 10;  // sync,ver,len,flags,seq,sys,comp,msgid(3)
    if (m_buffer.size() < kV2HeaderLen)
      break;  // wait for a full header (payload_len at [2])

    if (static_cast<quint8>(m_buffer[1]) != 0x02) {
      // Not a NavLink v2 frame — drop the false sync byte and rescan.
      emit unknownPacket(m_buffer.left(1));
      m_buffer.remove(0, 1);
      continue;
    }

    const int total =
        kV2HeaderLen + static_cast<uint8_t>(m_buffer[2]) + 2;  // + CRC-16
    if (m_buffer.size() < total)
      break;  // wait for the rest of the frame

    const QByteArray frame = m_buffer.left(total);
    emit packetReceived(frame);
    m_v2Router.feed(frame);  // CRC-check + typed dispatch (codec lives here)
    m_buffer.remove(0, total);
  }
}
