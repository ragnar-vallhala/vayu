#include "DroneProtocol.h"
#include <QStringList>

DroneProtocol::DroneProtocol(QObject *parent) : QObject(parent) {}

void DroneProtocol::parseLine(const QByteArray &line) {
  if (line.isEmpty())
    return;

  QString str = QString::fromLatin1(line);

  // ------------------------------------------------------------------ $IMU
  if (str.contains("$IMU,")) {
    int idx = str.indexOf("$IMU,");
    const QStringList parts = str.mid(idx + 5).split(',');
    if (parts.size() < 9) {
      emit unknownPacket(line);
      return;
    }
    ImuData d;
    bool ok = true;
    d.acc[0] = parts[0].toFloat(&ok);
    if (!ok)
      goto bad;
    d.acc[1] = parts[1].toFloat(&ok);
    if (!ok)
      goto bad;
    d.acc[2] = parts[2].toFloat(&ok);
    if (!ok)
      goto bad;
    d.gyr[0] = parts[3].toFloat(&ok);
    if (!ok)
      goto bad;
    d.gyr[1] = parts[4].toFloat(&ok);
    if (!ok)
      goto bad;
    d.gyr[2] = parts[5].toFloat(&ok);
    if (!ok)
      goto bad;
    d.mag[0] = parts[6].toFloat(&ok);
    if (!ok)
      goto bad;
    d.mag[1] = parts[7].toFloat(&ok);
    if (!ok)
      goto bad;
    d.mag[2] = parts[8].toFloat(&ok);
    if (!ok)
      goto bad;
    if (parts.size() >= 10)
      d.tempC = parts[9].toFloat();
    emit imuReceived(d);
    return;

  bad:
    emit unknownPacket(line);
    return;
  }

  // ------------------------------------------------------------------ $ATT
  if (str.contains("$ATT,")) {
    int idx = str.indexOf("$ATT,");
    const QStringList parts = str.mid(idx + 5).split(',');
    if (parts.size() < 3) {
      emit unknownPacket(line);
      return;
    }
    AttitudeData d;
    bool ok = true;
    d.roll = parts[0].toFloat(&ok);
    if (!ok) {
      emit unknownPacket(line);
      return;
    }
    d.pitch = parts[1].toFloat(&ok);
    if (!ok) {
      emit unknownPacket(line);
      return;
    }
    d.yaw = parts[2].toFloat(&ok);
    if (!ok) {
      emit unknownPacket(line);
      return;
    }
    emit attitudeReceived(d);
    return;
  }

  // ------------------------------------------------------------------ $LOG
  if (str.contains("$LOG,")) {
    int idx = str.indexOf("$LOG,");
    emit logReceived(str.mid(idx + 5).trimmed());
    return;
  }

  // ------------------------------------------------------------------ $HB
  if (str.contains("$HB,")) {
    int idx = str.indexOf("$HB,");
    const QStringList parts = str.mid(idx + 4).split(',');
    if (parts.size() >= 2) {
      bool ok1, ok2;
      uint64_t ts = parts[0].toULongLong(&ok1);
      uint8_t id = (uint8_t)parts[1].toUInt(&ok2);
      if (ok1 && ok2) {
        emit heartbeatReceived(ts, id);
        return;
      }
    }
  }

  // ------------------------------------------------------------------
  // $TIME_REQ
  if (str.contains("$TIME_REQ")) {
    emit timeSyncRequested();
    return;
  }

  // Unknown — forward as-is (also catches raw v_log output)
  emit logReceived(str.trimmed());
}
