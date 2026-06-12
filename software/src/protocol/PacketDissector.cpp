#include "PacketDissector.h"

#include "core/MathUtils.h"
#include "core/crc.h"

#include <QStringList>
#include <cstring>

namespace {

// ---- enum name maps (values mirror include/comm/comm_types.h & perf_packet.h,
//      kept local so the GCS doesn't pull in firmware headers) ---------------

QString sysStateName(uint16_t s) {
  switch (s) {
  case 0x01: return "UNINITIALIZED";
  case 0x02: return "INIT";
  case 0x04: return "STANDBY";
  case 0x08: return "PREARM";
  case 0x10: return "ARMED";
  case 0x20: return "IN_AIR";
  case 0x40: return "FAILSAFE";
  case 0x80: return "TERMINATED";
  case 0x100: return "CALIBRATING";
  default: return QString("0x%1").arg(s, 0, 16);
  }
}

QString flightModeName(uint8_t m) {
  switch (m) {
  case 0: return "STABILISE";
  case 1: return "ACRO";
  case 2: return "RELEASE_TO_RC";
  default: return QString::number(m);
  }
}

QString flightSourceName(uint8_t s) {
  switch (s) {
  case 0: return "RC";
  case 1: return "GCS";
  default: return QString::number(s);
  }
}

QString calibTypeName(uint8_t t) {
  switch (t) {
  case 0x00: return "PROGRESS";
  case 0x01: return "NOSE_UP";
  case 0x02: return "NOSE_DOWN";
  case 0x03: return "RIGHT_DOWN";
  case 0x04: return "LEFT_DOWN";
  case 0x05: return "UPRIGHT";
  case 0x06: return "UPSIDE_DOWN";
  case 0x07: return "FREE_ROT";
  case 0x08: return "MAG_AXIS_COVERAGE";
  default: return QString::number(t);
  }
}

QString perfFifoName(uint8_t id) {
  static const char *n[] = {"imu.raw",         "imu.telemetry", "imu.control",
                            "imu.calib",       "imu.calib_telem",
                            "attitude.telemetry", "attitude.control",
                            "rc.telemetry",    "rc.control",    "telemetry"};
  return (id < sizeof(n) / sizeof(n[0])) ? QString(n[id])
                                         : QString("fifo#%1").arg(id);
}

QString taskStateName(uint8_t s) {
  switch (s) {
  case 0: return "READY";
  case 1: return "RUNNING";
  case 2: return "BLOCKED";
  case 3: return "DELAYED";
  case 4: return "SUSPENDED";
  default: return QString::number(s);
  }
}

QString commandName(uint16_t c) {
  switch (c) {
  case 0x0001: return "CALIBRATE_IMU";
  case 0x0002: return "ARM";
  case 0x0003: return "DISARM";
  case 0x000A: return "SET_PID";
  case 0x000B: return "SET_GYRO_LPF";
  case 0x000C: return "SET_MOTOR_GEOMETRY";
  case 0x000D: return "SET_FLIGHT_MODE";
  default: return QString("0x%1").arg(c, 4, 16, QChar('0'));
  }
}

// Printable-ASCII rendering for LOG / RAW payloads.
QString asAscii(const QByteArray &b) {
  QString s;
  for (char c : b)
    s += (c >= 0x20 && c < 0x7f) ? QChar(c) : QChar('.');
  return s;
}

} // namespace

QString PacketDissector::typeName(uint8_t type) {
  switch (type) {
  case 0x0: return "HEARTBEAT";
  case 0x1: return "IMU_FULL";
  case 0x2: return "IMU_COMPRESSED";
  case 0x3: return "COMMAND";
  case 0x4: return "ATTITUDE";
  case 0x5: return "RC_CHANNELS";
  case 0x6: return "SYSTEM_STATUS";
  case 0x7: return "LOG";
  case 0x8: return "MOTOR";
  case 0x9: return "PERF_STATS";
  case 0xA: return "PERF_TASKNAME";
  default: return QString("UNKNOWN(0x%1)").arg(type, 1, 16);
  }
}

QString PacketDissector::originName(uint8_t o) {
  switch (o) {
  case 0x01: return "CALIBRATION";
  case 0x02: return "HEALTH";
  case 0x03: return "MOTOR";
  case 0x04: return "SYS_STATE";
  case 0x05: return "PID_ERROR";
  case 0x06: return "PID_UPDATE";
  case 0x07: return "FLIGHT_MODE";
  case 0x08: return "EST_PERF";
  default: return QString("0x%1").arg(o, 2, 16, QChar('0'));
  }
}

// ===========================================================================
//  Full field tree
// ===========================================================================
DissectField PacketDissector::dissect(const QByteArray &data) {
  const int n = data.size();
  const auto *raw = reinterpret_cast<const uint8_t *>(data.constData());

  auto u16 = [&](int i) { uint16_t v; memcpy(&v, raw + i, 2); return v; };
  auto u32 = [&](int i) { uint32_t v; memcpy(&v, raw + i, 4); return v; };
  auto f32 = [&](int i) { float v; memcpy(&v, raw + i, 4); return v; };
  auto f16 = [&](int i) { return MathUtils::float16_to_float32(u16(i)); };

  // Non-frame / malformed: show it raw.
  if (n < 8 || raw[0] != 0x56) {
    DissectField root("RAW", QString("%1 bytes").arg(n), 0, n);
    root.addChild("ASCII", asAscii(data), 0, n);
    return root;
  }

  const uint8_t version = raw[1] & 0x0F;
  const uint8_t type = (raw[1] >> 4) & 0x0F;
  const uint8_t length = raw[2];
  const uint8_t devId = raw[3];
  const uint32_t ts = u32(4);
  const int payOff = 8;
  const bool payOk = (payOff + length <= n);

  DissectField root(typeName(type), summary(data), 0, n);

  // ---- Header ----
  DissectField &hdr = root.addChild("Header", "", 0, 8);
  hdr.addChild("Sync", QString("0x%1").arg(raw[0], 2, 16, QChar('0')), 0, 1);
  hdr.addChild("Protocol Version", QString::number(version), 1, 1);
  hdr.addChild("Packet Type",
               QString("%1 (0x%2)").arg(typeName(type)).arg(type, 1, 16), 1, 1);
  hdr.addChild("Payload Length", QString::number(length), 2, 1);
  hdr.addChild("Device ID", QString::number(devId), 3, 1);
  hdr.addChild("Timestamp", QString("%1 ms").arg(ts), 4, 4);

  // ---- Payload ----
  DissectField &pl = root.addChild("Payload", "", payOff, length);
  if (!payOk) {
    pl.addChild("Error", "truncated — payload shorter than declared length",
                payOff, n - payOff);
  } else {
    const int p = payOff; // absolute base of payload
    switch (type) {
    case 0x1: { // IMU_FULL: 10 f32
      static const char *nm[] = {"Acc X", "Acc Y", "Acc Z", "Gyr X", "Gyr Y",
                                 "Gyr Z", "Mag X", "Mag Y", "Mag Z", "Temp"};
      static const char *un[] = {" m/s²", " m/s²", " m/s²", " °/s", " °/s",
                                 " °/s",  " µT",   " µT",   " µT",  " °C"};
      for (int i = 0; i < 10 && p + 4 * i + 4 <= n; i++)
        pl.addChild(nm[i], QString::number(f32(p + 4 * i), 'f', 4) + un[i],
                    p + 4 * i, 4);
      break;
    }
    case 0x2: { // IMU_COMPRESSED: 10 f16 deltas
      static const char *nm[] = {"Δ Acc X", "Δ Acc Y", "Δ Acc Z", "Δ Gyr X",
                                 "Δ Gyr Y", "Δ Gyr Z", "Δ Mag X", "Δ Mag Y",
                                 "Δ Mag Z", "Δ Temp"};
      for (int i = 0; i < 10 && p + 2 * i + 2 <= n; i++)
        pl.addChild(nm[i], QString::number(f16(p + 2 * i), 'f', 4), p + 2 * i,
                    2);
      break;
    }
    case 0x4: { // ATTITUDE: roll/pitch/yaw
      static const char *nm[] = {"Roll", "Pitch", "Yaw"};
      for (int i = 0; i < 3 && p + 4 * i + 4 <= n; i++)
        pl.addChild(nm[i], QString::number(f32(p + 4 * i), 'f', 2) + "°",
                    p + 4 * i, 4);
      break;
    }
    case 0x5: // RC_CHANNELS: 14 u16 µs
      for (int i = 0; i < 14 && p + 2 * i + 2 <= n; i++)
        pl.addChild(QString("CH %1").arg(i + 1),
                    QString::number(u16(p + 2 * i)) + " µs", p + 2 * i, 2);
      break;
    case 0x8: // MOTOR: 4 f32 (normalized)
      for (int i = 0; i < 4 && p + 4 * i + 4 <= n; i++)
        pl.addChild(QString("Motor %1").arg(i + 1),
                    QString::number(f32(p + 4 * i) * 100.0f, 'f', 1) + " %",
                    p + 4 * i, 4);
      break;
    case 0x7: // LOG
      pl.addChild("Message", asAscii(data.mid(p, length)), p, length);
      break;
    case 0xA: { // PERF_TASKNAME
      pl.addChild("Task ID", QString::number(raw[p]), p, 1);
      if (length > 1)
        pl.addChild("Name", asAscii(data.mid(p + 1, length - 1)), p + 1,
                    length - 1);
      break;
    }
    case 0x3: { // COMMAND
      if (length >= 2)
        pl.addChild("Command", commandName(u16(p)), p, 2);
      if (length > 2)
        pl.addChild("Args", data.mid(p + 2, length - 2).toHex(' ').toUpper(),
                    p + 2, length - 2);
      break;
    }
    case 0x6: { // SYSTEM_STATUS — origin dispatch
      const uint8_t origin = raw[p];
      pl.addChild("Origin", originName(origin), p, 1);
      switch (origin) {
      case 0x04: // SYS_STATE
        if (p + 6 <= n)
          pl.addChild("State", sysStateName((uint16_t)f32(p + 2)), p + 2, 4);
        break;
      case 0x07: // FLIGHT_MODE
        if (p + 4 <= n) {
          pl.addChild("Mode", flightModeName(raw[p + 2]), p + 2, 1);
          pl.addChild("Source", flightSourceName(raw[p + 3]), p + 3, 1);
        }
        break;
      case 0x08: { // EST_PERF
        static const char *nm[] = {"Peak", "Mean", "Decim", "Rate"};
        static const char *un[] = {" µs", " µs", "", " Hz"};
        for (int i = 0; i < 4 && p + 2 + 4 * i + 4 <= n; i++)
          pl.addChild(nm[i],
                      QString::number(f32(p + 2 + 4 * i), 'f', 2) + un[i],
                      p + 2 + 4 * i, 4);
        break;
      }
      case 0x05: { // PID_ERROR / CONTROL_DATA: 18 f32 at +2
        static const char *nm[] = {
            "roll_sp",     "pitch_sp",    "yaw_sp",     "roll",
            "pitch",       "yaw",         "roll_rate_sp", "pitch_rate_sp",
            "yaw_rate_sp", "roll_rate",   "pitch_rate", "yaw_rate",
            "roll_out",    "pitch_out",   "yaw_out",    "throttle_out",
            "outer_dt",    "inner_dt"};
        for (int i = 0; i < 18 && p + 2 + 4 * i + 4 <= n; i++)
          pl.addChild(nm[i], QString::number(f32(p + 2 + 4 * i), 'f', 4),
                      p + 2 + 4 * i, 4);
        break;
      }
      case 0x01: { // CALIBRATION
        if (p + 3 <= n)
          pl.addChild("Calib Type", calibTypeName(raw[p + 2]), p + 2, 1);
        for (int i = 0; p + 3 + 4 * i + 4 <= n && i < (length - 3) / 4; i++)
          pl.addChild(QString("Value %1").arg(i),
                      QString::number(f32(p + 3 + 4 * i), 'f', 4), p + 3 + 4 * i,
                      4);
        break;
      }
      default: // HEALTH / MOTOR / PID_UPDATE — show raw payload
        if (length > 1)
          pl.addChild("Data", data.mid(p + 1, length - 1).toHex(' ').toUpper(),
                      p + 1, length - 1);
        break;
      }
      break;
    }
    case 0x9: { // PERF_STATS fragment
      if (length >= 8) {
        const uint8_t schema = raw[p];
        const uint8_t section = raw[p + 1];
        const uint8_t index = raw[p + 2];
        const uint8_t count = raw[p + 3];
        const uint32_t seq = u32(p + 4);
        DissectField &fh = pl.addChild("Fragment", "", p, 8);
        fh.addChild("Schema", QString::number(schema), p, 1);
        fh.addChild("Section",
                    section == 0 ? "GLOBAL"
                                 : section == 1 ? "TASKS"
                                                : section == 2 ? "FIFOS" : "?",
                    p + 1, 1);
        fh.addChild("Index", QString::number(index), p + 2, 1);
        fh.addChild("Count", QString::number(count), p + 3, 1);
        fh.addChild("Seq", QString::number(seq), p + 4, 4);

        const int b = p + 8; // body base
        if (section == 0 && b + 72 <= n) { // GLOBAL
          DissectField &g = pl.addChild("Global", "", b, 72);
          g.addChild("Flags", QString("0x%1").arg(raw[b], 2, 16, QChar('0')), b,
                     1);
          g.addChild("Total Tasks", QString::number(raw[b + 1]), b + 1, 1);
          g.addChild("Total FIFOs", QString::number(raw[b + 2]), b + 2, 1);
          static const char *gn[] = {
              "Uptime ticks",  "Sched switches", "CPU cycles lo",
              "Idle cycles lo", "Systick count", "Systick last cyc",
              "Systick max cyc", "Systick preempt", "IPC takes",
              "IPC blocked",   "IPC gives",      "IPC timeouts",
              "Heap allocs",   "Heap frees",     "Heap OOM",
              "Heap peak",     "Heap total"};
          for (int i = 0; i < 17; i++)
            g.addChild(gn[i], QString::number(u32(b + 4 + 4 * i)), b + 4 + 4 * i,
                       4);
        } else if (section == 1) { // TASKS rows (20 B)
          for (int i = 0; i < count && b + i * 20 + 20 <= n; i++) {
            const int r = b + i * 20;
            DissectField &t =
                pl.addChild(QString("Task #%1").arg(raw[r]),
                            taskStateName(raw[r + 2]), r, 20);
            t.addChild("ID", QString::number(raw[r]), r, 1);
            t.addChild("Priority", QString::number(raw[r + 1]), r + 1, 1);
            t.addChild("State", taskStateName(raw[r + 2]), r + 2, 1);
            t.addChild("Stack peak", QString::number(u16(r + 4)), r + 4, 2);
            t.addChild("Stack size", QString::number(u16(r + 6)), r + 6, 2);
            t.addChild("Cycles lo", QString::number(u32(r + 8)), r + 8, 4);
            t.addChild("Switches in", QString::number(u32(r + 12)), r + 12, 4);
            t.addChild("Max burst cyc", QString::number(u32(r + 16)), r + 16, 4);
          }
        } else if (section == 2) { // FIFOS rows (8 B)
          for (int i = 0; i < count && b + i * 8 + 8 <= n; i++) {
            const int r = b + i * 8;
            DissectField &f = pl.addChild(
                perfFifoName(raw[r]),
                QString("%1/%2, drops %3")
                    .arg(u16(r + 2))
                    .arg(u16(r + 4))
                    .arg(u16(r + 6)),
                r, 8);
            f.addChild("ID", perfFifoName(raw[r]), r, 1);
            f.addChild("Peak", QString::number(u16(r + 2)), r + 2, 2);
            f.addChild("Capacity", QString::number(u16(r + 4)), r + 4, 2);
            f.addChild("Drops", QString::number(u16(r + 6)), r + 6, 2);
          }
        }
      }
      break;
    }
    default:
      break; // HEARTBEAT (no payload) / unknown
    }
  }

  // ---- CRC ----
  if (payOk && payOff + length + 4 <= n) {
    const int c = payOff + length;
    const uint32_t got = u32(c);
    const uint32_t calc = CRC32::calculate(raw, payOff + length);
    root.addChild("CRC32",
                  QString("0x%1 (%2)")
                      .arg(got, 8, 16, QChar('0'))
                      .arg(got == calc ? "OK"
                                       : QString("MISMATCH, calc 0x%1")
                                             .arg(calc, 8, 16, QChar('0'))),
                  c, 4);
  }
  return root;
}

// ===========================================================================
//  Cheap one-line summary (Info column) — reads only what it needs
// ===========================================================================
QString PacketDissector::summary(const QByteArray &data) {
  const int n = data.size();
  const auto *raw = reinterpret_cast<const uint8_t *>(data.constData());
  if (n < 8 || raw[0] != 0x56)
    return QString("RAW · %1").arg(asAscii(data.left(24)));

  auto u16 = [&](int i) { uint16_t v; memcpy(&v, raw + i, 2); return v; };
  auto f32 = [&](int i) { float v; memcpy(&v, raw + i, 4); return v; };

  const uint8_t type = (raw[1] >> 4) & 0x0F;
  const uint8_t length = raw[2];
  const int p = 8;
  const bool ok = (p + length <= n);

  switch (type) {
  case 0x0: return "Heartbeat";
  case 0x1: return "IMU full";
  case 0x2: return "IMU Δ (compressed)";
  case 0x4:
    return ok && p + 12 <= n
               ? QString("roll %1  pitch %2  yaw %3")
                     .arg(f32(p), 0, 'f', 1)
                     .arg(f32(p + 4), 0, 'f', 1)
                     .arg(f32(p + 8), 0, 'f', 1)
               : "Attitude";
  case 0x5: return "RC channels";
  case 0x7: return "LOG: " + asAscii(data.mid(p, qMin<int>(length, 48)));
  case 0x8: return "Motor telemetry";
  case 0xA: return ok ? QString("TaskName id=%1").arg(raw[p]) : "TaskName";
  case 0x3:
    return ok && length >= 2 ? "Cmd: " + commandName(u16(p)) : "Command";
  case 0x6: { // SYSTEM_STATUS
    if (!ok)
      return "System status";
    const uint8_t o = raw[p];
    switch (o) {
    case 0x04:
      return p + 6 <= n ? "State: " + sysStateName((uint16_t)f32(p + 2))
                        : "Sys state";
    case 0x07:
      return p + 4 <= n ? "Mode: " + flightModeName(raw[p + 2]) + " / " +
                              flightSourceName(raw[p + 3])
                        : "Flight mode";
    case 0x08: return "Est perf";
    case 0x05: return "PID error";
    case 0x01:
      return p + 3 <= n ? "Calib: " + calibTypeName(raw[p + 2]) : "Calibration";
    default: return "Status: " + originName(o);
    }
  }
  case 0x9: { // PERF_STATS
    if (!ok || length < 4)
      return "Perf stats";
    const char *s = raw[p + 1] == 0 ? "GLOBAL"
                                    : raw[p + 1] == 1 ? "TASKS"
                                                      : raw[p + 1] == 2 ? "FIFOS"
                                                                        : "?";
    return QString("Perf %1 idx=%2 n=%3").arg(s).arg(raw[p + 2]).arg(raw[p + 3]);
  }
  default: return typeName(type);
  }
}
