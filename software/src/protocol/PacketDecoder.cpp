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
  result.length = raw[2];
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
    } else if (result.length == 4 && raw[8] == 0x07) {
      // SYSTEM_ORIGIN_FLIGHT_MODE: [origin] [pad] [mode:u8] [source:u8]
      FlightModeStatus fm;
      fm.mode = raw[10];
      fm.source = raw[11];
      result.payload = fm;
    } else if (result.length == 18 && raw[8] == 0x08) {
      // SYSTEM_ORIGIN_EST_PERF: [origin] [n=4] [peak_us, mean_us, decim,
      // rate_hz : f32]
      float vals[4];
      memcpy(vals, raw + 10, 16);
      EstPerfData perf;
      perf.peak_us = vals[0];
      perf.mean_us = vals[1];
      perf.decim = vals[2];
      perf.rate_hz = vals[3];
      perf.timestamp = result.timestamp;
      result.payload = perf;
    } else if (result.length >= 2 && raw[8] == 0x05) {
      // SYSTEM_ORIGIN_CONTROL_DATA: [origin] [n] [18 floats]
      if (result.length == 74) {
        float vals[18];
        memcpy(vals, raw + 10, 72);
        ControlLoopData ctrl;
        ctrl.roll_angle_setpoint = vals[0];
        ctrl.pitch_angle_setpoint = vals[1];
        ctrl.yaw_angle_setpoint = vals[2];
        ctrl.roll_angle_current = vals[3];
        ctrl.pitch_angle_current = vals[4];
        ctrl.yaw_angle_current = vals[5];
        ctrl.roll_rate_setpoint = vals[6];
        ctrl.pitch_rate_setpoint = vals[7];
        ctrl.yaw_rate_setpoint = vals[8];
        ctrl.roll_rate_current = vals[9];
        ctrl.pitch_rate_current = vals[10];
        ctrl.yaw_rate_current = vals[11];
        ctrl.roll_output = vals[12];
        ctrl.pitch_output = vals[13];
        ctrl.yaw_output = vals[14];
        ctrl.throttle_output = vals[15];
        ctrl.outer_dt = vals[16];
        ctrl.inner_dt = vals[17];
        ctrl.timestamp = result.timestamp;
        result.payload = ctrl;
      } else {
        result.payload =
            QString("INVALID CONTROL_DATA LEN: %1").arg(result.length);
      }
    } else if (result.length >= 7 && raw[8] == 0x01) {
      // SYSTEM_ORIGIN_CALIBRATION
      CalibrationUpdate cal;
      cal.type = static_cast<CalibUpdateType>(raw[10]);

      if (cal.type == CalibUpdateType::MagAxisCoverage && result.length >= 15) {
        memcpy(cal.values, raw + 11, 12);
        result.payload = cal;
      } else if (result.length == 7) {
        memcpy(&cal.data, raw + 11, 4);
        result.payload = cal;
      } else {
        // Fallback for legacy progress or other strings
        result.payload = QString::fromLatin1(data.mid(8, result.length));
      }
    } else {
      // Legacy or other status origin: treat as string if small, or generic
      // format
      result.payload = QString::fromLatin1(data.mid(8, result.length));
    }
  } else if (result.type == 0x7) {
    // LOG
    result.payload = QString::fromLatin1(data.mid(8, result.length));
  } else if (result.type == 0x08) {
    // MOTOR_TELEMETRY
    if (result.length == 16) {
      MotorData motor;
      memcpy(motor.speeds, raw + 8, 16);
      result.payload = motor;
    }
  } else if (result.type == 0x09) {
    // PERF_STATS — fragmented; payload set only when the report completes.
    PerfReport report;
    if (decodePerf(raw + 8, result.length, report))
      result.payload = report;
  } else if (result.type == 0x0A) {
    // PERF_TASKNAME reply: [id][name NUL-terminated].
    if (result.length >= 1) {
      TaskNameInfo tn;
      tn.id = raw[8];
      const char *s = reinterpret_cast<const char *>(raw + 9);
      tn.name = QString::fromLatin1(s, qstrnlen(s, result.length - 1));
      result.payload = tn;
    }
  }

  return result;
}

// Wire layout mirrors firmware comm/perf_packet.h. Fragment header (8 bytes):
//   schema(1) section(1) index(1) count(1) seq(4)
// GLOBAL body: flags(1) ntasks(1) nfifos(1) pad(1) then 17 x u32.
// TASK row (20): id,prio,state,pad, stackPeak(2),stackSize(2),
//                cycles(4),switches(4),maxBurst(4).
// FIFO row (8):  id, pad, peak(2), capacity(2), drops(2).
bool PacketDecoder::decodePerf(const uint8_t *p, uint8_t length,
                               PerfReport &out) {
  if (length < 8)
    return false;
  uint8_t section = p[1];
  uint8_t count = p[3];
  uint32_t seq;
  memcpy(&seq, p + 4, 4);
  const uint8_t *body = p + 8;
  int bodyLen = static_cast<int>(length) - 8;

  // A new GLOBAL fragment starts (or restarts) a report.
  if (section == 0x0) { // PERF_SECTION_GLOBAL
    // body = flags/ntasks/nfifos/pad (4) + 17 u32 fields (68) = 72 bytes
    if (bodyLen < 4 + 17 * 4)
      return false;
    PerfReport r;
    r.seq = seq;
    r.enabled = (body[0] & 0x01) != 0;
    auto u32 = [&](int wordIndex) {
      uint32_t v;
      memcpy(&v, body + 4 + wordIndex * 4, 4);
      return v;
    };
    r.uptimeTicks = u32(0);
    r.schedSwitches = u32(1);
    r.cpuCyclesLo = u32(2);
    r.idleCyclesLo = u32(3);
    r.systickCount = u32(4);
    r.systickLastCyc = u32(5);
    r.systickMaxCyc = u32(6);
    r.systickPreemptions = u32(7);
    r.ipcTakes = u32(8);
    r.ipcBlocked = u32(9);
    r.ipcGives = u32(10);
    r.ipcTimeouts = u32(11);
    r.heapAllocs = u32(12);
    r.heapFrees = u32(13);
    r.heapOom = u32(14);
    r.heapPeakBytes = u32(15);
    r.heapTotalBytes = u32(16);
    m_perfAccum = r;
    m_perfHaveGlobal = true;
    // total counts live in body[1]/body[2]; stash them for completion test.
    m_perfAccum.tasks.reserve(body[1]);
    m_perfAccum.fifos.reserve(body[2]);
    m_pendingTasks = body[1];
    m_pendingFifos = body[2];
    return false; // wait for the rows
  }

  if (!m_perfHaveGlobal || seq != m_perfAccum.seq)
    return false; // stray fragment without its GLOBAL — ignore

  if (section == 0x1) { // PERF_SECTION_TASKS
    constexpr int kRow = 20;
    if (bodyLen < count * kRow)
      return false;
    for (int i = 0; i < count; i++) {
      const uint8_t *r = body + i * kRow;
      PerfTaskRow t;
      t.id = r[0];
      t.priority = r[1];
      t.state = r[2];
      memcpy(&t.stackPeak, r + 4, 2);
      memcpy(&t.stackSize, r + 6, 2);
      memcpy(&t.cycles, r + 8, 4);
      memcpy(&t.switches, r + 12, 4);
      memcpy(&t.maxBurst, r + 16, 4);
      m_perfAccum.tasks.push_back(t);
    }
  } else if (section == 0x2) { // PERF_SECTION_FIFOS
    if (bodyLen < count * 8)
      return false;
    for (int i = 0; i < count; i++) {
      const uint8_t *r = body + i * 8;
      PerfFifoRow f;
      f.id = r[0];
      memcpy(&f.peak, r + 2, 2);
      memcpy(&f.capacity, r + 4, 2);
      memcpy(&f.drops, r + 6, 2);
      m_perfAccum.fifos.push_back(f);
    }
  } else {
    return false;
  }

  // Report complete once both row sets have arrived.
  if (m_perfAccum.tasks.size() >= m_pendingTasks &&
      m_perfAccum.fifos.size() >= m_pendingFifos) {
    out = m_perfAccum;
    m_perfHaveGlobal = false;
    return true;
  }
  return false;
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
  case 0x8:
    return "MOTOR_TELEMETRY";
  case 0x9:
    return "PERF_STATS";
  case 0xA:
    return "PERF_TASKNAME";
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
