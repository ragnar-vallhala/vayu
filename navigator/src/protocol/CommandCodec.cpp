#include "CommandCodec.h"

extern "C" {
#include "navlink_msgs.h" // NavLink v2 generated codec
}

// Every command is now a typed NavLink v2 frame. Each encoder mirrors the
// firmware's navlink_router.c handler: target_sys addresses the FC (the former
// v1 device id), target_comp = 1, a rolling req_seq correlates the COMMAND_ACK.
// The generated *_encode() builds the whole frame (10-byte header + truncated
// payload + CRC-16). tsMs is vestigial (v2 frames carry no header timestamp).
namespace CommandCodec {
namespace {
uint8_t nextSeq() {
  static uint8_t s = 0;
  return s++;
}
QByteArray frame(const uint8_t *buf, size_t n) {
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}
} // namespace

QByteArray encodeSetPid(int controller, int axis, float kp, float ki, float kd,
                        float kff, quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_set_pid_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.controller = static_cast<uint8_t>(controller);
  m.axis = static_cast<uint8_t>(axis);
  m.kp = kp;
  m.ki = ki;
  m.kd = kd;
  m.kff = kff;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_pid_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_set_gyro_lpf_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.axis = static_cast<uint8_t>(axis);
  m.rc = rc;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_gyro_lpf_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeSetGyroNotch(bool enabled, float q, float fminHz, float fmaxHz,
                              float minRatio, bool autoband, quint8 devId,
                              quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_set_gyro_notch_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.enabled = enabled ? 1 : 0;
  m.q = q;
  m.fmin_hz = fminHz;
  m.fmax_hz = fmaxHz;
  m.min_ratio = minRatio;
  m.autoband = autoband ? 1 : 0;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_gyro_notch_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeSetFlightMode(int mode, quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_set_flight_mode_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.mode = static_cast<uint8_t>(mode);
  m.source = 1; // mode_source GCS
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_flight_mode_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeSetMotorGeometry(const float x[4], const float y[4],
                                  const float spin[4], quint8 devId,
                                  quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_set_motor_geometry_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.layout = 0;
  for (int i = 0; i < 4; ++i) {
    m.pos_x[i] = x[i];
    m.pos_y[i] = y[i];
    m.spin[i] = static_cast<int8_t>(spin[i]);
  }
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_motor_geometry_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeArm(quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_arm_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.force = 0;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_arm_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeDisarm(quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_cmd_disarm_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.force = 0;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_disarm_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeCalibrate(quint8 which, quint8 devId) {
  // which selects the calibration routine; 0xFF cancels. Single IMU (id 0), so
  // the firmware ignores imu_id. Cancel maps to the firmware's stop path.
  navlink_cmd_calibrate_imu_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.req_seq = nextSeq();
  m.which = which;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_calibrate_imu_encode(buf, &m, m.req_seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeTimeSyncRequest(quint8 seq, quint64 t1,
                                 qint32 commandedOffsetMs, quint8 devId) {
  Q_UNUSED(devId);
  navlink_time_sync_t m{};
  m.role = 0; // REQUEST
  m.seq = seq;
  m.t1_gcs_tx = t1;
  m.t2_fc_rx = 0;
  m.t3_fc_tx = 0;
  m.commanded_offset_ms = commandedOffsetMs;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_time_sync_encode(buf, &m, seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeTimeSyncRequestWide(quint8 seq, quint64 t1,
                                     qint64 commandedOffsetMs, quint8 devId) {
  Q_UNUSED(devId);
  navlink_time_sync_t m{};
  m.role = 2; // REQUEST_WIDE
  m.seq = seq;
  m.t1_gcs_tx = t1;
  m.t2_fc_rx = 0;
  m.t3_fc_tx = 0;
  // 64-bit correction split low/high; the FC reconstructs (hi<<32)|lo.
  m.commanded_offset_ms = static_cast<qint32>(static_cast<quint32>(
      static_cast<quint64>(commandedOffsetMs) & 0xFFFFFFFFu));
  m.commanded_offset_hi_ms =
      static_cast<qint32>(static_cast<quint64>(commandedOffsetMs) >> 32);
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_time_sync_encode(buf, &m, seq, 0xFF, 1);
  return frame(buf, n);
}

QByteArray encodeTaskNameRequest(int taskId, quint8 devId, quint32 tsMs) {
  Q_UNUSED(tsMs);
  navlink_perf_taskname_request_t m{};
  m.target_sys = devId;
  m.target_comp = 1;
  m.task_id = static_cast<uint8_t>(taskId);
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_perf_taskname_request_encode(buf, &m, nextSeq(), 0xFF, 1);
  return frame(buf, n);
}

} // namespace CommandCodec
