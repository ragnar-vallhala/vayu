#include <QtTest>
#include <cstring>

#include "CommandCodec.h"
#include "crc.h"

extern "C" {
#include "navlink_msgs.h"  // decode the emitted v2 frames with the real parser
}

// CMD_SET_PID now emits a NavLink v2 frame (10-byte header + truncated payload +
// CRC-16); the still-v1 commands (gyro-LPF, flight-mode, motor-geometry) keep
// the [cmd_id][argc][args] + CRC32 layout. The SetPid cases decode the frame
// through the generated parser (which validates the CRC and zero-fills any
// trailing-zero-truncated payload) and assert the decoded fields, rather than
// asserting value-dependent byte offsets.
class TstCommandCodec : public QObject {
  Q_OBJECT

private slots:
  void setPidFrameLayout();
  void crcRejectsCorruptedFrame();
  void setPidGainsRoundTrip();
  void gyroLpfAndFlightModeIds();
  void setGyroNotchRoundTrip();
};

namespace {
navlink_cmd_set_pid_t g_pid;
bool g_pidGot;
// Command handlers now return their COMMAND_ACK result (codegen enforces the ack).
navlink_ack_t onPid(void *, const navlink_frame_hdr_t *,
                    const navlink_cmd_set_pid_t *m) {
  g_pid = *m;
  g_pidGot = true;
  return navlink_ack_t{};
}
// Push a whole v2 frame through the parser; returns true (and fills out) only if
// it decoded a CRC-valid CMD_SET_PID.
bool decodePid(const QByteArray &f, navlink_cmd_set_pid_t &out) {
  g_pidGot = false;
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_handlers_t h{};
  h.on_cmd_set_pid = onPid;
  navlink_parser_push(&p, &h,
                      reinterpret_cast<const uint8_t *>(f.constData()),
                      size_t(f.size()));
  if (g_pidGot)
    out = g_pid;
  return g_pidGot;
}
}  // namespace

void TstCommandCodec::setPidFrameLayout() {
  // rate controller (1), pitch (1), arbitrary gains; kff=0 is trimmed on the
  // wire (trailing-zero truncation), so the frame size is value-dependent.
  const QByteArray f = CommandCodec::encodeSetPid(1, 1, 0.10f, 0.02f, 0.003f,
                                                  0.0f, /*dev*/ 42, /*ts*/ 7);
  QCOMPARE(quint8(f[0]), quint8(0x56));          // sync
  QCOMPARE(quint8(f[1]), quint8(0x02));          // NavLink v2 version byte
  QCOMPARE(quint8(f[2]), quint8(f.size() - 12)); // payload_len == size - hdr - crc
  const quint32 msgid = quint8(f[7]) | (quint32(quint8(f[8])) << 8) |
                        (quint32(quint8(f[9])) << 16);  // u24 LE
  QCOMPARE(msgid, quint32(NAVLINK_MSGID_CMD_SET_PID));  // 8195
  navlink_cmd_set_pid_t m{};
  QVERIFY(decodePid(f, m));
  QCOMPARE(quint8(m.target_sys), quint8(42));    // dev id addressed
  QCOMPARE(quint8(m.controller), quint8(1));     // rate loop
  QCOMPARE(quint8(m.axis), quint8(1));           // pitch
}

void TstCommandCodec::crcRejectsCorruptedFrame() {
  QByteArray f = CommandCodec::encodeSetPid(1, 0, 1.0f, 2.0f, 3.0f, 4.0f);
  navlink_cmd_set_pid_t m{};
  QVERIFY(decodePid(f, m));                  // intact frame decodes (CRC valid)
  f[10] = char(quint8(f[10]) ^ 0xFF);        // flip a payload byte
  QVERIFY(!decodePid(f, m));                 // CRC now fails -> no dispatch
}

void TstCommandCodec::setPidGainsRoundTrip() {
  const QByteArray f = CommandCodec::encodeSetPid(
      /*controller=rate*/ 1, /*axis=yaw*/ 2, 0.5f, 0.25f, 0.125f, 0.0625f);
  navlink_cmd_set_pid_t m{};
  QVERIFY(decodePid(f, m));
  QCOMPARE(quint8(m.controller), quint8(1));  // rate
  QCOMPARE(quint8(m.axis), quint8(2));        // yaw
  QCOMPARE(m.kp, 0.5f);
  QCOMPARE(m.ki, 0.25f);
  QCOMPARE(m.kd, 0.125f);
  QCOMPARE(m.kff, 0.0625f);
}

// Generic: assert the frame's msgid (u24 LE at [7..9]) and that it decodes
// (CRC-valid) through the parser.
static bool decodesAs(const QByteArray &f, quint32 wantMsgid,
                      const navlink_handlers_t &h) {
  const quint32 msgid = quint8(f[7]) | (quint32(quint8(f[8])) << 8) |
                        (quint32(quint8(f[9])) << 16);
  if (msgid != wantMsgid)
    return false;
  navlink_parser_t p;
  navlink_parser_init(&p);
  navlink_parser_push(&p, &h, reinterpret_cast<const uint8_t *>(f.constData()),
                      size_t(f.size()));
  return true;
}

namespace {
navlink_cmd_set_gyro_lpf_t g_lpf;
bool g_lpfGot;
navlink_ack_t onLpf(void *, const navlink_frame_hdr_t *,
                    const navlink_cmd_set_gyro_lpf_t *m) {
  g_lpf = *m;
  g_lpfGot = true;
  return navlink_ack_t{};
}
navlink_cmd_set_flight_mode_t g_fm;
bool g_fmGot;
navlink_ack_t onFm(void *, const navlink_frame_hdr_t *,
                   const navlink_cmd_set_flight_mode_t *m) {
  g_fm = *m;
  g_fmGot = true;
  return navlink_ack_t{};
}
navlink_cmd_set_motor_geometry_t g_geo;
bool g_geoGot;
navlink_ack_t onGeo(void *, const navlink_frame_hdr_t *,
                    const navlink_cmd_set_motor_geometry_t *m) {
  g_geo = *m;
  g_geoGot = true;
  return navlink_ack_t{};
}
}  // namespace

void TstCommandCodec::gyroLpfAndFlightModeIds() {
  navlink_handlers_t h{};
  g_lpfGot = g_fmGot = g_geoGot = false;
  h.on_cmd_set_gyro_lpf = onLpf;
  h.on_cmd_set_flight_mode = onFm;
  h.on_cmd_set_motor_geometry = onGeo;

  QVERIFY(decodesAs(CommandCodec::encodeSetGyroLpf(2, 0.004f),
                    NAVLINK_MSGID_CMD_SET_GYRO_LPF, h));
  QVERIFY(g_lpfGot);
  QCOMPARE(quint8(g_lpf.axis), quint8(2));
  QCOMPARE(g_lpf.rc, 0.004f);

  QVERIFY(decodesAs(CommandCodec::encodeSetFlightMode(1),
                    NAVLINK_MSGID_CMD_SET_FLIGHT_MODE, h));
  QVERIFY(g_fmGot);
  QCOMPARE(quint8(g_fm.mode), quint8(1));    // acro
  QCOMPARE(quint8(g_fm.source), quint8(1));  // GCS

  const float x[4] = {0.1f, -0.1f, -0.1f, 0.1f};
  const float y[4] = {0.1f, 0.1f, -0.1f, -0.1f};
  const float sp[4] = {1.f, -1.f, 1.f, -1.f};
  QVERIFY(decodesAs(CommandCodec::encodeSetMotorGeometry(x, y, sp),
                    NAVLINK_MSGID_CMD_SET_MOTOR_GEOMETRY, h));
  QVERIFY(g_geoGot);
  QCOMPARE(g_geo.pos_x[0], 0.1f);
  QCOMPARE(int(g_geo.spin[0]), 1);
  QCOMPARE(int(g_geo.spin[1]), -1);
}

namespace {
navlink_cmd_set_gyro_notch_t g_notch;
bool g_notchGot;
navlink_ack_t onNotch(void *, const navlink_frame_hdr_t *,
                      const navlink_cmd_set_gyro_notch_t *m) {
  g_notch = *m;
  g_notchGot = true;
  return navlink_ack_t{};
}
}  // namespace

void TstCommandCodec::setGyroNotchRoundTrip() {
  navlink_handlers_t h{};
  g_notchGot = false;
  h.on_cmd_set_gyro_notch = onNotch;

  QVERIFY(decodesAs(
      CommandCodec::encodeSetGyroNotch(true, 8.0f, 60.0f, 450.0f, 4.0f),
      NAVLINK_MSGID_CMD_SET_GYRO_NOTCH, h));
  QVERIFY(g_notchGot);
  QCOMPARE(quint8(g_notch.enabled), quint8(1));
  QCOMPARE(g_notch.q, 8.0f);
  QCOMPARE(g_notch.fmin_hz, 60.0f);
  QCOMPARE(g_notch.fmax_hz, 450.0f);
  QCOMPARE(g_notch.min_ratio, 4.0f);
}

QTEST_APPLESS_MAIN(TstCommandCodec)
#include "tst_command_codec.moc"
