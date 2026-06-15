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
};

static float readF32(const QByteArray &b, int off) {
  float f;
  std::memcpy(&f, b.constData() + off, 4);
  return f;
}
static quint16 readU16(const QByteArray &b, int off) {
  quint16 v;
  std::memcpy(&v, b.constData() + off, 2);
  return v;
}

namespace {
navlink_cmd_set_pid_t g_pid;
bool g_pidGot;
void onPid(void *, const navlink_frame_hdr_t *,
           const navlink_cmd_set_pid_t *m) {
  g_pid = *m;
  g_pidGot = true;
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

void TstCommandCodec::gyroLpfAndFlightModeIds() {
  const QByteArray g = CommandCodec::encodeSetGyroLpf(2, 0.004f);
  QCOMPARE(readU16(g, 8), quint16(0x000B));  // CMD_SET_GYRO_LPF
  QCOMPARE(quint8(g[10]), quint8(2));        // argc = 2 (axis, rc)
  QCOMPARE(readF32(g, 11), 2.0f);            // axis
  QCOMPARE(readF32(g, 15), 0.004f);          // rc

  const QByteArray m = CommandCodec::encodeSetFlightMode(1);
  QCOMPARE(readU16(m, 8), quint16(0x000D));  // CMD_SET_FLIGHT_MODE
  QCOMPARE(quint8(m[10]), quint8(1));        // argc = 1
  QCOMPARE(readF32(m, 11), 1.0f);            // mode = acro

  const float x[4] = {0.1f, -0.1f, -0.1f, 0.1f};
  const float y[4] = {0.1f, 0.1f, -0.1f, -0.1f};
  const float sp[4] = {1.f, -1.f, 1.f, -1.f};
  const QByteArray g2 = CommandCodec::encodeSetMotorGeometry(x, y, sp);
  QCOMPARE(readU16(g2, 8), quint16(0x000C));  // CMD_SET_MOTOR_GEOMETRY
  QCOMPARE(quint8(g2[10]), quint8(12));       // argc = 12 (x[4],y[4],spin[4])
  QCOMPARE(readF32(g2, 11), 0.1f);            // x0
  QCOMPARE(readF32(g2, 11 + 8 * 4), 1.0f);    // spin0 (after x[4],y[4])
}

QTEST_APPLESS_MAIN(TstCommandCodec)
#include "tst_command_codec.moc"
