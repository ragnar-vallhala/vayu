#include <QtTest>
#include <cstring>

#include "CommandCodec.h"
#include "crc.h"

// CMD_SET_PID codec: the emitted frame must match the firmware's NavLink +
// pid_config schema exactly (sync/type/len/dev/ts header, [cmd_id][argc][args]
// payload, trailing CRC32 over everything before it).
class TstCommandCodec : public QObject {
  Q_OBJECT

private slots:
  void setPidFrameLayout();
  void crcCoversHeaderAndPayload();
  void argcAndArgsRoundTrip();
  void gyroLpfAndFlightModeIds();
};

static float readF32(const QByteArray &b, int off) {
  float f;
  std::memcpy(&f, b.constData() + off, 4);
  return f;
}
static quint32 readU32(const QByteArray &b, int off) {
  quint32 v;
  std::memcpy(&v, b.constData() + off, 4);
  return v;
}
static quint16 readU16(const QByteArray &b, int off) {
  quint16 v;
  std::memcpy(&v, b.constData() + off, 2);
  return v;
}

void TstCommandCodec::setPidFrameLayout() {
  // rate controller (1), pitch (1), arbitrary gains.
  const QByteArray f = CommandCodec::encodeSetPid(1, 1, 0.10f, 0.02f, 0.003f,
                                                  0.0f, /*dev*/ 42, /*ts*/ 7);
  // header(8) + payload(2+1+6*4=27) + crc(4) = 39
  QCOMPARE(f.size(), 39);
  QCOMPARE(quint8(f[0]), quint8(0x56));        // sync
  QCOMPARE(quint8(f[1]), quint8(0x31));        // packet type 3 | proto v1
  QCOMPARE(quint8(f[2]), quint8(27));          // payload length
  QCOMPARE(quint8(f[3]), quint8(42));          // device id
  QCOMPARE(readU32(f, 4), quint32(7));         // timestamp
  QCOMPARE(readU16(f, 8), quint16(0x000A));    // cmd_id == CMD_SET_PID
  QCOMPARE(quint8(f[10]), quint8(6));          // argc == PID_SET_ARGC
}

void TstCommandCodec::crcCoversHeaderAndPayload() {
  const QByteArray f =
      CommandCodec::encodeSetPid(1, 0, 1.0f, 2.0f, 3.0f, 4.0f);
  const int crcOff = f.size() - 4;
  const quint32 want = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(f.constData()), quint32(crcOff));
  QCOMPARE(readU32(f, crcOff), want);
}

void TstCommandCodec::argcAndArgsRoundTrip() {
  const QByteArray f = CommandCodec::encodeSetPid(
      /*controller=rate*/ 1, /*axis=yaw*/ 2, 0.5f, 0.25f, 0.125f, 0.0625f);
  // args start at offset 11 (after cmd_id + argc).
  QCOMPARE(readF32(f, 11), 1.0f);   // controller
  QCOMPARE(readF32(f, 15), 2.0f);   // axis
  QCOMPARE(readF32(f, 19), 0.5f);   // Kp
  QCOMPARE(readF32(f, 23), 0.25f);  // Ki
  QCOMPARE(readF32(f, 27), 0.125f); // Kd
  QCOMPARE(readF32(f, 31), 0.0625f);// Kff
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
}

QTEST_APPLESS_MAIN(TstCommandCodec)
#include "tst_command_codec.moc"
