#include "CommandCodec.h"

#include "crc.h"

extern "C" {
#include "navlink_msgs.h"  // NavLink v2 generated codec
}

namespace CommandCodec {

QByteArray encodeCommand(quint16 cmdId, const QVector<float> &args,
                         quint8 devId, quint32 tsMs) {
  QByteArray payload;
  payload.append(reinterpret_cast<const char *>(&cmdId), 2);  // cmd_id (LE)
  payload.append(static_cast<char>(args.size() & 0xFF));      // argc
  for (float a : args)
    payload.append(reinterpret_cast<const char *>(&a), 4);    // arg (LE f32)

  QByteArray pkt;
  pkt.append(static_cast<char>(0x56));                          // sync
  pkt.append(static_cast<char>(0x31));                          // type 3 | v1
  pkt.append(static_cast<char>(payload.size() & 0xFF));         // length
  pkt.append(static_cast<char>(devId));                         // device id
  pkt.append(reinterpret_cast<const char *>(&tsMs), 4);         // timestamp
  pkt.append(payload);
  const uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()),
      static_cast<uint32_t>(pkt.size()));
  pkt.append(reinterpret_cast<const char *>(&crc), 4);
  return pkt;
}

QByteArray encodeSetPid(int controller, int axis, float kp, float ki, float kd,
                        float kff, quint8 devId, quint32 tsMs) {
  // NavLink v2 (navlink/INTEGRATION.md, Phase 3): emit a typed CMD_SET_PID frame
  // instead of the v1 generic COMMAND. The FC correlates the COMMAND_ACK by
  // req_seq. (v2 frames carry no header timestamp; tsMs is unused.)
  Q_UNUSED(tsMs);
  static uint8_t s_seq = 0;
  navlink_cmd_set_pid_t m{};
  m.target_sys = devId;     // address the FC (v1 used device id 42)
  m.target_comp = 1;
  m.req_seq = s_seq;
  m.controller = static_cast<uint8_t>(controller);
  m.axis = static_cast<uint8_t>(axis);
  m.kp = kp;
  m.ki = ki;
  m.kd = kd;
  m.kff = kff;
  uint8_t buf[NAVLINK_MAX_FRAME];
  size_t n = navlink_cmd_set_pid_encode(buf, &m, s_seq++, /*sysid (GCS)*/ 0xFF,
                                        /*compid*/ 1);
  return QByteArray(reinterpret_cast<const char *>(buf), static_cast<int>(n));
}

QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId, quint32 tsMs) {
  return encodeCommand(kCmdSetGyroLpf, {float(axis), rc}, devId, tsMs);
}

QByteArray encodeSetFlightMode(int mode, quint8 devId, quint32 tsMs) {
  return encodeCommand(kCmdSetFlightMode, {float(mode)}, devId, tsMs);
}

QByteArray encodeSetMotorGeometry(const float x[4], const float y[4],
                                  const float spin[4], quint8 devId,
                                  quint32 tsMs) {
  QVector<float> args;
  for (int i = 0; i < 4; ++i) args.append(x[i]);
  for (int i = 0; i < 4; ++i) args.append(y[i]);
  for (int i = 0; i < 4; ++i) args.append(spin[i]);
  return encodeCommand(kCmdSetMotorGeometry, args, devId, tsMs);
}

}  // namespace CommandCodec
