#include "CommandCodec.h"

#include "crc.h"

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
  return encodeCommand(kCmdSetPid,
                       {float(controller), float(axis), kp, ki, kd, kff}, devId,
                       tsMs);
}

QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId, quint32 tsMs) {
  return encodeCommand(kCmdSetGyroLpf, {float(axis), rc}, devId, tsMs);
}

QByteArray encodeSetFlightMode(int mode, quint8 devId, quint32 tsMs) {
  return encodeCommand(kCmdSetFlightMode, {float(mode)}, devId, tsMs);
}

}  // namespace CommandCodec
