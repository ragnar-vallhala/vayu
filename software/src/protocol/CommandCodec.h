#pragma once

#include <QByteArray>
#include <QVector>
#include <QtGlobal>

// Builds NavLink command frames (GCS -> FC, packet type 3) — the outbound twin
// of DroneProtocol's rx parser. Pure + static so frame construction is shared
// and unit-testable instead of hand-rolled at each call site.
//
// Frame: [0x56 sync][type|ver = 0x31][len:u8][dev_id:u8][ts:u32 LE]
//        [payload][crc32:u32 LE]
// where payload = [cmd_id:u16 LE][argc:u8][arg:f32 LE]... and crc32 covers
// every byte up to (not including) the crc. Mirrors the inline frames in
// MainWindow and the firmware's comm_processor / pid_config schema.
namespace CommandCodec {

// Command ids (mirror firmware include/comm/comm_types.h).
inline constexpr quint16 kCmdSetPid = 0x000A;
inline constexpr quint16 kCmdSetGyroLpf = 0x000B;
inline constexpr quint16 kCmdSetMotorGeometry = 0x000C;
inline constexpr quint16 kCmdSetFlightMode = 0x000D;

// Generic command frame with float args.
QByteArray encodeCommand(quint16 cmdId, const QVector<float> &args,
                         quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_PID for one (controller, axis): controller 0=angle / 1=rate,
// axis 0=roll / 1=pitch / 2=yaw. argc = 6 (controller, axis, Kp, Ki, Kd, Kff).
QByteArray encodeSetPid(int controller, int axis, float kp, float ki, float kd,
                        float kff, quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_GYRO_LPF (0x000B): rate-loop gyro low-pass for one axis. argc = 2
// (axis, rc[s]; rc<=0 disables).
QByteArray encodeSetGyroLpf(int axis, float rc, quint8 devId = 42,
                            quint32 tsMs = 0);

// CMD_SET_FLIGHT_MODE (0x000D): 0=stabilise/angle, 1=acro, 2=release to RC.
QByteArray encodeSetFlightMode(int mode, quint8 devId = 42, quint32 tsMs = 0);

// CMD_SET_MOTOR_GEOMETRY (0x000C): set the firmware mixer signs from the motor
// layout so the control mix matches the airframe. argc = 12, ordered x[4], y[4],
// spin[4] (motor body x/y [m] and spin +1 CCW / -1 CW), per motor 0..3.
QByteArray encodeSetMotorGeometry(const float x[4], const float y[4],
                                  const float spin[4], quint8 devId = 42,
                                  quint32 tsMs = 0);

}  // namespace CommandCodec
