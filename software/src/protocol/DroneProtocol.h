#pragma once

#include "NavlinkRouter.h"
#include "PacketDecoder.h"
#include "Types.h"
#include <QByteArray>
#include <QObject>
#include <QString>

/**
 * Parses newline-terminated ASCII telemetry packets emitted by the Vayu
 * firmware over UART.
 *
 * Packet formats:
 *   $IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<tempC>
 *   $ATT,<roll>,<pitch>,<yaw>
 *   $LOG,<message text>
 *
 * The firmware should send these using v_log or a direct uart_write call.
 */
class DroneProtocol : public QObject {
  Q_OBJECT

public:
  explicit DroneProtocol(QObject *parent = nullptr);

public slots:
  void processData(const QByteArray &data);
  // Advanced setting: when false, packets that fail CRC are accepted anyway
  // (for debugging a flaky link). Defaults to enforcing CRC. Lives on the
  // worker thread; drive it via a queued invoke.
  void setCrcCheck(bool on) { m_checkCrc = on; }

signals:
  void imuReceived(const ImuData &data);
  void attitudeReceived(const AttitudeData &data);
  void rcReceived(const RcData &data);
  void motorReceived(const MotorData &data);
  void controlLoopDataReceived(const ControlLoopData &data);
  void estPerfReceived(const EstPerfData &data);
  void flightModeReceived(quint8 mode, quint8 source);
  void logReceived(const QString &message);
  void statusReceived(const QString &message);
  void calibrationUpdateReceived(const CalibrationUpdate &update);
  void heartbeatReceived(uint64_t timestamp, uint8_t deviceId);
  // Time-sync RESPONSE (0xB): the four NTP timestamps (t4 captured on receipt,
  // on the worker thread). MainWindow feeds these to TimeSyncEstimator.
  void timeSyncResponse(quint8 seq, quint64 t1, quint64 t2, quint64 t3,
                        quint64 t4);
  void perfReceived(const PerfReport &report);
  void taskNameReceived(int taskId, const QString &name);
  void timeSyncRequested();
  void unknownPacket(const QByteArray &raw);
  void packetReceived(const QByteArray &packet);

private:
  QByteArray m_buffer;
  PacketDecoder m_decoder;
  bool m_checkCrc = true;  // Advanced ▸ CRC checking
  void parseBuffer();

  // NavLink v2 receive path (navlink/INTEGRATION.md). parseBuffer() demuxes v2
  // frames (byte1 == 0x02) off the shared byte stream and hands them to the
  // router, which decodes + dispatches. Its hooks (set in our ctor) re-emit the
  // existing Qt signals, so TelemetryEngine / VehicleState / the UI are
  // unchanged. The generated codec lives only inside NavlinkRouter.
  NavlinkRouter m_v2Router;
};
