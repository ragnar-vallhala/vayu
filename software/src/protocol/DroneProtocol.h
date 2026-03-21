#pragma once

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

signals:
  void imuReceived(const ImuData &data);
  void attitudeReceived(const AttitudeData &data);
  void rcReceived(const RcData &data);
  void logReceived(const QString &message);
  void statusReceived(const QString &message);
  void calibrationUpdateReceived(const CalibrationUpdate &update);
  void heartbeatReceived(uint64_t timestamp, uint8_t deviceId);
  void timeSyncRequested();
  void unknownPacket(const QByteArray &raw);
  void packetReceived(const QByteArray &packet);

private:
  QByteArray m_buffer;
  PacketDecoder m_decoder;
  void parseBuffer();
};
