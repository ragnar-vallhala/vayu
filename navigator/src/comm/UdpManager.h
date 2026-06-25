#pragma once

#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUdpSocket>

/**
 * @brief UDP telemetry source — the network twin of SerialManager.
 *
 * Binds a local UDP port and emits each received datagram's bytes via
 * dataReceived(), so DroneProtocol parses a WiFi/UDP stream (from the ESP8266
 * telemetry bridge) exactly like the serial stream. On bind it sends a small
 * "hello" datagram to the broadcast address so the bridge can learn this GCS's
 * address and switch from broadcast to unicast.
 */
class UdpManager : public QObject {
  Q_OBJECT

public:
  explicit UdpManager(QObject *parent = nullptr);

  bool bind(quint16 port);
  void close();
  bool isOpen() const;
  quint16 port() const { return m_port; }

  // Send a command frame back to the bridge (GCS -> FC). Unicasts to the peer
  // learned from received telemetry; broadcasts until one is seen. Mirrors
  // SerialManager::write so MainWindow can route to either transport.
  bool write(const QByteArray &data);

signals:
  void dataReceived(const QByteArray &data);
  void dataSent(const QByteArray &data); // mirrors SerialManager::dataSent
  void connectionStateChanged(bool connected);
  void errorOccurred(const QString &message);

private slots:
  void onReadyRead();

private:
  QUdpSocket *m_sock = nullptr;
  quint16 m_port = 0;
  QHostAddress m_peer;       // bridge address, learned from received telemetry
  bool m_havePeer = false;
  QTimer *m_hello = nullptr;  // periodic re-announce so the bridge keeps unicasting
};
