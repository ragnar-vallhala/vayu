#pragma once

#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QTimer>

class SerialManager : public QObject {
  Q_OBJECT

public:
  explicit SerialManager(QObject *parent = nullptr);
  ~SerialManager() override;

  bool open(const QString &portName, qint32 baudRate);
  void close();
  bool write(const QByteArray &data);

  bool isOpen() const;
  QString currentPort() const { return m_port.portName(); }

  // Auto-reconnect on transient serial errors (FR-CONN-05 / FR-UX-?).
  // When enabled and the port closes due to an error, we schedule a
  // re-open on the last (port, baud) with a small backoff. Capped at
  // kMaxRetries to avoid spinning if the device is genuinely gone.
  void setAutoReconnect(bool on) { m_autoReconnect = on; }
  bool autoReconnect() const     { return m_autoReconnect; }

  // Static helpers
  static QStringList availablePorts();

signals:
  void dataReceived(const QByteArray &data);
  void dataSent(const QByteArray &data);
  void connectionStateChanged(bool connected);
  void errorOccurred(const QString &message);
  // Optional UX hook — fires before each scheduled retry.
  void reconnectAttempt(int attempt, int maxAttempts);

private slots:
  void onReadyRead();
  void onErrorOccurred(QSerialPort::SerialPortError error);
  void tryReconnect();

private:
  QSerialPort m_port;
  QByteArray m_buffer;

  // Auto-reconnect bookkeeping.
  bool    m_autoReconnect = false;
  bool    m_userClose     = false;   // true when close() came from UI
  QString m_lastPort;
  qint32  m_lastBaud      = 0;
  int     m_retryCount    = 0;
  QTimer  m_retryTimer;

  static constexpr int kInitialDelayMs = 1000;
  static constexpr int kMaxDelayMs     = 8000;
  static constexpr int kMaxRetries     = 5;
};
