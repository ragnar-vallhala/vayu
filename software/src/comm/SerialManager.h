#pragma once

#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include <QString>

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

  // Static helpers
  static QStringList availablePorts();

signals:
  void packetReceived(const QByteArray &line);
  void connectionStateChanged(bool connected);
  void errorOccurred(const QString &message);

private slots:
  void onReadyRead();
  void onErrorOccurred(QSerialPort::SerialPortError error);

private:
  QSerialPort m_port;
  QByteArray m_buffer;
};
