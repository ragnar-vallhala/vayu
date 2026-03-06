#include "SerialManager.h"

#include <QSerialPortInfo>

SerialManager::SerialManager(QObject *parent) : QObject(parent) {
  connect(&m_port, &QSerialPort::readyRead, this, &SerialManager::onReadyRead);
  connect(&m_port, &QSerialPort::errorOccurred, this,
          &SerialManager::onErrorOccurred);
}

SerialManager::~SerialManager() { close(); }

bool SerialManager::open(const QString &portName, qint32 baudRate) {
  if (m_port.isOpen()) {
    m_port.close();
  }
  m_buffer.clear();

  m_port.setPortName(portName);
  m_port.setBaudRate(baudRate);
  m_port.setDataBits(QSerialPort::Data8);
  m_port.setParity(QSerialPort::NoParity);
  m_port.setStopBits(QSerialPort::OneStop);
  m_port.setFlowControl(QSerialPort::NoFlowControl);

  if (!m_port.open(QIODevice::ReadWrite)) {
    emit errorOccurred(
        tr("Cannot open %1: %2").arg(portName, m_port.errorString()));
    return false;
  }

  emit connectionStateChanged(true);
  return true;
}

void SerialManager::close() {
  if (m_port.isOpen()) {
    m_port.close();
    emit connectionStateChanged(false);
  }
}

bool SerialManager::write(const QByteArray &data) {
  if (!m_port.isOpen())
    return false;
  bool ok = m_port.write(data) == data.size();
  if (ok) {
    emit dataSent(data);
  }
  return ok;
}

bool SerialManager::isOpen() const { return m_port.isOpen(); }

QStringList SerialManager::availablePorts() {
  QStringList names;
  const auto ports = QSerialPortInfo::availablePorts();
  for (const auto &info : ports) {
    names << info.portName();
  }
  return names;
}

// ---------------------------------------------------------------------------
// Private slots
// ---------------------------------------------------------------------------
void SerialManager::onReadyRead() {
  QByteArray data = m_port.readAll();
  if (!data.isEmpty()) {
    emit dataReceived(data);
  }
}

void SerialManager::onErrorOccurred(QSerialPort::SerialPortError error) {
  if (error == QSerialPort::NoError)
    return;

  emit errorOccurred(m_port.errorString());
  if (m_port.isOpen()) {
    m_port.close();
    emit connectionStateChanged(false);
  }
}
