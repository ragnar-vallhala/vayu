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
  return m_port.write(data) == data.size();
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
  m_buffer += m_port.readAll();

  // Emit each complete newline-terminated line
  while (true) {
    int idx = m_buffer.indexOf('\n');
    if (idx == -1)
      break;

    QByteArray line = m_buffer.left(idx + 1).trimmed();
    m_buffer.remove(0, idx + 1);

    if (!line.isEmpty()) {
      emit packetReceived(line);
    }
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
