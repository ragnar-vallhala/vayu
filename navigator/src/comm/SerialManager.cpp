#include "SerialManager.h"

#include <QSerialPortInfo>

SerialManager::SerialManager(QObject *parent) : QObject(parent) {
  connect(&m_port, &QSerialPort::readyRead, this, &SerialManager::onReadyRead);
  connect(&m_port, &QSerialPort::errorOccurred, this,
          &SerialManager::onErrorOccurred);
  m_retryTimer.setSingleShot(true);
  connect(&m_retryTimer, &QTimer::timeout, this, &SerialManager::tryReconnect);
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

  // Successful open — remember these for auto-reconnect, reset the
  // retry counter so the next transient failure starts a fresh ramp.
  m_lastPort = portName;
  m_lastBaud = baudRate;
  m_retryCount = 0;
  m_userClose = false;

  emit connectionStateChanged(true);
  return true;
}

void SerialManager::close() {
  // UI-driven close — suppress auto-reconnect so a deliberate
  // Disconnect doesn't immediately bounce back.
  m_userClose = true;
  m_retryTimer.stop();
  if (m_port.isOpen()) {
    m_port.close();
    emit connectionStateChanged(false);
  }
}

void SerialManager::tryReconnect() {
  if (m_userClose || m_lastPort.isEmpty())
    return;
  if (m_retryCount >= kMaxRetries) {
    emit errorOccurred(
        tr("Auto-reconnect: gave up after %1 attempts").arg(kMaxRetries));
    return;
  }
  ++m_retryCount;
  emit reconnectAttempt(m_retryCount, kMaxRetries);
  if (!open(m_lastPort, m_lastBaud)) {
    // open() emitted errorOccurred for us. Schedule the next attempt
    // with exponential backoff capped at kMaxDelayMs.
    const int delay =
        std::min(m_initialDelayMs * (1 << (m_retryCount - 1)), kMaxDelayMs);
    m_retryTimer.start(delay);
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
  // Schedule a reconnect if the user wants one and didn't ask for the
  // disconnect themselves.
  if (m_autoReconnect && !m_userClose && !m_lastPort.isEmpty()) {
    if (!m_retryTimer.isActive())
      m_retryTimer.start(m_initialDelayMs);
  }
}
