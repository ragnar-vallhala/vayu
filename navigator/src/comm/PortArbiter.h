#pragma once

#include <QHash>
#include <QObject>
#include <QString>

// PortArbiter — single source of truth for which subsystem owns a given serial
// device. The board telemetry (SerialManager) and the in-sim RC bridge
// (RcBridge) can both target the same tty (e.g. /dev/ttyUSB0); opening it from
// two places at once makes the kernel split the byte stream between them, which
// shows up as "Resource temporarily unavailable" + garbage framing.
//
// Policy: LAST ACQUIRE WINS. acquire(port, owner) hands the port to `owner` and,
// if a *different* owner currently holds it, revokes it from that owner via
// revoked() — the previous owner is expected to close/disconnect on that signal.
//
// Port names are normalised so "/dev/ttyUSB0" and "ttyUSB0" map to one key
// (SerialManager uses the short QSerialPort name; RcBridge uses the full path).
class PortArbiter : public QObject {
  Q_OBJECT
 public:
  static PortArbiter& instance() {
    static PortArbiter inst;
    return inst;
  }

  static QString normalize(const QString& port) {
    return port.trimmed().section('/', -1);
  }

  // Claim `port` for `owner`. If a different owner holds it, that owner is
  // revoked first. No-op when `owner` already holds it.
  void acquire(const QString& port, QObject* owner) {
    const QString key = normalize(port);
    if (key.isEmpty() || owner == nullptr) return;
    QObject* prev = m_owners.value(key, nullptr);
    if (prev == owner) return;
    m_owners.insert(key, owner);
    if (prev != nullptr) emit revoked(key, prev);
  }

  // Release `port` only if `owner` currently holds it.
  void release(const QString& port, QObject* owner) {
    const QString key = normalize(port);
    if (m_owners.value(key, nullptr) == owner) m_owners.remove(key);
  }

 signals:
  // `port` is the normalised key; `owner` is the QObject that must give it up.
  void revoked(const QString& port, QObject* owner);

 private:
  PortArbiter() = default;
  QHash<QString, QObject*> m_owners;
};
