#include "TelemetryEngine.h"

#include "SerialManager.h"
#include "UdpManager.h"

#include <QDateTime>
#include <QMetaType>
#include <QStringList>

TelemetryEngine::TelemetryEngine(QObject *parent)
    : QObject(parent),
      m_protocol(new DroneProtocol(this)),
      m_serial(new SerialManager(this)),
      m_udp(new UdpManager(this)) {

  // The parser now lives on the worker thread, so DroneProtocol's struct-carrying
  // signals reach the GUI widgets (control-loop / perf / calibration) via QUEUED
  // connections — register the payload metatypes so the copies marshal correctly.
  qRegisterMetaType<ImuData>("ImuData");
  qRegisterMetaType<AttitudeData>("AttitudeData");
  qRegisterMetaType<RcData>("RcData");
  qRegisterMetaType<MotorData>("MotorData");
  qRegisterMetaType<ControlLoopData>("ControlLoopData");
  qRegisterMetaType<EstPerfData>("EstPerfData");
  qRegisterMetaType<CalibrationUpdate>("CalibrationUpdate");
  qRegisterMetaType<PerfReport>("PerfReport");

  m_elapsed.start();

  // --- Live transports → parser. Both transports are children of the engine,
  // so after moveToThread() these are same-thread (direct) connections and the
  // socket is drained on the worker thread. onLiveBytes() tees to the recorder
  // and (unless replaying) feeds the parser. ---
  connect(m_serial, &SerialManager::dataReceived, this,
          &TelemetryEngine::onLiveBytes);
  connect(m_udp, &UdpManager::dataReceived, this,
          &TelemetryEngine::onLiveBytes);

  // --- Forward transport status/error/tx to the GUI (auto-queued). ---
  connect(m_serial, &SerialManager::connectionStateChanged, this,
          [this](bool c) { emit connectionStateChanged(c, /*isUdp=*/false); });
  connect(m_udp, &UdpManager::connectionStateChanged, this,
          [this](bool c) { emit connectionStateChanged(c, /*isUdp=*/true); });
  connect(m_serial, &SerialManager::errorOccurred, this,
          [this](const QString &m) { emit errorOccurred(m, /*isUdp=*/false); });
  connect(m_udp, &UdpManager::errorOccurred, this,
          [this](const QString &m) { emit errorOccurred(m, /*isUdp=*/true); });
  connect(m_serial, &SerialManager::dataSent, this,
          [this](const QByteArray &p) { emit txPacket(p); });
  connect(m_udp, &UdpManager::dataSent, this,
          [this](const QByteArray &p) { emit txPacket(p); });

  // --- Wire-packet counter: one tick per valid + unknown packet. This is the
  // true inbound packet rate (DroneProtocol emits packetReceived for every
  // valid packet, unknownPacket for malformed). ---
  auto bumpCount = [this] {
    QMutexLocker lock(&m_mutex);
    ++m_state.packetCount;
  };
  connect(m_protocol, &DroneProtocol::packetReceived, this, bumpCount);
  connect(m_protocol, &DroneProtocol::unknownPacket, this, bumpCount);

  // Live packet-type-filtered export: write each kept wire packet verbatim to a
  // replayable .bin. Runs on the worker thread (same as the recorder).
  connect(m_protocol, &DroneProtocol::packetReceived, this,
          [this](const QByteArray &pkt) {
            if (!m_exporting || pkt.size() < 2) return;
            const int type = (static_cast<quint8>(pkt[1]) >> 4) & 0x0F;
            if (m_exportMask & static_cast<quint16>(1u << type))
              m_exporter.writeFrame(quint64(m_elapsed.nsecsElapsed() / 1000), pkt);
          });

  // --- High-rate state signals: consumed here to update the store. These are
  // NEVER re-wired to the UI; the UI pulls snapshot() at the render rate. ---
  connect(m_protocol, &DroneProtocol::imuReceived, this,
          [this](const ImuData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.imu = d;
            m_state.lastImuMs = QDateTime::currentMSecsSinceEpoch();
          });

  connect(m_protocol, &DroneProtocol::attitudeReceived, this,
          [this](const AttitudeData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.attitude = d;
            m_state.lastAttMs = QDateTime::currentMSecsSinceEpoch();
            // Per-packet std-dev accumulation (must not be down-sampled to the
            // render rate); computed values are published in the snapshot.
            m_attStats[0].push(d.roll);
            m_attStats[1].push(d.pitch);
            m_attStats[2].push(d.yaw);
            m_state.rollStd = m_attStats[0].stdDev();
            m_state.pitchStd = m_attStats[1].stdDev();
            m_state.yawStd = m_attStats[2].stdDev();
          });

  connect(m_protocol, &DroneProtocol::rcReceived, this,
          [this](const RcData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.rc = d;
            m_state.lastRcMs = QDateTime::currentMSecsSinceEpoch();
          });

  connect(m_protocol, &DroneProtocol::motorReceived, this,
          [this](const MotorData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.motors = d;
            m_state.lastMotorMs = QDateTime::currentMSecsSinceEpoch();
          });

  connect(m_protocol, &DroneProtocol::baroReceived, this,
          [this](const BaroData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.baro = d;
            m_state.lastBaroMs = QDateTime::currentMSecsSinceEpoch();
          });

  connect(m_protocol, &DroneProtocol::verticalStateReceived, this,
          [this](const VerticalStateData &d) {
            QMutexLocker lock(&m_mutex);
            m_state.vertical = d;
            m_state.lastVerticalMs = QDateTime::currentMSecsSinceEpoch();
          });

  connect(m_protocol, &DroneProtocol::flightModeReceived, this,
          [this](quint8 mode, quint8 source) {
            QMutexLocker lock(&m_mutex);
            m_state.flightMode = mode;
            m_state.flightModeSource = source;
          });

  connect(m_protocol, &DroneProtocol::statusReceived, this,
          [this](const QString &msg) {
            // PACKET_TYPE_SYSTEM_STATUS is multiplexed across origins; only the
            // SYS_STATE origin decodes to a real state name. Ignore anything
            // that isn't a recognised state (the binary HEALTH counters get
            // stringified to non-printable bytes upstream).
            static const QStringList kStateNames = {
                "UNINITIALIZED", "INIT",     "STANDBY",    "PREARM",
                "ARMED",         "IN_AIR",   "FAILSAFE",   "TERMINATED",
                "CALIBRATING"};
            if (!kStateNames.contains(msg)) return;
            QMutexLocker lock(&m_mutex);
            m_state.vehicleState = msg;
            // ARMED / IN_AIR / FAILSAFE all count as armed so the operator can
            // always recover (DISARM stays available).
            m_state.armed =
                (msg == "ARMED" || msg == "IN_AIR" || msg == "FAILSAFE");
          });
}

void TelemetryEngine::onLiveBytes(const QByteArray &b) {
  if (m_recorder.isOpen())
    m_recorder.writeFrame(quint64(m_elapsed.nsecsElapsed() / 1000), b);
  if (m_acceptLive)
    feedBytes(b);
}

void TelemetryEngine::feedBytes(const QByteArray &data) {
  m_protocol->processData(data);
}

VehicleState TelemetryEngine::snapshot() const {
  QMutexLocker lock(&m_mutex);
  return m_state;
}

// ---- Command API (runs on the worker thread) -------------------------------

void TelemetryEngine::openSerial(const QString &port, int baud) {
  const bool ok = m_serial->open(port, baud);
  emit linkOpened(ok, port, /*isUdp=*/false);
}

void TelemetryEngine::bindUdp(int port) {
  const bool ok = m_udp->bind(static_cast<quint16>(port));
  emit linkOpened(ok, QStringLiteral("udp:%1").arg(port), /*isUdp=*/true);
}

void TelemetryEngine::closeLinks() {
  m_udp->close();
  m_serial->close();
}

void TelemetryEngine::send(const QByteArray &pkt) {
  // Route to the active transport: UDP (ESP bridge) wins, else the wired serial.
  if (m_udp->isOpen())
    m_udp->write(pkt);
  else if (m_serial->isOpen())
    m_serial->write(pkt);
}

void TelemetryEngine::setAutoReconnect(bool on) {
  m_serial->setAutoReconnect(on);
}

void TelemetryEngine::setReconnectInterval(int ms) {
  m_serial->setReconnectIntervalMs(ms);
}

void TelemetryEngine::setReplayMode(bool on) { setLiveFeed(!on); }

void TelemetryEngine::setLiveFeed(bool on) { m_acceptLive = on; }

void TelemetryEngine::startRecording(const QString &path,
                                     qulonglong startWallClockMs) {
  if (m_recorder.isOpen()) return;
  m_recorder.open(path, /*protocolVersion=*/1, startWallClockMs);
}

void TelemetryEngine::stopRecording() {
  if (m_recorder.isOpen()) m_recorder.close();
}

void TelemetryEngine::startExport(const QString &path, int typeMask) {
  if (m_exporter.isOpen()) m_exporter.close();
  m_exportMask = static_cast<quint16>(typeMask);
  m_exporting = m_exporter.open(
      path, /*protocolVersion=*/1,
      quint64(QDateTime::currentMSecsSinceEpoch()));
  emit exportStateChanged(m_exporting, m_exporting ? path : QString());
}

void TelemetryEngine::stopExport() {
  if (m_exporter.isOpen()) m_exporter.close();
  m_exporting = false;
  emit exportStateChanged(false, QString());
}
