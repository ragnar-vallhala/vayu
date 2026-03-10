#include "FrequencyRibbon.h"
#include "DroneProtocol.h"
#include <QHBoxLayout>
#include <cmath>

FrequencyRibbon::FrequencyRibbon(QWidget *parent) : QWidget(parent) {
  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(10, 4, 10, 4);
  layout->setSpacing(12);

  setStyleSheet("FrequencyRibbon { background-color: #1a1d27; border-bottom: "
                "1px solid #2a3347; }");

  // Add packet types with distinct colors
  addPacketType("IMU", "IMU", "#4ECDC4");      // Teal
  addPacketType("ATT", "Attitude", "#FF6B6B"); // Coral
  addPacketType("HB", "Heartbeat", "#FFE66D"); // Yellow
  addPacketType("RC", "RC", "#98C379");        // Green
  addPacketType("LOG", "LOG", "#ABB2BF");      // Gray
  addPacketType("RAW", "RAW/UNK", "#C678DD");  // Purple

  layout->addStretch();

  // "TOTAL" badge at the end
  addPacketType("TOTAL", "TOTAL RX", "#61AFEF"); // Blue

  m_timer = new QTimer(this);
  connect(m_timer, &QTimer::timeout, this, &FrequencyRibbon::onUpdateTimer);
  m_timer->start(200); // 5 Hz update rate for high responsiveness

  m_elapsed.start();
  m_lastUpdateTime = m_elapsed.elapsed();
}

void FrequencyRibbon::setProtocol(DroneProtocol *protocol) {
  if (!protocol)
    return;

  connect(protocol, &DroneProtocol::imuReceived, this, [this]() {
    m_stats["IMU"].count++;
    m_stats["TOTAL"].count++;
  });
  connect(protocol, &DroneProtocol::attitudeReceived, this, [this]() {
    m_stats["ATT"].count++;
    m_stats["TOTAL"].count++;
  });
  connect(protocol, &DroneProtocol::heartbeatReceived, this, [this]() {
    m_stats["HB"].count++;
    m_stats["TOTAL"].count++;
  });
  connect(protocol, &DroneProtocol::rcReceived, this, [this]() {
    m_stats["RC"].count++;
    m_stats["TOTAL"].count++;
  });
  connect(protocol, &DroneProtocol::logReceived, this, [this]() {
    m_stats["LOG"].count++;
    m_stats["TOTAL"].count++;
  });
  connect(protocol, &DroneProtocol::unknownPacket, this, [this]() {
    m_stats["RAW"].count++;
    m_stats["TOTAL"].count++;
  });
}

void FrequencyRibbon::addPacketType(const QString &key,
                                    const QString &displayName,
                                    const QString &color) {
  auto *layout = qobject_cast<QHBoxLayout *>(this->layout());
  if (!layout)
    return;

  auto *label = new QLabel(this);
  label->setAlignment(Qt::AlignCenter);
  label->setContentsMargins(8, 2, 8, 2);
  label->setStyleSheet(getPillStyle(color));
  layout->addWidget(label);

  PacketStats stats;
  stats.label = label;
  stats.color = color;
  stats.displayName = displayName;
  m_stats[key] = stats;

  label->setText(QString("%1: 0.0 Hz").arg(displayName));
}

void FrequencyRibbon::packetReceived(const QString &pktType) {
  if (m_stats.contains(pktType)) {
    m_stats[pktType].count++;
    m_stats["TOTAL"].count++;
  }
}

void FrequencyRibbon::onUpdateTimer() {
  qint64 now = m_elapsed.elapsed();
  double dt = (now - m_lastUpdateTime) / 1000.0;
  if (dt < 0.05)
    return; // Minimum window to avoid instability

  for (auto it = m_stats.begin(); it != m_stats.end(); ++it) {
    float rawHz = static_cast<float>(it.value().count / dt);

    // EMA Smoothing: Smoothed = (1 - alpha) * Smoothed + alpha * Raw
    // If it was zero for a while and suddenly jumps, alpha helps it ramp up
    // smoothly.
    it.value().smoothedHz =
        (1.0f - m_alpha) * it.value().smoothedHz + m_alpha * rawHz;

    // Threshold to force absolute zero if very low
    if (it.value().smoothedHz < 0.05f)
      it.value().smoothedHz = 0.0f;

    it.value().count = 0;
  }

  updateLabels();
  m_lastUpdateTime = now;
}

void FrequencyRibbon::updateLabels() {
  for (auto it = m_stats.begin(); it != m_stats.end(); ++it) {
    float val = it.value().smoothedHz;
    it.value().label->setText(
        QString("%1: %2 Hz").arg(it.value().displayName).arg(val, 0, 'f', 1));

    // Opt: Dim the pill if frequency is zero
    if (val <= 0.0f) {
      // This isn't standard QLabel, so use stylesheet instead or just ignore
      it.value().label->setStyleSheet(getPillStyle(it.value().color) +
                                      " color: #4B5263; ");
    } else {
      it.value().label->setStyleSheet(getPillStyle(it.value().color));
    }
  }
}

QString FrequencyRibbon::getPillStyle(const QString &color) {
  return QString("QLabel { "
                 "  background-color: rgba(40, 44, 52, 180); "
                 "  color: %1; "
                 "  border: 1px solid %1; "
                 "  border-radius: 10px; "
                 "  font-family: 'Monospace'; "
                 "  font-size: 10px; "
                 "  font-weight: bold; "
                 "  padding: 1px 10px; "
                 "}")
      .arg(color);
}
