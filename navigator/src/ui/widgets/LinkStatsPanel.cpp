#include "LinkStatsPanel.h"

#include "DroneProtocol.h"
#include "RealTimeGraph.h"

#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

namespace {
// 1.23 MB / 4.5 KB / 980 B — binary units, Linux-style.
QString humanBytes(double b) {
  static const char *u[] = {"B", "KB", "MB", "GB", "TB"};
  int i = 0;
  while (b >= 1024.0 && i < 4) {
    b /= 1024.0;
    ++i;
  }
  return QString::number(b, 'f', i == 0 ? 0 : 2) + " " + u[i];
}
QString humanRate(double bytesPerSec) { return humanBytes(bytesPerSec) + "/s"; }

const char *kDown = "#4ECDC4"; // teal
const char *kUp = "#E5A95C";   // amber
const char *kDim = "#4B5263";

QString pill(const QString &color, int pt = 12) {
  return QString("color: %1; font-weight: bold; font-size: %2px;")
      .arg(color)
      .arg(pt);
}
QLabel *caption(QWidget *p, const QString &text) {
  auto *l = new QLabel(text, p);
  l->setStyleSheet("color: #6b7280; font-size: 9px;");
  return l;
}
} // namespace

LinkStatsPanel::LinkStatsPanel(QWidget *parent) : QWidget(parent) {
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(2, 4, 2, 0);
  outer->setSpacing(2);

  // ---- Header: live readouts -------------------------------------------
  auto *header = new QHBoxLayout();
  header->setSpacing(14);
  m_qualityLabel = new QLabel("Link —", this);
  m_qualityLabel->setStyleSheet(pill(kDim));
  m_downLabel = new QLabel("↓ —", this); // ↓
  m_downLabel->setStyleSheet(pill(kDown));
  m_upLabel = new QLabel("↑ —", this); // ↑
  m_upLabel->setStyleSheet(pill(kUp));
  m_totalLabel = new QLabel("RX 0 B  ·  TX 0 B", this);
  m_totalLabel->setStyleSheet(pill("#ABB2BF", 11));
  header->addWidget(m_qualityLabel);
  header->addWidget(m_downLabel);
  header->addWidget(m_upLabel);
  header->addStretch();
  header->addWidget(m_totalLabel);
  outer->addLayout(header);

  // ---- Two graphs, split in half ---------------------------------------
  auto *graphs = new QHBoxLayout();
  graphs->setSpacing(8);

  auto *leftCol = new QVBoxLayout();
  leftCol->setSpacing(1);
  leftCol->addWidget(caption(this, tr("Link Quality (%)")));
  m_qualityGraph = new RealTimeGraph(this, 1);
  m_qualityGraph->setWindowSeconds(60);
  m_qualityGraph->setYRange(0.0f, 100.0f);
  m_qualityGraph->setColor(0, QColor(kDown));
  m_qualityGraph->setMinimumHeight(80);
  leftCol->addWidget(m_qualityGraph, 1);

  auto *rightCol = new QVBoxLayout();
  rightCol->setSpacing(1);
  rightCol->addWidget(caption(this, tr("Throughput KB/s   ↓ down   ↑ up")));
  m_speedGraph = new RealTimeGraph(this, 2);
  m_speedGraph->setWindowSeconds(60);
  m_speedGraph->setDynamicYAxis(true); // speeds vary widely — auto-scale
  m_speedGraph->setColor(0, QColor(kDown));
  m_speedGraph->setColor(1, QColor(kUp));
  m_speedGraph->setMinimumHeight(80);
  rightCol->addWidget(m_speedGraph, 1);

  graphs->addLayout(leftCol, 1);
  graphs->addLayout(rightCol, 1);
  outer->addLayout(graphs, 1);

  setMaximumHeight(170);

  m_timer = new QTimer(this);
  connect(m_timer, &QTimer::timeout, this, &LinkStatsPanel::onTick);
  m_timer->start(1000); // metric is defined "per second"
  m_clock.start();
}

void LinkStatsPanel::setProtocol(DroneProtocol *protocol) {
  if (!protocol)
    return;
  connect(protocol, &DroneProtocol::packetReceived, this,
          [this](const QByteArray &pkt) {
            m_goodBytes += pkt.size();
            m_totalRx += pkt.size();
          });
  connect(protocol, &DroneProtocol::unknownPacket, this,
          [this](const QByteArray &raw) {
            m_rawBytes += raw.size();
            m_totalRx += raw.size();
          });
}

void LinkStatsPanel::addTxBytes(int n) {
  if (n <= 0)
    return;
  m_txBytes += static_cast<uint64_t>(n);
  m_totalTx += static_cast<uint64_t>(n);
}

void LinkStatsPanel::onTick() {
  // True elapsed window so the rates are honest even if the timer slips.
  double dt = m_clock.restart() / 1000.0;
  if (dt < 0.2)
    return;

  const uint64_t good = m_goodBytes;
  const uint64_t rxBytes = m_goodBytes + m_rawBytes;
  const uint64_t txBytes = m_txBytes;
  m_goodBytes = 0;
  m_rawBytes = 0;
  m_txBytes = 0;

  const double downBps = rxBytes / dt;
  const double upBps = txBytes / dt;

  // Throughput plot (KB/s) — always sampled so the timeline is continuous.
  m_speedGraph->appendData(static_cast<float>(downBps / 1024.0), 0);
  m_speedGraph->appendData(static_cast<float>(upBps / 1024.0), 1);

  m_downLabel->setText("↓ " + humanRate(downBps));
  m_upLabel->setText("↑ " + humanRate(upBps));
  m_totalLabel->setText("RX " + humanBytes(static_cast<double>(m_totalRx)) +
                        "  ·  TX " +
                        humanBytes(static_cast<double>(m_totalTx)));

  // Link quality only when there were RX bytes to judge.
  if (rxBytes == 0) {
    m_qualityLabel->setText("Link —");
    m_qualityLabel->setStyleSheet(pill(kDim));
    return;
  }
  const double pct = 100.0 * static_cast<double>(good) / rxBytes;
  m_qualityGraph->appendData(static_cast<float>(pct));
  const char *qc = pct >= 90.0   ? "#98C379"
                   : pct >= 70.0 ? "#FFE66D"
                                 : "#FF6B6B";
  m_qualityLabel->setText("Link " + QString::number(pct, 'f', 1) + "%");
  m_qualityLabel->setStyleSheet(pill(qc));
}
