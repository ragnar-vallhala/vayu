#include "MainStatusBar.h"

#include "core/Theme.h"

#include <cmath>

MainStatusBar::MainStatusBar(QWidget *parent) : QStatusBar(parent) {
  m_connStatus = new QLabel("  ● Disconnected  ", this);
  m_connStatus->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kDanger)));

  m_syncStatus = new QLabel("  Diff: ---  ", this);
  m_syncStatus->setStyleSheet(
      QString("color: %1; font-family: Monospace;")
          .arg(Theme::hex(Theme::kTextMuted)));

  m_pktStatus = new QLabel("  Packets: 0  ", this);

  m_rateStatus = new QLabel("  Rate: 0 Hz  ", this);
  m_rateStatus->setStyleSheet(
      QString("color: %1; font-family: Monospace;")
          .arg(Theme::hex(Theme::kTextMuted)));

  // Mockup parity: segments are left-aligned; a build/protocol/nav hint sits
  // on the far right. addWidget() docks left, addPermanentWidget() docks right.
  addWidget(m_connStatus);
  addWidget(m_syncStatus);
  addWidget(m_pktStatus);
  addWidget(m_rateStatus);

  m_infoLabel = new QLabel("Navigator · NavLink v1 · Ctrl+1‑8 navigate  ", this);
  m_infoLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextDim)));
  addPermanentWidget(m_infoLabel);
}

// ---------------------------------------------------------------------------

void MainStatusBar::setConnectionStatus(bool connected, const QString &portLabel) {
  if (connected) {
    m_connStatus->setText(QString("  ● Connected: %1  ").arg(portLabel));
    m_connStatus->setStyleSheet(
        QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kOk)));
  } else {
    m_connStatus->setText("  ● Disconnected  ");
    m_connStatus->setStyleSheet(
        QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kDanger)));
  }
}

void MainStatusBar::setReplayStatus() {
  m_connStatus->setText("  ● Replay  ");
  m_connStatus->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kAccent)));
}

void MainStatusBar::setError(const QString &message) {
  m_connStatus->setText(QString("  ● Error: %1  ").arg(message));
  m_connStatus->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(Theme::hex(Theme::kDanger)));
}

void MainStatusBar::setPacketCount(int n) {
  m_pktStatus->setText(QString("  Packets: %1  ").arg(n));
}

void MainStatusBar::setPacketRate(double hz) {
  m_rateStatus->setText(QString("  Rate: %1 Hz  ").arg(hz, 0, 'f', 0));
}

void MainStatusBar::showSyncDrift(qint32 driftMs) {
  const QString text =
      QString("  Diff: %1%2ms  ").arg(driftMs >= 0 ? "+" : "").arg(driftMs);
  m_syncStatus->setText(text);

  const qint32 absDiff = std::abs(driftMs);
  QColor c;
  if      (absDiff < 20)  c = Theme::kOk;
  else if (absDiff < 100) c = Theme::kWarn;
  else                    c = Theme::kDanger;
  m_syncStatus->setStyleSheet(
      QString("color: %1; font-weight: bold; font-family: Monospace;")
          .arg(Theme::hex(c)));
}

// ---------------------------------------------------------------------------
// LIVE blinker (label lives in MainToolbar; we just style it).
// ---------------------------------------------------------------------------

void MainStatusBar::flashLive(QLabel *live) {
  if (!live) return;
  live->setStyleSheet(
      "background: #2D6A2D; color: #FFFFFF; border-radius: 4px; "
      "font-weight: bold; padding: 2px 8px; "
      "border: 1px solid #98C379;");
}

void MainStatusBar::fadeLive(QLabel *live, qint64 elapsed) {
  if (!live) return;

  if (elapsed < 150) {
    // Hold bright; nothing to do.
    return;
  }
  if (elapsed < 2000) {
    // Exponential decay back to the idle look. Same curve the old
    // inline updater used; pulled here so the math has one home.
    const double factor = std::exp(-4.0 * (elapsed - 150) / 1850.0);
    const int alpha = static_cast<int>(255 * factor);
    const int green = static_cast<int>(106 * factor + 29);  // 29 minimum
    live->setStyleSheet(
        QString("background: rgba(30, %1, 30, 200); "
                "color: rgba(255, 255, 255, %2); "
                "border-radius: 4px; font-weight: bold; padding: 2px 8px; "
                "border: 1px solid rgba(152, 195, 121, %2);")
            .arg(green)
            .arg(alpha));
    return;
  }
  // Idle: drop back to the dark.qss QLabel#LiveBlinker default.
  live->setStyleSheet(QString());
}
