#pragma once

#include <QLabel>
#include <QStatusBar>

// Bottom-of-window status bar for Navigator. Owns three permanent
// labels: connection pill, sync-drift readout, packet counter.
//
// Also exposes two helpers that operate on the LIVE label living in
// MainToolbar — `flashLive` is called on heartbeat rx, `fadeLive` is
// called from the UI tick to animate the after-glow. The LIVE label
// pointer is passed in so we don't take a hard dependency on the
// toolbar header here.
class MainStatusBar : public QStatusBar {
  Q_OBJECT

public:
  explicit MainStatusBar(QWidget *parent = nullptr);

  void setConnectionStatus(bool connected, const QString &portLabel);
  void setError(const QString &message);
  void setPacketCount(int n);
  // Inbound packet rate (Hz), sampled ~1 Hz by MainWindow. Mockup "Rate: N Hz".
  void setPacketRate(double hz);
  // Updates the "Diff: ±N ms" pill colour-banded by absolute drift:
  // green < 20 ms, amber < 100 ms, red otherwise.
  void showSyncDrift(qint32 driftMs);

  // LIVE blinker helpers. `live` is the QLabel from MainToolbar.
  // flashLive sets the bright accent style. fadeLive applies an
  // exponential decay back to the idle stylesheet from dark.qss.
  static void flashLive(QLabel *live);
  static void fadeLive(QLabel *live, qint64 elapsedSinceFlashMs);

private:
  QLabel *m_connStatus = nullptr;
  QLabel *m_syncStatus = nullptr;
  QLabel *m_pktStatus  = nullptr;
  QLabel *m_rateStatus = nullptr;
  QLabel *m_infoLabel  = nullptr;  // right-aligned "Navigator · NavLink v1 · …"
};
