#pragma once

#include <QElapsedTimer>
#include <QLabel>
#include <QWidget>
#include <cstdint>

class DroneProtocol;
class RealTimeGraph;
class QTimer;

/**
 * Nettop-style link panel for the packet analyzer: link quality on the left,
 * up/down throughput on the right, plus cumulative byte totals in the header.
 *
 *   link quality (%) = good_bytes / (good_bytes + raw_bytes) * 100   (per second)
 *   down speed       = RX bytes / second   (good + raw)
 *   up speed         = TX bytes / second   (commands sent to the FC)
 *
 * RX bytes come from the decoder — every consumed byte is emitted via exactly
 * one of DroneProtocol::packetReceived (a whole valid packet) or
 * unknownPacket (discarded junk), so the two tallies partition the stream.
 * TX bytes are fed in via addTxBytes() from the analyzer's TX log (serial + UDP).
 */
class LinkStatsPanel : public QWidget {
  Q_OBJECT

public:
  explicit LinkStatsPanel(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);
  void addTxBytes(int n); // called once per logged TX packet

private slots:
  void onTick(); // ~1 Hz: fold the window's byte tallies into samples

private:
  RealTimeGraph *m_qualityGraph = nullptr; // 1 series: quality %
  RealTimeGraph *m_speedGraph = nullptr;   // series 0 = down, 1 = up (KB/s)
  QLabel *m_qualityLabel = nullptr;
  QLabel *m_downLabel = nullptr;
  QLabel *m_upLabel = nullptr;
  QLabel *m_totalLabel = nullptr;

  QTimer *m_timer = nullptr;
  QElapsedTimer m_clock; // measures the true window length for rate maths

  // In-progress 1-second window.
  uint64_t m_goodBytes = 0;
  uint64_t m_rawBytes = 0;
  uint64_t m_txBytes = 0;

  // Cumulative since start / Clear.
  uint64_t m_totalRx = 0;
  uint64_t m_totalTx = 0;
};
