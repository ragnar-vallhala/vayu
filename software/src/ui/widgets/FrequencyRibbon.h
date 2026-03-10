#pragma once

#include <QElapsedTimer>
#include <QLabel>
#include <QMap>
#include <QTimer>
#include <QWidget>

class DroneProtocol;

/**
 * A horizontal ribbon that displays the frequency (Hz) of various received
 * packets. Includes smoothing (EMA) and a premium "pill" aesthetic.
 */
class FrequencyRibbon : public QWidget {
  Q_OBJECT

public:
  explicit FrequencyRibbon(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

public slots:
  void packetReceived(const QString &pktType);

private slots:
  void onUpdateTimer();

private:
  struct PacketStats {
    uint32_t count = 0;
    float currentHz = 0.0f;
    float smoothedHz = 0.0f;
    QLabel *label = nullptr;
    QString color;
    QString displayName;
  };

  void addPacketType(const QString &key, const QString &displayName,
                     const QString &color);
  void updateLabels();
  QString getPillStyle(const QString &color);

  QMap<QString, PacketStats> m_stats;
  QTimer *m_timer = nullptr;
  QElapsedTimer m_elapsed;
  qint64 m_lastUpdateTime = 0;

  // Smoothing factor (0.0 to 1.0). Higher = more responsive, lower = smoother.
  const float m_alpha = 0.2f;
};
