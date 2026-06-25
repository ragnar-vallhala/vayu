#pragma once

#include "RealTimeGraph.h"
#include "RollingStats.h"
#include <QLabel>
#include <QVector>
#include <QWidget>

class DroneViewWidget;

class MotorStatusWidget : public QWidget {
  Q_OBJECT

public:
  explicit MotorStatusWidget(QWidget *parent = nullptr);
  ~MotorStatusWidget() override = default;

  void setMotorSpeeds(const QVector<float> &speeds);

signals:
  void backToHomeRequested();

private:
  DroneViewWidget *m_droneView;
  RealTimeGraph *m_graph;
  QLabel *m_valLabels[4];
  QLabel *m_stdLabels[4];
  // Power group (mockup .gframe "Power"). Avg-throttle / max-spread are derived
  // from the live motor speeds; battery / current / ESC-temp / mAh are
  // placeholders until power telemetry exists (keep-the-widget policy).
  QLabel *m_avgThrottle = nullptr;
  QLabel *m_maxSpread = nullptr;
  QLabel *m_battery = nullptr;
  QLabel *m_current = nullptr;
  QLabel *m_escTemp = nullptr;
  QLabel *m_mahUsed = nullptr;
  RollingStats m_stats[4];
  QVector<float> m_speeds;
};
