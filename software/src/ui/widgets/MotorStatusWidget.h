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
  RollingStats m_stats[4];
  QVector<float> m_speeds;
};
