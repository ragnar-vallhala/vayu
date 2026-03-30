#pragma once

#include "RealTimeGraph.h"
#include "protocol/DroneProtocol.h"
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

class PidErrorPlot : public QWidget {
  Q_OBJECT

public:
  explicit PidErrorPlot(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

signals:
  void backToHomeRequested();

private slots:
  void onPidErrorReceived(const PidErrorData &data);

private:
  DroneProtocol *m_protocol = nullptr;
  RealTimeGraph *m_graph;
  QLabel *m_rollVal;
  QLabel *m_pitchVal;
  QLabel *m_yawVal;
};
