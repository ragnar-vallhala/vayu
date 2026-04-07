#pragma once

#include "RealTimeGraph.h"
#include "protocol/DroneProtocol.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

class ControlLoopPlot : public QWidget {
  Q_OBJECT

public:
  explicit ControlLoopPlot(QWidget *parent = nullptr);
  void setProtocol(DroneProtocol *protocol);

signals:
  void backToHomeRequested();

private slots:
  void onControlLoopDataReceived(const ControlLoopData &data);
  void onImuReceived(const ImuData &data);

private:
  DroneProtocol *m_protocol = nullptr;

  RealTimeGraph *m_angleGraph;
  RealTimeGraph *m_rateGraph;
  RealTimeGraph *m_outputGraph;
  RealTimeGraph *m_dtGraph;

  // Angle labels
  QLabel *m_rollAngleErrVal;
  QLabel *m_pitchAngleErrVal;
  QLabel *m_yawAngleErrVal;
  QLabel *m_rollAngleSpVal;
  QLabel *m_pitchAngleSpVal;
  QLabel *m_yawAngleSpVal;

  // Rate labels
  QLabel *m_rollRateErrVal;
  QLabel *m_pitchRateErrVal;
  QLabel *m_yawRateErrVal;
  QLabel *m_rollRateSpVal;
  QLabel *m_pitchRateSpVal;
  QLabel *m_yawRateSpVal;

  // Output labels
  QLabel *m_rollOutVal;
  QLabel *m_pitchOutVal;
  QLabel *m_yawOutVal;
  QLabel *m_throttleOutVal;
  QLabel *m_dtVal;

  float m_gyroScale = 1.0f;
};
