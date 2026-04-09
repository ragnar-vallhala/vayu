#pragma once

#include "RealTimeGraph.h"
#include "protocol/DroneProtocol.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QQueue>
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
  QLabel *m_rollAngleSpVal;
  QLabel *m_pitchAngleSpVal;
  QLabel *m_yawAngleSpVal;
  QLabel *m_rollAngleCurrVal;
  QLabel *m_pitchAngleCurrVal;
  QLabel *m_yawAngleCurrVal;

  // Rate labels
  QLabel *m_rollRateSpVal;
  QLabel *m_pitchRateSpVal;
  QLabel *m_yawRateSpVal;
  QLabel *m_rollRateCurrVal;
  QLabel *m_pitchRateCurrVal;
  QLabel *m_yawRateCurrVal;

  // Output labels
  QLabel *m_rollOutVal;
  QLabel *m_pitchOutVal;
  QLabel *m_yawOutVal;
  QLabel *m_throttleOutVal;
  QLabel *m_dtOuterVal;
  QLabel *m_dtInnerVal;
  QLabel *m_dtOuterStdVal;
  QLabel *m_dtInnerStdVal;

  QQueue<float> m_outerDtHistory;
  QQueue<float> m_innerDtHistory;
  const int m_stdWindowSize = 100;

  float m_gyroScale = 1.0f;
};
