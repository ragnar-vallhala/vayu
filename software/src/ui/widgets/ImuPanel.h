#pragma once

#include "RealTimeGraph.h"
#include "Types.h"
#include <QGroupBox>
#include <QLabel>

/**
 * Displays three axes of one IMU sensor channel (acc / gyr / mag)
 * in a labelled group box with large LCD-style numbers and real-time graphs.
 */
class ImuAxisGroup : public QGroupBox {
  Q_OBJECT

public:
  ImuAxisGroup(const QString &title, const QString &unit,
               QWidget *parent = nullptr);

  void setValues(float x, float y, float z);
  void setWindowSeconds(int seconds);
  void setDropoutRate(double rate);

private:
  QLabel *m_labels[3];
  RealTimeGraph *m_graph;
};

// ---------------------------------------------------------------------------

class ImuPanel : public QGroupBox {
  Q_OBJECT

public:
  explicit ImuPanel(QWidget *parent = nullptr);

public slots:
  void updateImu(const ImuData &data);
  void setSensor(const QString &name); // swap displayed sensor name
  void setGraphWindow(int seconds);
  void setGraphDropout(double rate);

private:
  ImuAxisGroup *m_acc;
  ImuAxisGroup *m_gyr;
  ImuAxisGroup *m_mag;
  ImuAxisGroup *m_temp;
};
