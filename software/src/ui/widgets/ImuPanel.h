#pragma once

#include "RealTimeGraph.h"
#include "RollingStats.h"
#include "Types.h"
#include "VGauge.h"
#include <QColor>
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
  // Push one vehicle-state colour cell onto this group's graph band.
  void pushState(const QColor &c) { m_graph->pushState(c); }
  // Exposed so panel-level CSV export can include this group's traces.
  RealTimeGraph *graph() const { return m_graph; }

private:
  QLabel *m_labels[3];
  QLabel *m_stdLabels[3];
  RollingStats m_stats[3];
  RealTimeGraph *m_graph;
  float m_sigmaMax = 0.5f;  // right-axis σ scale, grown to fit (mockup stdMax)
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
  // Battery level (0..100 %) for the right-hand gauge. No battery telemetry
  // exists yet, so MainWindow leaves the placeholder default in place.
  void setBattery(double pct);
  // Current vehicle-state colour for the graph state band (mockup .g-status):
  // advanced one cell per IMU update so the band scrolls with the traces.
  void setVehicleState(const QColor &c) { m_state = c; }

private:
  ImuAxisGroup *m_acc;
  ImuAxisGroup *m_gyr;
  ImuAxisGroup *m_mag;
  VGauge *m_tempGauge;   // device temperature (from ImuData.tempC)
  VGauge *m_battGauge;   // battery % (placeholder until telemetry exists)
  QColor m_state{0x98, 0xC3, 0x79};  // standby green by default
};
