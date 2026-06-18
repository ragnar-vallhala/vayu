#pragma once

#include "RealTimeGraph.h"
#include "RollingStats.h"
#include "Types.h"
#include "VGauge.h"
#include <QColor>
#include <QLabel>
#include <QWidget>

/**
 * Dashboard IMU telemetry panel (mockup parity): a header (icon + title),
 * a 2×2 grid of self-contained graphs — Accelerometer, Gyroscope, Magnetometer
 * and Baro Altitude — beside a right-hand column with the device-temp + battery
 * vertical gauges, with a vehicle-state legend under the grid. Each graph plots
 * its axes plus rolling-σ traces and the vehicle-state colour band.
 */
class ImuPanel : public QWidget {
  Q_OBJECT

public:
  explicit ImuPanel(QWidget *parent = nullptr);

public slots:
  // `available` = live IMU telemetry is fresh. When false the temp gauge reads
  // "-" and the graphs go NA on their own (no new samples).
  void updateImu(const ImuData &data, bool available = true);
  // Baro altitude (m) for the "Baro Altitude" graph: MSL (sea-level height) and
  // AGL (height above the ground reference). `available` = fresh BARO telemetry;
  // when false the graph stops getting samples and goes NA.
  void setBaroAltitude(float mslM, float aglM, bool available = true);
  void setSensor(const QString &name);  // updates the header (e.g. "IMU — BMX160")
  void setGraphWindow(int seconds);
  void setGraphDropout(double rate);
  // Settings ▸ Plots & Graphs appearance toggles, fanned out to every graph.
  void setSigmaTraces(bool on);   // rolling-σ overlay (acc/gyr/mag)
  void setStateBand(bool on);     // vehicle-state colour band
  void setTraceWidth(double w);   // trace pen width
  void setAntialias(bool on);     // antialiased trace painting
  // Battery level (0..100 %); no telemetry source yet so MainWindow leaves it.
  void setBattery(double pct);
  // Vehicle-state colour for the graphs' state band (advanced per IMU update).
  void setVehicleState(const QColor &c) { m_state = c; }

private:
  QLabel *m_header = nullptr;
  RealTimeGraph *m_accG = nullptr;
  RealTimeGraph *m_gyrG = nullptr;
  RealTimeGraph *m_magG = nullptr;
  RealTimeGraph *m_baroG = nullptr;  // fed by BARO telemetry (setBaroAltitude)
  RollingStats m_accStats[3];
  RollingStats m_gyrStats[3];
  RollingStats m_magStats[3];
  VGauge *m_tempGauge = nullptr;
  VGauge *m_battGauge = nullptr;
  QColor m_state{0x98, 0xC3, 0x79};  // standby green by default
};
