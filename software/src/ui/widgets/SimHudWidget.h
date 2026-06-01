#pragma once

#include "../../vsim/SimWorker.h"

#include <QString>
#include <QVector>
#include <QWidget>
#include <array>

// SimHudWidget — an FPV-style flight HUD painted over the simulator
// viewport: artificial horizon + pitch ladder, roll arc, heading/compass
// tape, altitude + speed tapes, vertical speed, and per-motor throttle.
// Transparent + mouse-transparent so the 3D view shows through and camera
// control still works. Fed from the sim pose at 60 Hz.
class SimHudWidget : public QWidget {
  Q_OBJECT
 public:
  explicit SimHudWidget(QWidget* parent = nullptr);

  void setSnapshot(const vsim::SimSnapshot& s);

  // Telemetry-fed (not in the sim pose): the firmware flight-state name and
  // the latest IMU sample. setImu pushes into rolling history for the plots.
  void setStatus(const QString& s);
  void setImu(const float acc[3], const float gyr[3]);

 protected:
  void paintEvent(QPaintEvent* e) override;

 private:
  static constexpr int kHistN = 128;       // mini-plot history depth

  float roll_ = 0, pitch_ = 0, yaw_ = 0;   // deg
  float alt_ = 0, gs_ = 0, vs_ = 0;        // m, m/s, m/s
  std::array<float, 4> motor_{0, 0, 0, 0}; // duty [0,1]

  QString status_;                         // flight-state name
  std::array<QVector<float>, 3> accHist_;  // m/s² X Y Z
  std::array<QVector<float>, 3> gyrHist_;  // deg/s X Y Z
};
