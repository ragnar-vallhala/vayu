#pragma once

#include "../../vsim/SimWorker.h"

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

 protected:
  void paintEvent(QPaintEvent* e) override;

 private:
  float roll_ = 0, pitch_ = 0, yaw_ = 0;   // deg
  float alt_ = 0, gs_ = 0, vs_ = 0;        // m, m/s, m/s
  std::array<float, 4> motor_{0, 0, 0, 0}; // duty [0,1]
};
