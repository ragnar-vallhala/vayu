#pragma once

#include <QWidget>

// Compact green-HUD artificial horizon: sky/ground fill rotated by roll with a
// pitch ladder, roll arc + pointer, boresight and a heading readout. Styled
// like the in-sim SimHudWidget horizon, sized for a corner overlay. Driven by
// roll/pitch/yaw in degrees (NED aerospace convention).
class HorizonHud : public QWidget {
  Q_OBJECT
public:
  explicit HorizonHud(QWidget *parent = nullptr);

public slots:
  void setAttitude(float rollDeg, float pitchDeg, float yawDeg);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
};
