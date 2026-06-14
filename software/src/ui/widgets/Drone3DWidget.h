#pragma once

#include "../core/Types.h"
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QVector3D>

class QTimer;

// Stylised quad-copter attitude view, mirroring the UI mockup's #adi3d: an
// X-frame seen from a tilted top-down angle — two crossed arms, four motor pods
// (front red, rear green), an accent "nose" heading marker and spinning props,
// over a soft ground shadow. Banks/pitches/yaws with the live attitude.
class Drone3DWidget : public QOpenGLWidget {
  Q_OBJECT

public:
  explicit Drone3DWidget(QWidget *parent = nullptr);
  void setAttitude(const AttitudeData &att);
  void setAttitude(float roll, float pitch, float yaw);

protected:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
  float m_propPhase = 0.0f;  // degrees, advanced by the spin timer
  QTimer *m_spinTimer = nullptr;
};
