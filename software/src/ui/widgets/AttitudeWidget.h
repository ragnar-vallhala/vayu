#pragma once

#include "Types.h"
#include <QOpenGLFunctions>
#include <QOpenGLWidget>

class QTimer;

/**
 * An artificial horizon widget drawn with QPainter.
 *
 * Displays a sky/ground split rotated by roll with a pitch offset,
 * compass heading (yaw) in a bottom strip, and wing level markers.
 */
class AttitudeWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  explicit AttitudeWidget(QWidget *parent = nullptr);

  QSize minimumSizeHint() const override { return {260, 260}; }
  QSize sizeHint() const override { return {320, 320}; }

public slots:
  void setAttitude(const AttitudeData &att);
  void setAttitude(float roll, float pitch, float yaw);

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  // Banked-horizon ADI (mockup style): the sky/ground horizon tilts with roll
  // and shifts with pitch inside `adi`; a roll-arc + bank pointer sit on top, a
  // scrolling heading tape runs along the bottom strip, fixed wings in the
  // centre. `adi` is the viewport rect above the tape.
  void drawHorizon(QPainter &p, const QRectF &adi);
  void drawPitchLadder(QPainter &p, const QRectF &adi);
  void drawRollArc(QPainter &p, const QRectF &adi);
  void drawAircraftRef(QPainter &p, const QRectF &adi);
  void drawHeadingTape(QPainter &p, const QRectF &tape);

  // Displayed angles are eased toward the latest target each frame (m_smoothTimer)
  // so bursty / low-rate telemetry renders as smooth motion instead of snapping.
  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
  float m_targetRoll = 0.0f;
  float m_targetPitch = 0.0f;
  float m_targetYaw = 0.0f;
  bool m_haveTarget = false;     // snap to the first sample, ease after that
  QTimer *m_smoothTimer = nullptr;
};
