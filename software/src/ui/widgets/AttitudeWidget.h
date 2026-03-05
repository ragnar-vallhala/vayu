#pragma once

#include "Types.h"
#include <QOpenGLFunctions>
#include <QOpenGLWidget>

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

private:
  void drawHorizon(QPainter &p, int w, int h);
  void drawBankIndicator(QPainter &p, int cx, int cy, int r);
  void drawCrosshair(QPainter &p, int cx, int cy);
  void drawCompass(QPainter &p, int w, int h);

  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
};
