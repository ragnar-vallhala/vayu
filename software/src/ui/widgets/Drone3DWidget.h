#pragma once

#include "../core/Types.h"
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QVector3D>

class Drone3DWidget : public QOpenGLWidget {
  Q_OBJECT

public:
  explicit Drone3DWidget(QWidget *parent = nullptr);
  void setAttitude(const AttitudeData &att);
  void setAttitude(float roll, float pitch, float yaw);

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private:
  struct Face {
    QVector3D points[4];
    QColor color;
  };

  void drawCube(QPainter &p, const QMatrix4x4 &mvp, float w, float h, float d,
                const QColor &color);
  QPointF project(const QVector3D &v, const QMatrix4x4 &mvp);

  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;
};
