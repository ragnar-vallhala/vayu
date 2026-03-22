#include "Drone3DWidget.h"
#include <QMatrix4x4>
#include <QPainter>
#include <QPainterPath>
#include <QVector3D>
#include <QtMath>
#include <algorithm>

Drone3DWidget::Drone3DWidget(QWidget *parent) : QOpenGLWidget(parent) {
  setMinimumSize(260, 260);
}

void Drone3DWidget::setAttitude(const AttitudeData &att) {
  setAttitude(att.roll, att.pitch, att.yaw);
}

void Drone3DWidget::setAttitude(float roll, float pitch, float yaw) {
  m_roll = roll;
  m_pitch = pitch;
  m_yaw = yaw;
  update();
}

void Drone3DWidget::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  int w = width();
  int h = height();
  p.fillRect(rect(), QColor(26, 29, 39));

  // --- 3D Projection Setup ---
  QMatrix4x4 model;
  // Apply rotations (invert to match drone physics vs camera)
  // Standard PFD: Drone moves relative to horizon.
  // Here we show the DRONE rotating, so use raw angles or inverted depending on
  // perspective. We'll show the drone rotating in a fixed world.
  model.rotate(m_yaw, 0, 0, 1);   // Yaw (Z)
  model.rotate(m_pitch, 0, 1, 0); // Pitch (Y)
  model.rotate(m_roll, 1, 0, 0);  // Roll (X)

  QMatrix4x4 view;
  view.lookAt(QVector3D(10, 10, 10), QVector3D(0, 0, 0), QVector3D(0, 0, 1));

  QMatrix4x4 projection;
  projection.perspective(45.0f, (float)w / h, 0.1f, 100.0f);

  QMatrix4x4 mvp = projection * view * model;

  // --- Draw Drone ---
  // 1. Arms (X configuration)
  float armL = 4.0f;
  float armW = 0.3f;
  float armH = 0.2f;

  // Arm 1 (Front Right: +X, -Y)
  drawCube(
      p,
      mvp *
          []() {
            QMatrix4x4 m;
            m.rotate(-45, 0, 0, 1);
            m.translate(2.0, 0, 0);
            return m;
          }(),
      armL, armW, armH, QColor(62, 68, 82));
  // Arm 2 (Back Right: +X, +Y)
  drawCube(
      p,
      mvp *
          []() {
            QMatrix4x4 m;
            m.rotate(45, 0, 0, 1);
            m.translate(2.0, 0, 0);
            return m;
          }(),
      armL, armW, armH, QColor(62, 68, 82));
  // Arm 3 (Back Left: -X, +Y)
  drawCube(
      p,
      mvp *
          []() {
            QMatrix4x4 m;
            m.rotate(135, 0, 0, 1);
            m.translate(2.0, 0, 0);
            return m;
          }(),
      armL, armW, armH, QColor(62, 68, 82));
  // Arm 4 (Front Left: -X, -Y)
  drawCube(
      p,
      mvp *
          []() {
            QMatrix4x4 m;
            m.rotate(-135, 0, 0, 1);
            m.translate(2.0, 0, 0);
            return m;
          }(),
      armL, armW, armH, QColor(62, 68, 82));

  // 2. Central Body
  drawCube(p, mvp, 2.5f, 1.5f, 0.8f, QColor(33, 37, 43));

  // 3. Front Indicator (Red sphere-ish)
  QVector3D frontPos(1.2f, 0.0f, 0.0f);
  QPointF frontPt = project(frontPos, mvp);
  p.setBrush(QColor(224, 108, 117));
  p.setPen(Qt::NoPen);
  p.drawEllipse(frontPt, 6, 6);
}

void Drone3DWidget::drawCube(QPainter &p, const QMatrix4x4 &mvp, float w,
                             float h, float d, const QColor &color) {
  // Define 8 vertices
  QVector3D v[8] = {{-w / 2, -h / 2, -d / 2}, {w / 2, -h / 2, -d / 2},
                    {w / 2, h / 2, -d / 2},   {-w / 2, h / 2, -d / 2},
                    {-w / 2, -h / 2, d / 2},  {w / 2, -h / 2, d / 2},
                    {w / 2, h / 2, d / 2},    {-w / 2, h / 2, d / 2}};

  // Define 6 faces (indices)
  int faces[6][4] = {
      {0, 1, 2, 3}, {4, 5, 6, 7}, // Bottom, Top
      {0, 1, 5, 4}, {2, 3, 7, 6}, // Front, Back
      {0, 3, 7, 4}, {1, 2, 6, 5}  // Left, Right
  };

  // Sort faces by depth (very simple painter's algorithm)
  struct FaceData {
    int idx;
    float depth;
  };
  QVector<FaceData> sortedFaces;
  for (int i = 0; i < 6; ++i) {
    float avgZ = 0;
    for (int j = 0; j < 4; ++j) {
      QVector3D tv = mvp * v[faces[i][j]];
      avgZ += tv.z();
    }
    sortedFaces.append({i, avgZ / 4.0f});
  }
  std::sort(
      sortedFaces.begin(), sortedFaces.end(),
      [](const FaceData &a, const FaceData &b) { return a.depth < b.depth; });

  // Draw faces
  for (const auto &fd : sortedFaces) {
    QPolygonF poly;
    for (int i = 0; i < 4; ++i) {
      poly << project(v[faces[fd.idx][i]], mvp);
    }

    // Simple shading based on face index
    QColor c = color;
    if (fd.idx == 1)
      c = c.lighter(120); // Top
    else if (fd.idx == 0)
      c = c.darker(150); // Bottom
    else
      c = c.darker(110);

    p.setBrush(c);
    p.setPen(QPen(c.lighter(150), 1));
    p.drawPolygon(poly);
  }
}

QPointF Drone3DWidget::project(const QVector3D &v, const QMatrix4x4 &mvp) {
  QVector4D v4(v, 1.0f);
  QVector4D clip = mvp * v4;
  if (clip.w() == 0)
    return QPointF(0, 0);

  float ndcX = clip.x() / clip.w();
  float ndcY = clip.y() / clip.w();

  float screenX = (ndcX + 1.0) * width() / 2.0;
  float screenY = (1.0 - ndcY) * height() / 2.0;

  return QPointF(screenX, screenY);
}

void Drone3DWidget::resizeEvent(QResizeEvent *event) {
  QOpenGLWidget::resizeEvent(event);
}
