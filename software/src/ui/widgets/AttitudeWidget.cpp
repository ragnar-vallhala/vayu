#include "AttitudeWidget.h"

#include <QFont>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>

AttitudeWidget::AttitudeWidget(QWidget *parent) : QOpenGLWidget(parent) {
  setMinimumSize(260, 260);
}

void AttitudeWidget::setAttitude(const AttitudeData &att) {
  setAttitude(att.roll, att.pitch, att.yaw);
}

void AttitudeWidget::setAttitude(float roll, float pitch, float yaw) {
  m_roll = roll;
  m_pitch = pitch;
  m_yaw = yaw;
  update();
}

void AttitudeWidget::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
}

void AttitudeWidget::resizeGL(int w, int h) { glViewport(0, 0, w, h); }

void AttitudeWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const int W = width(), H = height();
  const int cx = W / 2, cy = H / 2;
  // Use the smaller dimension for radius so widget is square-ish
  const int r = qMin(W, H) / 2 - 6;

  // Clip to circle
  QPainterPath clip;
  clip.addEllipse(cx - r, cy - r, 2 * r, 2 * r);
  p.setClipPath(clip);

  drawHorizon(p, W, H);
  p.setClipping(false);

  drawBankIndicator(p, cx, cy, r);
  drawCrosshair(p, cx, cy);
  drawCompass(p, W, H);

  // Circle border
  p.setPen(QPen(QColor(180, 190, 200), 2));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r);
}

// ---------------------------------------------------------------------------
void AttitudeWidget::drawHorizon(QPainter &p, int W, int H) {
  const int cx = W / 2, cy = H / 2;
  const int r = qMin(W, H) / 2;

  // Pitch: 1 degree → 3 pixels offset
  const float pixPerDeg = 3.0f;
  const float pitchOffset = m_pitch * pixPerDeg;

  p.save();
  p.translate(cx, cy);
  p.rotate(-m_roll);

  const int BIG = r * 3;

  // Sky
  QLinearGradient skyGrad(0, -BIG, 0, 0);
  skyGrad.setColorAt(0.0, QColor(20, 80, 160));
  skyGrad.setColorAt(1.0, QColor(60, 140, 230));
  p.fillRect(-BIG, -BIG + (int)pitchOffset, 2 * BIG, BIG, skyGrad);

  // Ground
  QLinearGradient gndGrad(0, 0, 0, BIG);
  gndGrad.setColorAt(0.0, QColor(120, 80, 30));
  gndGrad.setColorAt(1.0, QColor(60, 40, 10));
  p.fillRect(-BIG, (int)pitchOffset, 2 * BIG, BIG, gndGrad);

  // Horizon line
  p.setPen(QPen(QColor(255, 230, 100), 2));
  p.drawLine(-BIG, (int)pitchOffset, BIG, (int)pitchOffset);

  // Pitch ladder lines every 10°
  p.setPen(QPen(Qt::white, 1));
  QFont f;
  f.setPixelSize(10);
  p.setFont(f);
  for (int deg = -40; deg <= 40; deg += 10) {
    if (deg == 0)
      continue;
    int y = (int)(pitchOffset - deg * pixPerDeg);
    int len = (qAbs(deg) % 20 == 0) ? 40 : 24;
    p.drawLine(-len, y, len, y);
    p.drawText(len + 4, y + 4, QString::number(deg));
  }

  p.restore();
}

void AttitudeWidget::drawBankIndicator(QPainter &p, int cx, int cy, int r) {
  p.save();
  p.translate(cx, cy);

  // Arc tick marks at 0, ±10, ±20, ±30, ±45, ±60° of bank
  const int angles[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  p.setPen(QPen(QColor(200, 210, 220), 1));
  for (int a : angles) {
    p.save();
    p.rotate(a);
    int tickLen =
        (a == 0 || a == -30 || a == 30 || a == -60 || a == 60) ? 12 : 7;
    p.drawLine(0, -r - 2, 0, -r - 2 - tickLen);
    p.restore();
  }

  // Bank pointer (triangle) shows current roll
  p.rotate(-m_roll);
  QPolygon tri;
  tri << QPoint(0, -(r - 4)) << QPoint(-7, -(r - 18)) << QPoint(7, -(r - 18));
  p.setBrush(QColor(255, 220, 50));
  p.setPen(Qt::NoPen);
  p.drawPolygon(tri);

  p.restore();
}

void AttitudeWidget::drawCrosshair(QPainter &p, int cx, int cy) {
  // Fixed aircraft reference symbol
  p.setPen(QPen(QColor(255, 200, 0), 3, Qt::SolidLine, Qt::RoundCap));

  // Left wing
  p.drawLine(cx - 60, cy, cx - 20, cy);
  p.drawLine(cx - 20, cy, cx - 20, cy + 12);

  // Right wing
  p.drawLine(cx + 60, cy, cx + 20, cy);
  p.drawLine(cx + 20, cy, cx + 20, cy + 12);

  // Centre dot
  p.setBrush(QColor(255, 200, 0));
  p.setPen(Qt::NoPen);
  p.drawEllipse(cx - 5, cy - 5, 10, 10);
}

void AttitudeWidget::drawCompass(QPainter &p, int W, int H) {
  // Thin heading strip at the bottom
  const int stripH = 28;
  const int y0 = H - stripH - 4;

  p.fillRect(0, y0, W, stripH, QColor(20, 24, 32, 210));
  p.setPen(QColor(200, 210, 220));

  QFont f;
  f.setPixelSize(11);
  f.setBold(true);
  p.setFont(f);

  // Draw ±60° of heading labels centred on current yaw
  const float degPerPx = 1.5f;
  for (int d = -60; d <= 60; d += 10) {
    int hdg = ((int)(m_yaw + d) % 360 + 360) % 360;
    int x = W / 2 + (int)(d / degPerPx);
    // Tick
    p.setPen(QPen(QColor(160, 170, 180), 1));
    p.drawLine(x, y0, x, y0 + 8);
    // Label
    p.setPen(QColor(220, 230, 240));
    QString label;
    if (hdg == 0)
      label = "N";
    else if (hdg == 90)
      label = "E";
    else if (hdg == 180)
      label = "S";
    else if (hdg == 270)
      label = "W";
    else
      label = QString::number(hdg);
    p.drawText(x - 10, y0 + 11, 20, 16, Qt::AlignHCenter, label);
  }

  // Centre triangle pointer
  p.setBrush(QColor(255, 220, 50));
  p.setPen(Qt::NoPen);
  QPolygon ptr;
  ptr << QPoint(W / 2 - 5, y0) << QPoint(W / 2 + 5, y0)
      << QPoint(W / 2, y0 + 7);
  p.drawPolygon(ptr);
}
