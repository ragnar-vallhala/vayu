#include "AttitudeWidget.h"

#include <QFont>
#include <QFontMetrics>
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
  const int tapeH = 26;                        // bottom heading-tape strip
  const QRectF adi(2, 2, W - 4, H - tapeH - 4);  // horizon viewport
  const QRectF tape(2, H - tapeH, W - 4, tapeH - 2);

  // Horizon + pitch ladder clipped to the rounded ADI viewport.
  p.save();
  QPainterPath clip;
  clip.addRoundedRect(adi, 10, 10);
  p.setClipPath(clip);
  drawHorizon(p, adi);
  drawPitchLadder(p, adi);
  p.restore();

  // Overlays (unclipped): roll arc on top, fixed wings, viewport border.
  drawRollArc(p, adi);
  drawAircraftRef(p, adi);
  p.setPen(QPen(QColor(0x3E, 0x44, 0x52), 1.5));
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(adi, 10, 10);

  drawHeadingTape(p, tape);
}

// ---------------------------------------------------------------------------
void AttitudeWidget::drawHorizon(QPainter &p, const QRectF &adi) {
  const double pixPerDeg = adi.height() * 0.012;  // ~ ±40° spans the viewport
  const double pitchOffset = m_pitch * pixPerDeg;
  const double cx = adi.center().x(), cy = adi.center().y();
  const double BIG = qMax(adi.width(), adi.height()) * 2.0;

  p.save();
  p.translate(cx, cy);
  p.rotate(-m_roll);

  QLinearGradient sky(0, -BIG, 0, 0);
  sky.setColorAt(0.0, QColor(0x2A, 0x6C, 0xB0));
  sky.setColorAt(1.0, QColor(0x5C, 0x9F, 0xE0));
  p.fillRect(QRectF(-BIG, -BIG + pitchOffset, 2 * BIG, BIG), sky);

  QLinearGradient gnd(0, 0, 0, BIG);
  gnd.setColorAt(0.0, QColor(0x7A, 0x57, 0x2E));
  gnd.setColorAt(1.0, QColor(0x4A, 0x34, 0x16));
  p.fillRect(QRectF(-BIG, pitchOffset, 2 * BIG, BIG), gnd);

  p.setPen(QPen(QColor(0xF5, 0xE6, 0x9B), 2));
  p.drawLine(QPointF(-BIG, pitchOffset), QPointF(BIG, pitchOffset));
  p.restore();
}

void AttitudeWidget::drawPitchLadder(QPainter &p, const QRectF &adi) {
  const double pixPerDeg = adi.height() * 0.012;
  const double pitchOffset = m_pitch * pixPerDeg;

  p.save();
  p.translate(adi.center());
  p.rotate(-m_roll);
  p.setPen(QPen(QColor(255, 255, 255, 220), 1));
  QFont f;
  f.setPixelSize(10);
  p.setFont(f);
  for (int deg = -40; deg <= 40; deg += 10) {
    if (deg == 0) continue;
    const double y = pitchOffset - deg * pixPerDeg;
    const double len = (qAbs(deg) % 20 == 0) ? 42 : 24;
    p.drawLine(QPointF(-len, y), QPointF(len, y));
    const QString s = QString::number(qAbs(deg));
    p.drawText(QRectF(len + 4, y - 7, 26, 14), Qt::AlignLeft | Qt::AlignVCenter, s);
    p.drawText(QRectF(-len - 30, y - 7, 26, 14), Qt::AlignRight | Qt::AlignVCenter, s);
  }
  p.restore();
}

void AttitudeWidget::drawRollArc(QPainter &p, const QRectF &adi) {
  const double cx = adi.center().x();
  const double cy = adi.center().y();
  const double r = adi.width() * 0.42;

  // Fixed roll-scale ticks across the top arc (0 at top, ±10/20/30/45/60).
  p.setPen(QPen(QColor(210, 218, 228), 1.2));
  const int angles[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  for (int a : angles) {
    const double rad = qDegreesToRadians(-90.0 + a);  // -90 = straight up
    const bool major = (a == 0 || qAbs(a) == 30 || qAbs(a) == 60);
    const double len = major ? 11 : 6;
    const QPointF o(cx + r * qCos(rad), cy + r * qSin(rad));
    const QPointF i(cx + (r - len) * qCos(rad), cy + (r - len) * qSin(rad));
    p.drawLine(i, o);
  }

  // Bank pointer (triangle) hanging from the top, rotated by current roll.
  p.save();
  p.translate(cx, cy);
  p.rotate(-m_roll);
  QPolygonF tri;
  tri << QPointF(0, -r + 2) << QPointF(-7, -r + 15) << QPointF(7, -r + 15);
  p.setBrush(QColor(0xFF, 0xD4, 0x32));
  p.setPen(Qt::NoPen);
  p.drawPolygon(tri);
  p.restore();
}

void AttitudeWidget::drawAircraftRef(QPainter &p, const QRectF &adi) {
  const double cx = adi.center().x(), cy = adi.center().y();
  p.setPen(QPen(QColor(0xFF, 0xC8, 0x00), 3, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(QPointF(cx - 58, cy), QPointF(cx - 22, cy));
  p.drawLine(QPointF(cx - 22, cy), QPointF(cx - 22, cy + 11));
  p.drawLine(QPointF(cx + 58, cy), QPointF(cx + 22, cy));
  p.drawLine(QPointF(cx + 22, cy), QPointF(cx + 22, cy + 11));
  p.setBrush(QColor(0xFF, 0xC8, 0x00));
  p.setPen(Qt::NoPen);
  p.drawEllipse(QPointF(cx, cy), 4, 4);
}

void AttitudeWidget::drawHeadingTape(QPainter &p, const QRectF &tape) {
  p.save();
  p.setClipRect(tape);
  p.fillRect(tape, QColor(0x16, 0x1A, 0x22));

  const double span = 160.0;  // degrees visible across the strip
  const double pxPerDeg = tape.width() / span;
  const double cx = tape.center().x();

  QFont f;
  f.setPixelSize(10);
  f.setBold(true);
  p.setFont(f);

  // Ticks every 5°, labels every 30° (cardinal letters at the quadrants).
  const int from = int(m_yaw - span / 2) - 5;
  const int to = int(m_yaw + span / 2) + 5;
  for (int d = from; d <= to; ++d) {
    int hdg = ((d % 360) + 360) % 360;
    if (hdg % 5 != 0) continue;
    const double x = cx + (d - m_yaw) * pxPerDeg;
    const bool major = (hdg % 30 == 0);
    p.setPen(QPen(QColor(0x8A, 0x92, 0xA6), 1));
    p.drawLine(QPointF(x, tape.top()), QPointF(x, tape.top() + (major ? 7 : 4)));
    if (major) {
      QString lbl;
      if (hdg == 0) lbl = "N";
      else if (hdg == 90) lbl = "E";
      else if (hdg == 180) lbl = "S";
      else if (hdg == 270) lbl = "W";
      else lbl = QString::number(hdg);
      p.setPen(QColor(0xCF, 0xD7, 0xE3));
      p.drawText(QRectF(x - 20, tape.top() + 7, 40, tape.height() - 7),
                 Qt::AlignHCenter | Qt::AlignVCenter, lbl);
    }
  }

  // Centre pointer (current heading).
  p.setBrush(QColor(0xFF, 0xD4, 0x32));
  p.setPen(Qt::NoPen);
  QPolygonF ptr;
  ptr << QPointF(cx, tape.top() + 7) << QPointF(cx - 6, tape.top())
      << QPointF(cx + 6, tape.top());
  p.drawPolygon(ptr);
  p.restore();
}
