#include "HorizonHud.h"

#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <cmath>

HorizonHud::HorizonHud(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_TransparentForMouseEvents);
}

void HorizonHud::setAttitude(float rollDeg, float pitchDeg, float yawDeg) {
  roll_ = rollDeg;
  pitch_ = pitchDeg;
  yaw_ = yawDeg;
  update();
}

void HorizonHud::paintEvent(QPaintEvent*) {
  const int W = width(), H = height();
  if (W < 40 || H < 40) return;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const QColor hud(90, 255, 160), dim(90, 255, 160, 120);
  const QColor sky(40, 90, 150), gnd(60, 110, 45), frame(90, 255, 160, 90);
  const QPointF c(W * 0.5, H * 0.5);
  const double ppd = H / 60.0;            // px per pitch degree

  // Rounded frame + clip so the rotating horizon stays inside the box.
  QPainterPath clip;
  clip.addRoundedRect(QRectF(1, 1, W - 2, H - 2), 6, 6);
  p.setClipPath(clip);

  // --- rotating sky/ground + ladder ---
  p.save();
  p.translate(c);
  p.rotate(-roll_);
  p.translate(0, pitch_ * ppd);
  const double BIG = std::hypot(W, H);
  p.fillRect(QRectF(-BIG, -BIG, 2 * BIG, BIG), sky);   // above horizon
  p.fillRect(QRectF(-BIG, 0, 2 * BIG, BIG), gnd);      // below horizon
  QPen hp(hud);
  hp.setWidthF(1.6);
  p.setPen(hp);
  p.drawLine(QPointF(-BIG, 0), QPointF(BIG, 0));       // horizon line
  QFont f(QStringLiteral("monospace"));
  f.setPixelSize(9);
  p.setFont(f);
  for (int a = -60; a <= 60; a += 15) {
    if (a == 0) continue;
    const double y = -a * ppd;
    if (std::abs(y) > H * 0.6) continue;
    const double half = (a % 30 == 0) ? W * 0.16 : W * 0.10;
    QPen lp(dim);
    lp.setWidthF(1.1);
    lp.setStyle(a < 0 ? Qt::DashLine : Qt::SolidLine);
    p.setPen(lp);
    p.drawLine(QPointF(-half, y), QPointF(half, y));
  }
  p.restore();

  // --- fixed boresight ---
  QPen bp(hud);
  bp.setWidthF(2.0);
  p.setPen(bp);
  p.drawLine(QPointF(c.x() - 22, c.y()), QPointF(c.x() - 8, c.y()));
  p.drawLine(QPointF(c.x() + 8, c.y()), QPointF(c.x() + 22, c.y()));
  p.drawLine(QPointF(c.x(), c.y() - 6), QPointF(c.x(), c.y()));

  // --- roll arc + pointer (top) ---
  const double R = std::min(W, H) * 0.40;
  QPen ap(dim);
  ap.setWidthF(1.2);
  p.setPen(ap);
  p.drawArc(QRectF(c.x() - R, c.y() - R, 2 * R, 2 * R), (90 - 50) * 16, 100 * 16);
  for (int a : {-45, -30, -15, 0, 15, 30, 45}) {
    const double sn = std::sin(qDegreesToRadians((double)a));
    const double cs = std::cos(qDegreesToRadians((double)a));
    const double inset = (a % 30 == 0) ? 7 : 4;
    p.drawLine(QPointF(c.x() + R * sn, c.y() - R * cs),
               QPointF(c.x() + (R - inset) * sn, c.y() - (R - inset) * cs));
  }
  const double rr = std::clamp((double)roll_, -50.0, 50.0);
  const double sn = std::sin(qDegreesToRadians(rr)), cs = std::cos(qDegreesToRadians(rr));
  const QPointF tip(c.x() + R * sn, c.y() - R * cs);
  const QPointF dir(sn, -cs), perp(-dir.y(), dir.x());
  QPolygonF tri;
  tri << tip << (tip - dir * 8 + perp * 4) << (tip - dir * 8 - perp * 4);
  p.setBrush(hud);
  p.setPen(Qt::NoPen);
  p.drawPolygon(tri);
  p.setBrush(Qt::NoBrush);

  // --- heading readout (bottom) ---
  const double hdg = std::fmod(yaw_ + 360.0, 360.0);
  QPen tp(hud);
  tp.setWidthF(1.0);
  p.setPen(tp);
  f.setPixelSize(10);
  f.setBold(true);
  p.setFont(f);
  p.drawText(QRectF(0, H - 15, W, 13), Qt::AlignHCenter,
             QString("HDG %1°").arg((int)std::lround(hdg)));

  // --- frame ---
  p.setClipping(false);
  QPen fp(frame);
  fp.setWidthF(1.2);
  p.setPen(fp);
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(QRectF(1, 1, W - 2, H - 2), 6, 6);
}
