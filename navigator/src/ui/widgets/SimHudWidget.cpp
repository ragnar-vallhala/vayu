#include "SimHudWidget.h"

#include "core/Units.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <cmath>

SimHudWidget::SimHudWidget(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_NoSystemBackground);
  setAttribute(Qt::WA_TranslucentBackground);
}

void SimHudWidget::setSnapshot(const vsim::SimSnapshot &s) {
  // NED aerospace extraction — Qt's getEulerAngles (Y-up) permutes the axes for
  // an NED airframe, which rotated the ladder by the drifting heading.
  vsim::quatToEulerNED(s.att, &roll_, &pitch_, &yaw_);
  alt_ = -s.pos_w.z();                        // NED z down → altitude up
  gs_ = std::hypot(s.vel_w.x(), s.vel_w.y()); // ground speed
  vs_ = -s.vel_w.z();                         // +up
  motor_ = s.motor_duty;
  update();
}

void SimHudWidget::setStatus(const QString &s) {
  status_ = s;
  update();
}

void SimHudWidget::setFlightMode(const QString &m) {
  mode_ = m;
  update();
}

void SimHudWidget::setImu(const float acc[3], const float gyr[3]) {
  for (int i = 0; i < 3; ++i) {
    accHist_[i].append(acc[i]);
    if (accHist_[i].size() > kHistN)
      accHist_[i].removeFirst();
    gyrHist_[i].append(gyr[i]);
    if (gyrHist_[i].size() > kHistN)
      gyrHist_[i].removeFirst();
  }
  update();
}

void SimHudWidget::paintEvent(QPaintEvent *) {
  const int W = width(), H = height();
  if (W < 80 || H < 80)
    return;

  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QPointF c(W * 0.5, H * 0.5);
  const QColor hud(90, 255, 160), dim(90, 255, 160, 110), warn(255, 95, 95);
  const QColor box(0, 0, 0, 130);
  QFont font(QStringLiteral("monospace"));
  font.setPixelSize(11);
  font.setBold(true);
  p.setFont(font);

  auto pen = [&](const QColor &col, double w, Qt::PenStyle st = Qt::SolidLine) {
    QPen q(col);
    q.setWidthF(w);
    q.setStyle(st);
    p.setPen(q);
  };

  // ===== Artificial horizon + pitch ladder =====
  {
    p.save();
    // Clip to the central column so the horizon/ladder never paint over
    // the side speed/altitude tapes (which occupy ~80 px each edge),
    // regardless of pitch/roll.
    const double sideMargin = 110.0;
    p.setClipRect(QRectF(sideMargin, 0, W - 2 * sideMargin, H));
    p.translate(c);
    p.rotate(-roll_);
    const double ppd = H / 45.0; // px per pitch degree
    p.translate(0, pitch_ * ppd);
    pen(hud, 2.0);
    // Keep the horizon clear of the side speed/altitude tapes.
    const double hl = W * 0.5 - 115.0;
    p.drawLine(QPointF(-hl, 0), QPointF(-W * 0.05, 0));
    p.drawLine(QPointF(W * 0.05, 0), QPointF(hl, 0));
    for (int a = -90; a <= 90; a += 10) {
      if (a == 0)
        continue;
      const double y = -a * ppd;
      if (std::abs(y) > H * 0.55)
        continue;
      const double half = (a % 20 == 0) ? W * 0.10 : W * 0.06;
      pen(dim, 1.3, a < 0 ? Qt::DashLine : Qt::SolidLine);
      p.drawLine(QPointF(-half, y), QPointF(-W * 0.03, y));
      p.drawLine(QPointF(W * 0.03, y), QPointF(half, y));
      pen(hud, 1.0);
      p.drawText(QRectF(-half - 30, y - 7, 26, 14),
                 Qt::AlignRight | Qt::AlignVCenter, QString::number(a));
      p.drawText(QRectF(half + 4, y - 7, 26, 14),
                 Qt::AlignLeft | Qt::AlignVCenter, QString::number(a));
    }
    p.restore();
  }

  // ===== center boresight =====
  pen(hud, 2.0);
  p.drawLine(QPointF(c.x() - 30, c.y()), QPointF(c.x() - 10, c.y()));
  p.drawLine(QPointF(c.x() + 10, c.y()), QPointF(c.x() + 30, c.y()));
  p.drawLine(QPointF(c.x(), c.y() - 8), QPointF(c.x(), c.y()));
  p.setBrush(hud);
  p.drawEllipse(c, 2.5, 2.5);
  p.setBrush(Qt::NoBrush);

  // ===== roll arc + pointer =====
  {
    const double R = qMin(W, H) * 0.34;
    pen(dim, 1.4);
    p.drawArc(QRectF(c.x() - R, c.y() - R, 2 * R, 2 * R), (90 - 60) * 16,
              120 * 16);
    pen(hud, 1.4);
    for (int a : {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60}) {
      const double sn = std::sin(qDegreesToRadians((double)a));
      const double cs = std::cos(qDegreesToRadians((double)a));
      const double inset = (a % 30 == 0) ? 12 : 7;
      p.drawLine(QPointF(c.x() + R * sn, c.y() - R * cs),
                 QPointF(c.x() + (R - inset) * sn, c.y() - (R - inset) * cs));
    }
    const double rr = std::clamp((double)roll_, -60.0, 60.0);
    const double sn = std::sin(qDegreesToRadians(rr));
    const double cs = std::cos(qDegreesToRadians(rr));
    const QPointF tip(c.x() + R * sn, c.y() - R * cs);
    const QPointF dir(sn, -cs), perp(-dir.y(), dir.x());
    QPolygonF tri;
    tri << tip << (tip - dir * 11 + perp * 6) << (tip - dir * 11 - perp * 6);
    p.setBrush(hud);
    p.drawPolygon(tri);
    p.setBrush(Qt::NoBrush);
  }

  // ===== heading / compass tape (top) =====
  {
    const double hdg = std::fmod(yaw_ + 360.0, 360.0);
    const double tapeW = W * 0.5, ppd = tapeW / 80.0, ty = 16;
    p.save();
    p.setClipRect(QRectF(c.x() - tapeW / 2, ty - 4, tapeW, 34));
    const int base = (int)std::floor(hdg) - 45;
    for (int iv = base; iv <= base + 90; ++iv) {
      const int h = ((iv % 360) + 360) % 360;
      if (h % 5)
        continue;
      const double x = c.x() + (iv - hdg) * ppd;
      if (h % 10 == 0) {
        pen(hud, 1.3);
        p.drawLine(QPointF(x, ty), QPointF(x, ty + 9));
        QString lbl = h == 0          ? "N"
                      : h == 90       ? "E"
                      : h == 180      ? "S"
                      : h == 270      ? "W"
                      : (h % 30 == 0) ? QString::number(h)
                                      : QString();
        if (!lbl.isEmpty())
          p.drawText(QRectF(x - 16, ty + 10, 32, 14),
                     Qt::AlignHCenter | Qt::AlignTop, lbl);
      } else {
        pen(dim, 1.1);
        p.drawLine(QPointF(x, ty), QPointF(x, ty + 5));
      }
    }
    p.restore();
    // caret + numeric heading box
    pen(hud, 1.4);
    QPolygonF car;
    car << QPointF(c.x(), ty + 11) << QPointF(c.x() - 6, ty + 2)
        << QPointF(c.x() + 6, ty + 2);
    p.setBrush(hud);
    p.drawPolygon(car);
    p.setBrush(Qt::NoBrush);
    const QRectF hb(c.x() - 24, ty - 18, 48, 17);
    p.fillRect(hb, box);
    p.drawRect(hb);
    p.drawText(hb, Qt::AlignCenter, QString("%1°").arg((int)std::lround(hdg)));
  }

  // vertical tape helper: side = -1 left (value increases up), +1 right.
  auto tape = [&](double x, int side, double value, double ppu, int majorEvery,
                  const QString &title, int decimals) {
    const double top = H * 0.20, bot = H * 0.80, mid = (top + bot) / 2;
    p.save();
    p.setClipRect(QRectF(x - 60, top - 2, 120, bot - top + 4));
    pen(dim, 1.2);
    p.drawLine(QPointF(x, top), QPointF(x, bot));
    const int lo = (int)std::floor(value - (mid - top) / ppu) - 1;
    const int hi = (int)std::ceil(value + (bot - mid) / ppu) + 1;
    for (int v = lo; v <= hi; ++v) {
      const double y = mid - (v - value) * ppu;
      if (y < top || y > bot)
        continue;
      const bool major = (v % majorEvery == 0);
      const double len = major ? 12 : 6;
      pen(major ? hud : dim, major ? 1.3 : 1.0);
      p.drawLine(QPointF(x, y), QPointF(x + side * len, y));
      if (major && v >= 0)
        p.drawText(QRectF(x + side * 16 - (side < 0 ? 44 : 0), y - 7, 44, 14),
                   (side < 0 ? Qt::AlignRight : Qt::AlignLeft) |
                       Qt::AlignVCenter,
                   QString::number(v));
    }
    p.restore();
    // pointer + value box
    pen(hud, 1.4);
    const QRectF vb(side < 0 ? x - 56 : x + 4, mid - 10, 52, 20);
    p.fillRect(vb, box);
    p.drawRect(vb);
    p.drawText(vb, Qt::AlignCenter, QString::number(value, 'f', decimals));
    QPolygonF ptr;
    ptr << QPointF(x, mid) << QPointF(x + side * 8, mid - 6)
        << QPointF(x + side * 8, mid + 6);
    p.setBrush(hud);
    p.drawPolygon(ptr);
    p.setBrush(Qt::NoBrush);
    p.drawText(QRectF(x - 30, H * 0.20 - 18, 60, 14), Qt::AlignHCenter, title);
  };

  // Units + precision come from Settings ▸ Units & Display (Units:: helper).
  const int dec = Units::decimals();
  tape(60.0, -1, Units::toSpeed(gs_), 12.0, 5,
       tr("SPD ") + Units::speedSuffix(), dec); // left: ground speed
  tape(W - 60.0, +1, Units::toAltitude(alt_), 14.0, 5,
       tr("ALT ") + Units::altSuffix(), dec); // right: altitude

  // vertical speed under the altitude box
  pen(vs_ >= 0 ? hud : warn, 1.2);
  p.drawText(QRectF(W - 86, H * 0.80 + 6, 80, 16), Qt::AlignRight,
             QString("VS %1").arg(Units::toSpeed(vs_), 0, 'f', dec));

  // ===== per-motor throttle bars (bottom-left) =====
  {
    const double bw = 11, gap = 6, bh = 64, bx = 22, by = H - 26;
    for (int i = 0; i < 4; ++i) {
      const double mx = bx + i * (bw + gap);
      pen(dim, 1.0);
      p.drawRect(QRectF(mx, by - bh, bw, bh));
      const double f = std::clamp(motor_[i], 0.0f, 1.0f);
      p.fillRect(QRectF(mx, by - bh * f, bw, bh * f), f > 0.92 ? warn : hud);
      pen(hud, 1.0);
      p.drawText(QRectF(mx - 4, by + 2, bw + 8, 14), Qt::AlignCenter,
                 QString("M%1").arg(i + 1));
    }
  }

  // ===== system status (top-left) =====
  {
    const QString st = status_.isEmpty() ? tr("—") : status_;
    const bool danger = st.contains("FAILSAFE") || st.contains("ARMED") ||
                        st.contains("IN_AIR");
    const QColor scol = danger ? warn : hud;
    const QString label = tr("STATE ") + st;
    const QFontMetrics fm(font);
    const QRectF sb(12, 12, fm.horizontalAdvance(label) + 16, 18);
    p.fillRect(sb, box);
    pen(scol, 1.3);
    p.drawRect(sb);
    p.drawText(sb, Qt::AlignCenter, label);

    // Flight-mode pill just below the state pill (acro highlighted, since it
    // drops the bank-angle limit).
    if (!mode_.isEmpty()) {
      const QString mlabel = tr("MODE ") + mode_;
      const QColor mcol = mode_.contains("ACRO") ? warn : hud;
      const QRectF mb(12, 12 + 18 + 4, fm.horizontalAdvance(mlabel) + 16, 18);
      p.fillRect(mb, box);
      pen(mcol, 1.3);
      p.drawRect(mb);
      p.drawText(mb, Qt::AlignCenter, mlabel);
    }
  }

  // ===== accel / gyro mini-plots (bottom-right) =====
  {
    auto plot = [&](const QRectF &r, const QString &title,
                    const std::array<QVector<float>, 3> &hist,
                    double minRange) {
      p.fillRect(r, box);
      pen(dim, 1.0);
      p.drawRect(r);
      // Auto-range to the largest |sample|, floored so a still vehicle still
      // shows a readable trace.
      double mx = minRange;
      for (int i = 0; i < 3; ++i)
        for (float v : hist[i])
          mx = std::max(mx, (double)std::abs(v));
      const double midY = r.y() + r.height() * 0.58;
      const double plotH = r.height() * 0.38;
      pen(dim, 0.8, Qt::DotLine);
      p.drawLine(QPointF(r.x() + 3, midY), QPointF(r.right() - 3, midY));
      const QColor axc[3] = {QColor(255, 95, 95), QColor(90, 255, 160),
                             QColor(90, 180, 255)};
      for (int i = 0; i < 3; ++i) {
        const QVector<float> &h = hist[i];
        if (h.size() < 2)
          continue;
        QPolygonF poly;
        for (int k = 0; k < h.size(); ++k) {
          const double x =
              r.x() + 4 + (r.width() - 8) * (double)k / (kHistN - 1);
          const double y =
              midY - std::clamp((double)h[k] / mx, -1.0, 1.0) * plotH;
          poly << QPointF(x, y);
        }
        pen(axc[i], 1.2);
        p.drawPolyline(poly);
      }
      pen(hud, 1.0);
      p.drawText(QRectF(r.x() + 4, r.y() + 1, r.width() - 8, 12), Qt::AlignLeft,
                 title);
      pen(dim, 1.0);
      p.drawText(QRectF(r.x() + 4, r.y() + 1, r.width() - 8, 12),
                 Qt::AlignRight,
                 QString::fromUtf8("±%1").arg(mx, 0, 'f', mx < 10 ? 1 : 0));
    };
    const double pw = 210, ph = 46, px = W - pw - 14;
    plot(QRectF(px, H - 118, pw, ph), tr("GYR °/s"), gyrHist_, 30.0);
    plot(QRectF(px, H - 66, pw, ph), tr("ACC m/s²"), accHist_, 2.0);
  }
}
