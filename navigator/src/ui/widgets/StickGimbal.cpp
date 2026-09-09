#include "StickGimbal.h"

#include <QPainter>
#include <cmath>

#include "core/Theme.h"

StickGimbal::StickGimbal(const QString &caption, QWidget *parent)
    : QWidget(parent), m_caption(caption) {
  setMinimumSize(134, 152);
}

void StickGimbal::setPos(float x, float y) {
  m_haveData = std::isfinite(x) && std::isfinite(y);
  m_x = std::clamp(x, -1.0f, 1.0f);
  m_y = std::clamp(y, -1.0f, 1.0f);
  update();
}

void StickGimbal::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  // Square gimbal box (mockup .gimbal: --base bg, --border-strong, radius 4).
  const int box = std::min(width(), height() - 18);
  const QRectF g((width() - box) / 2.0, 0, box, box);
  p.setPen(QPen(QColor(Theme::hex(Theme::kBorderStrong)), 1));
  p.setBrush(QColor(0x13, 0x14, 0x1B)); // --base
  p.drawRoundedRect(g, 4, 4);

  // Dashed centre crosshair (rgba(255,255,255,.13)).
  QPen dash(QColor(255, 255, 255, 33), 1, Qt::DashLine);
  p.setPen(dash);
  const double cx = g.center().x(), cy = g.center().y();
  p.drawLine(QPointF(g.left() + 6, cy), QPointF(g.right() - 6, cy));
  p.drawLine(QPointF(cx, g.top() + 6), QPointF(cx, g.bottom() - 6));

  if (m_haveData) {
    // Glowing accent dot (16px, #cfe3ff ring, accent fill).
    const double r = box / 2.0 - 10.0;
    const QPointF dot(cx + m_x * r, cy - m_y * r);
    p.setPen(QPen(QColor(0xCF, 0xE3, 0xFF), 2));
    p.setBrush(QColor(Theme::hex(Theme::kAccent)));
    p.drawEllipse(dot, 7, 7);
  } else {
    p.setPen(QColor(Theme::hex(Theme::kTextDim)));
    p.drawText(g, Qt::AlignCenter, "N/A");
  }

  // Caption (mockup .stk-cap).
  QFont f = p.font();
  f.setPointSize(8);
  p.setFont(f);
  p.setPen(QColor(Theme::hex(Theme::kTextDim)));
  p.drawText(QRectF(0, g.bottom() + 3, width(), 14),
             Qt::AlignHCenter | Qt::AlignTop, m_caption);
}
