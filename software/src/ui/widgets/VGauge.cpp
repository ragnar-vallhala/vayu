#include "VGauge.h"

#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>

VGauge::VGauge(const QString &caption, double minVal, double maxVal,
               const QString &unit, Palette pal, QWidget *parent)
    : QWidget(parent), m_caption(caption), m_unit(unit), m_min(minVal),
      m_max(maxVal), m_value(minVal), m_palette(pal) {
  setMinimumWidth(52);
  setValue(minVal);
}

void VGauge::setValue(double v) {
  m_value = std::clamp(v, m_min, m_max);
  if (!m_readoutPinned) {
    if (m_unit == "%")
      m_readout = QString("%1%").arg(m_value, 0, 'f', 0);
    else
      m_readout = QString("%1%2").arg(m_value, 0, 'f', 1).arg(m_unit);
  }
  update();
}

void VGauge::setReadoutText(const QString &text) {
  m_readout = text;
  m_readoutPinned = true;
  update();
}

void VGauge::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const int W = width();
  const int H = height();
  const int capH = 26;   // caption band (two lines fit)
  const int readH = 16;  // readout band
  const int tickW = 16;  // tick-label gutter on the left

  QFont f = p.font();

  // ---- Caption (top, dim, bold) ----
  f.setPixelSize(9);
  f.setBold(true);
  p.setFont(f);
  p.setPen(QColor(0x8A, 0x92, 0xA6));
  p.drawText(QRect(0, 0, W, capH), Qt::AlignHCenter | Qt::AlignVCenter,
             m_caption);

  // ---- Mid region: tick labels + track ----
  const int y0 = capH + 2;
  const int y1 = H - readH - 2;
  const int midH = y1 - y0;
  if (midH <= 8) return;

  const QRectF track(tickW + 3, y0, W - (tickW + 3) - 1, midH);

  // Track background + border.
  p.setPen(QPen(QColor(0x3E, 0x44, 0x52), 1));
  p.setBrush(QColor(0x1A, 0x1D, 0x27));
  p.drawRoundedRect(track, 6, 6);

  // Gradient fill, anchored to the fill region (bottom → top), clipped to the
  // value fraction — matches the mockup's .tg-fill behaviour.
  const double frac =
      (m_max > m_min) ? (m_value - m_min) / (m_max - m_min) : 0.0;
  const double fillH = frac * (track.height() - 2);
  if (fillH > 0.5) {
    const QRectF fill(track.left() + 1, track.bottom() - 1 - fillH,
                      track.width() - 2, fillH);
    QLinearGradient g(fill.bottomLeft(), fill.topLeft());
    if (m_palette == Battery) {
      g.setColorAt(0.0, QColor(0xE0, 0x6C, 0x75));
      g.setColorAt(0.35, QColor(0xD1, 0x9A, 0x66));
      g.setColorAt(0.62, QColor(0x98, 0xC3, 0x79));
      g.setColorAt(1.0, QColor(0x98, 0xC3, 0x79));
    } else {  // Temp
      g.setColorAt(0.0, QColor(0x3A, 0x5F, 0x8F));
      g.setColorAt(0.25, QColor(0x61, 0xAF, 0xEF));
      g.setColorAt(0.5, QColor(0x98, 0xC3, 0x79));
      g.setColorAt(0.75, QColor(0xD1, 0x9A, 0x66));
      g.setColorAt(1.0, QColor(0xE0, 0x6C, 0x75));
    }
    QPainterPath clip;
    clip.addRoundedRect(track.adjusted(1, 1, -1, -1), 5, 5);
    p.save();
    p.setClipPath(clip);
    p.fillRect(fill, g);
    p.restore();
  }

  // Quarter tick marks across the track.
  p.setPen(QColor(255, 255, 255, 20));
  for (int q = 1; q <= 3; ++q) {
    const double ty = track.top() + track.height() * q / 4.0;
    p.drawLine(QPointF(track.left() + 1, ty), QPointF(track.right() - 1, ty));
  }

  // Tick labels (max … 0) in the left gutter, aligned to the track edges.
  f.setPixelSize(8);
  f.setBold(false);
  p.setFont(f);
  p.setPen(QColor(0x8A, 0x92, 0xA6));
  for (int i = 0; i <= 4; ++i) {
    const double val = m_max - i * (m_max - m_min) / 4.0;
    const double ty = track.top() + track.height() * i / 4.0;
    p.drawText(QRectF(0, ty - 6, tickW, 12), Qt::AlignRight | Qt::AlignVCenter,
               QString::number(val, 'f', 0));
  }

  // ---- Readout (bottom, bold) ----
  f.setPixelSize(11);
  f.setBold(true);
  p.setFont(f);
  p.setPen(QColor(0xE8, 0xF0, 0xFE));
  p.drawText(QRect(0, H - readH, W, readH), Qt::AlignHCenter | Qt::AlignVCenter,
             m_readout);
}
