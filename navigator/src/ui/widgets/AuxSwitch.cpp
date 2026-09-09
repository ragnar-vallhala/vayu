#include "AuxSwitch.h"

#include <QPainter>

#include "core/Theme.h"

AuxSwitch::AuxSwitch(const QString &label, const QStringList &segments,
                     QWidget *parent)
    : QWidget(parent), m_label(label), m_segments(segments) {
  setMinimumHeight(22);
}

void AuxSwitch::setActive(int index) {
  m_active = index;
  update();
}

void AuxSwitch::setFromChannel(int us) {
  if (us <= 0) {
    m_active = -1; // no data
  } else if (m_segments.size() >= 3) {
    m_active = us < 1300 ? 0 : (us > 1700 ? 2 : 1);
  } else {
    m_active = us < 1500 ? 0 : 1;
  }
  update();
}

void AuxSwitch::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QFont f = p.font();
  f.setPointSize(8);
  p.setFont(f);

  // Label on the left (mockup .sw-lbl).
  p.setPen(QColor(Theme::hex(Theme::kTextMuted)));
  const int lblW = 110;
  p.drawText(QRect(0, 0, lblW, height()), Qt::AlignLeft | Qt::AlignVCenter,
             m_label);

  // Segmented control on the right.
  const int n = m_segments.size();
  if (n == 0)
    return;
  const int segW = (width() - lblW) / n;
  const int x0 = lblW;
  const int h = std::min(18, height());
  const int top = (height() - h) / 2;
  const QColor border(Theme::hex(Theme::kBorderStrong));
  for (int i = 0; i < n; ++i) {
    const QRect seg(x0 + i * segW, top, segW, h);
    const bool on = (i == m_active);
    p.fillRect(seg, on ? QColor(0x2A, 0x3A, 0x55) : QColor(0, 0, 0, 0));
    p.setPen(border);
    p.drawRect(seg);
    p.setPen(on ? QColor(0xCF, 0xE3, 0xFF)
                : QColor(Theme::hex(Theme::kTextDim)));
    QFont sf = f;
    sf.setBold(on);
    p.setFont(sf);
    p.drawText(seg, Qt::AlignCenter, m_active < 0 ? "—" : m_segments[i]);
  }
}
