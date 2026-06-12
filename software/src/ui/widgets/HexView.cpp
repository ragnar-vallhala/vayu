#include "HexView.h"

#include <QFontDatabase>
#include <QPainter>

namespace {
constexpr int kBytesPerRow = 16;
constexpr int kPad = 6;
// Column layout (in character cells): "OOOO  " offset(4)+2sp, then 16×"HH "
// (3 each), then 1 gap, then 16 ascii chars.
constexpr int kHexCol = 6;                       // first hex pair column
constexpr int kAsciiCol = kHexCol + 16 * 3 + 1;  // first ascii column
constexpr int kTotalCols = kAsciiCol + 16;       // line width in chars
} // namespace

HexView::HexView(QWidget *parent) : QWidget(parent) {
  setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  setAttribute(Qt::WA_OpaquePaintEvent);
}

void HexView::setData(const QByteArray &data) {
  m_data = data;
  m_hlOff = -1;
  m_hlLen = 0;
  setMinimumSize(sizeHint()); // let the scroll area scroll tall packets
  updateGeometry();
  update();
}

void HexView::setHighlight(int off, int len) {
  m_hlOff = off;
  m_hlLen = len;
  update();
}

int HexView::charW() const { return fontMetrics().horizontalAdvance('0'); }
int HexView::lineH() const { return fontMetrics().height(); }
int HexView::rows() const {
  return (m_data.size() + kBytesPerRow - 1) / kBytesPerRow;
}

QSize HexView::sizeHint() const {
  return QSize(kTotalCols * charW() + 2 * kPad,
               qMax(1, rows()) * lineH() + 2 * kPad);
}

void HexView::paintEvent(QPaintEvent *) {
  QPainter pnt(this);
  pnt.fillRect(rect(), QColor(33, 37, 43));
  pnt.setFont(font());
  const int cw = charW(), lh = lineH();
  const auto *raw = reinterpret_cast<const uint8_t *>(m_data.constData());

  auto cellX = [&](int col) { return kPad + col * cw; };

  for (int r = 0; r < rows(); ++r) {
    const int y = kPad + r * lh;
    const int base = r * kBytesPerRow;

    // Highlight backgrounds for any selected bytes on this row.
    if (m_hlOff >= 0) {
      for (int k = 0; k < kBytesPerRow && base + k < m_data.size(); ++k) {
        const int idx = base + k;
        if (idx < m_hlOff || idx >= m_hlOff + m_hlLen)
          continue;
        QColor hl(97, 175, 239, 70); // accent, translucent
        pnt.fillRect(cellX(kHexCol + k * 3) - cw / 4, y, cw * 2 + cw / 2, lh,
                     hl);
        pnt.fillRect(cellX(kAsciiCol + k), y, cw, lh, hl);
      }
    }

    // Offset.
    pnt.setPen(QColor(107, 114, 128));
    pnt.drawText(cellX(0), y, cw * 4, lh, Qt::AlignLeft | Qt::AlignVCenter,
                 QString("%1").arg(base, 4, 16, QChar('0')).toUpper());

    // Hex + ASCII.
    for (int k = 0; k < kBytesPerRow && base + k < m_data.size(); ++k) {
      const uint8_t b = raw[base + k];
      const bool lit = (m_hlOff >= 0 && base + k >= m_hlOff &&
                        base + k < m_hlOff + m_hlLen);
      pnt.setPen(lit ? QColor(232, 240, 254) : QColor(171, 178, 191));
      pnt.drawText(cellX(kHexCol + k * 3), y, cw * 2, lh,
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QString("%1").arg(b, 2, 16, QChar('0')).toUpper());
      const QChar c = (b >= 0x20 && b < 0x7f) ? QChar(b) : QChar('.');
      pnt.drawText(cellX(kAsciiCol + k), y, cw, lh,
                   Qt::AlignLeft | Qt::AlignVCenter, c);
    }
  }
}
