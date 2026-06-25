#pragma once

#include <QColor>
#include <QIcon>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPolygonF>
#include <QRectF>

// Line-icon set mirroring the inline SVGs in the UI mockup (docs/ui-mockup),
// drawn with QPainter primitives in a 24×24 space (no QtSvg dependency). Use
// svgIcon() for a QAction (menus) or svgPixmap() for a QLabel (panel titles).
namespace ui {

enum class Icon {
  Dashboard, Rc, Motors, Control, Packets, Sim, Perf, Calib, Arm, Imu, Attitude,
  // Settings-pane nav icons (mockup parity).
  Streams, Link, Display, Plots, Logging, Alerts, Advanced
};

inline void paintIcon(QPainter &p, Icon id) {
  switch (id) {
    case Icon::Dashboard:
    case Icon::Attitude:
      p.drawEllipse(QPointF(12, 12), 9, 9);
      p.drawLine(QPointF(3, 12), QPointF(21, 12));
      break;
    case Icon::Rc:
      p.drawEllipse(QPointF(12, 12), 3, 3);
      p.drawLine(QPointF(4, 12), QPointF(8, 12));
      p.drawLine(QPointF(16, 12), QPointF(20, 12));
      p.drawLine(QPointF(12, 4), QPointF(12, 8));
      p.drawLine(QPointF(12, 16), QPointF(12, 20));
      break;
    case Icon::Motors:
      p.drawEllipse(QPointF(7, 7), 3, 3);
      p.drawEllipse(QPointF(17, 7), 3, 3);
      p.drawEllipse(QPointF(7, 17), 3, 3);
      p.drawEllipse(QPointF(17, 17), 3, 3);
      break;
    case Icon::Control: {
      QPolygonF poly;
      poly << QPointF(3, 18) << QPointF(8, 9) << QPointF(12, 14)
           << QPointF(15, 7) << QPointF(21, 18);
      p.drawPolyline(poly);
      break;
    }
    case Icon::Packets:
      p.drawLine(QPointF(4, 6), QPointF(20, 6));
      p.drawLine(QPointF(4, 12), QPointF(20, 12));
      p.drawLine(QPointF(4, 18), QPointF(14, 18));
      break;
    case Icon::Sim:
      p.drawEllipse(QPointF(12, 12), 10, 6);  // eye outline (approx)
      p.drawEllipse(QPointF(12, 12), 3, 3);
      break;
    case Icon::Perf: {
      QPolygonF poly;
      poly << QPointF(3, 12) << QPointF(6, 12) << QPointF(8, 5)
           << QPointF(12, 19) << QPointF(14, 12) << QPointF(21, 12);
      p.drawPolyline(poly);
      break;
    }
    case Icon::Calib:
      p.drawEllipse(QPointF(12, 12), 5, 5);
      p.drawLine(QPointF(12, 2), QPointF(12, 5));
      p.drawLine(QPointF(12, 19), QPointF(12, 22));
      p.drawLine(QPointF(2, 12), QPointF(5, 12));
      p.drawLine(QPointF(19, 12), QPointF(22, 12));
      break;
    case Icon::Arm: {
      QPainterPath sh;
      sh.moveTo(12, 2);
      sh.lineTo(3, 6);
      sh.lineTo(3, 12);
      sh.cubicTo(3, 17, 6.5, 20, 12, 22);
      sh.cubicTo(17.5, 20, 21, 17, 21, 12);
      sh.lineTo(21, 6);
      sh.closeSubpath();
      p.drawPath(sh);
      break;
    }
    case Icon::Imu: {
      QPolygonF poly;
      poly << QPointF(3, 12) << QPointF(7, 12) << QPointF(10, 4)
           << QPointF(14, 20) << QPointF(17, 12) << QPointF(21, 12);
      p.drawPolyline(poly);
      break;
    }
    case Icon::Streams: {  // telemetry waveform
      QPolygonF poly;
      poly << QPointF(3, 12) << QPointF(6, 12) << QPointF(8, 5)
           << QPointF(12, 19) << QPointF(14, 12) << QPointF(21, 12);
      p.drawPolyline(poly);
      break;
    }
    case Icon::Link: {  // chain link
      QPainterPath l;
      l.moveTo(10, 8);
      l.lineTo(7, 8);
      l.arcTo(QRectF(3, 8, 8, 8), 90, 180);
      l.lineTo(10, 16);
      QPainterPath r;
      r.moveTo(14, 8);
      r.lineTo(17, 8);
      r.arcTo(QRectF(13, 8, 8, 8), 90, -180);
      r.lineTo(14, 16);
      p.drawPath(l);
      p.drawPath(r);
      p.drawLine(QPointF(9, 12), QPointF(15, 12));
      break;
    }
    case Icon::Display:  // monitor
      p.drawRoundedRect(QRectF(3, 4, 18, 13), 2, 2);
      p.drawLine(QPointF(8, 20), QPointF(16, 20));
      break;
    case Icon::Plots: {  // line chart
      QPolygonF poly;
      poly << QPointF(3, 18) << QPointF(8, 9) << QPointF(12, 14)
           << QPointF(15, 7) << QPointF(21, 18);
      p.drawPolyline(poly);
      break;
    }
    case Icon::Logging:  // document with lines
      p.drawRect(QRectF(4, 4, 16, 16));
      p.drawLine(QPointF(8, 8), QPointF(16, 8));
      p.drawLine(QPointF(8, 12), QPointF(16, 12));
      p.drawLine(QPointF(8, 16), QPointF(13, 16));
      break;
    case Icon::Alerts: {  // bell
      QPainterPath b;
      b.moveTo(12, 4);
      b.arcTo(QRectF(7, 4, 10, 10), 90, 90);   // → (7,9)
      b.lineTo(7, 13);
      b.lineTo(5, 16);
      b.lineTo(19, 16);
      b.lineTo(17, 13);
      b.lineTo(17, 9);
      b.arcTo(QRectF(7, 4, 10, 10), 0, 90);    // → (12,4)
      b.closeSubpath();
      p.drawPath(b);
      QPainterPath clap;
      clap.moveTo(10, 20);
      clap.arcTo(QRectF(10, 18, 4, 4), 180, -180);
      p.drawPath(clap);
      break;
    }
    case Icon::Advanced:  // gear / settings cross
      p.drawEllipse(QPointF(12, 12), 3, 3);
      p.drawLine(QPointF(12, 2), QPointF(12, 5));
      p.drawLine(QPointF(12, 19), QPointF(12, 22));
      p.drawLine(QPointF(2, 12), QPointF(5, 12));
      p.drawLine(QPointF(19, 12), QPointF(22, 12));
      break;
  }
}

inline QIcon svgIcon(Icon id, const QColor &stroke = QColor(0xAB, 0xB2, 0xBF),
                     int px = 18) {
  QPixmap pm(px, px);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing);
  p.scale(px / 24.0, px / 24.0);
  QPen pen(stroke, 2.0);
  pen.setCapStyle(Qt::RoundCap);
  pen.setJoinStyle(Qt::RoundJoin);
  p.setPen(pen);
  p.setBrush(Qt::NoBrush);
  paintIcon(p, id);
  p.end();
  return QIcon(pm);
}

inline QPixmap svgPixmap(Icon id, const QColor &stroke, int px) {
  return svgIcon(id, stroke, px).pixmap(px, px);
}

}  // namespace ui
