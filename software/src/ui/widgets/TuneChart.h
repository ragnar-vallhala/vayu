#pragma once

#include <QPainter>
#include <QVector>
#include <QWidget>
#include <algorithm>
#include <cmath>

// Tiny header-only convergence chart for the Autotune section: scatter of the
// per-evaluation cost plus a best-so-far step line and the baseline. No
// Q_OBJECT (no signals/slots) so it needs neither moc nor a CMake entry.
class TuneChart : public QWidget {
 public:
  explicit TuneChart(QWidget* parent = nullptr) : QWidget(parent) {
    setMinimumHeight(150);
  }

  void reset(double baseline = -1.0) {
    cost_.clear();
    best_.clear();
    baseline_ = baseline;
    update();
  }
  void setBaseline(double b) { baseline_ = b; update(); }
  void addPoint(double cost, double best) {
    cost_.append(cost);
    best_.append(best);
    update();
  }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    const QColor grid(255, 255, 255, 30), hud(90, 255, 160), warn(255, 150, 80),
        base(150, 150, 160), txt(180, 200, 190);
    const int W = width(), H = height();
    const QRectF plot(34, 8, W - 44, H - 26);
    p.fillRect(rect(), QColor(20, 24, 28));
    p.setPen(QPen(grid, 1.0));
    p.drawRect(plot);

    if (cost_.isEmpty()) {
      QFont nf = p.font();
      nf.setBold(true);
      nf.setPixelSize(std::max(18, H / 3));
      p.setFont(nf);
      p.setPen(QColor(0xE8, 0xF0, 0xFE, 40));  // faded watermark
      p.drawText(rect(), Qt::AlignCenter, QStringLiteral("NA"));
      return;
    }

    // Y range: ignore the huge instability-penalty values so the useful band
    // is readable; clamp those to the top.
    double vmax = 1.0, vmin = 1e30;
    auto finite = [](double v) { return v < 1e5; };
    for (double v : cost_) if (finite(v)) { vmax = std::max(vmax, v); vmin = std::min(vmin, v); }
    for (double v : best_) if (finite(v)) { vmin = std::min(vmin, v); }
    if (baseline_ > 0 && finite(baseline_)) { vmax = std::max(vmax, baseline_); vmin = std::min(vmin, baseline_); }
    if (vmin > vmax) vmin = 0;
    const double pad = (vmax - vmin) * 0.08 + 1e-3;
    vmax += pad; vmin = std::max(0.0, vmin - pad);
    const int n = cost_.size();
    auto X = [&](int i) { return plot.left() + plot.width() * (n <= 1 ? 0.5 : double(i) / (n - 1)); };
    auto Y = [&](double v) {
      const double cv = std::min(v, vmax);
      return plot.bottom() - plot.height() * (cv - vmin) / std::max(1e-6, vmax - vmin);
    };

    // axis labels
    p.setPen(txt);
    QFont f = p.font(); f.setPixelSize(9); p.setFont(f);
    p.drawText(QRectF(0, plot.top() - 4, 30, 12), Qt::AlignRight, QString::number(vmax, 'f', 0));
    p.drawText(QRectF(0, plot.bottom() - 8, 30, 12), Qt::AlignRight, QString::number(vmin, 'f', 0));

    if (baseline_ > 0 && finite(baseline_)) {
      p.setPen(QPen(base, 1.0, Qt::DashLine));
      p.drawLine(QPointF(plot.left(), Y(baseline_)), QPointF(plot.right(), Y(baseline_)));
    }
    // cost scatter
    p.setPen(Qt::NoPen);
    for (int i = 0; i < n; ++i) {
      const bool diverged = !finite(cost_[i]);
      p.setBrush(diverged ? warn : QColor(90, 200, 255, 180));
      p.drawEllipse(QPointF(X(i), Y(cost_[i])), 2.4, 2.4);
    }
    // best-so-far line
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(hud, 1.6));
    QPolygonF poly;
    for (int i = 0; i < n; ++i) poly << QPointF(X(i), Y(best_[i]));
    p.drawPolyline(poly);

    p.setPen(txt);
    p.drawText(QRectF(plot.left(), H - 14, plot.width(), 12), Qt::AlignHCenter,
               QString("eval %1   best %2").arg(n).arg(best_.isEmpty() ? 0.0 : best_.last(), 0, 'f', 2));
  }

 private:
  QVector<double> cost_, best_;
  double baseline_ = -1.0;
};
