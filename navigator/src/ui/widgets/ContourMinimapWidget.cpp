#include "ContourMinimapWidget.h"

#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {
constexpr int kGrid = 112; // height samples per side (scaled up to the widget)

// Hypsometric ramp: low ground green -> meadow yellow -> tan -> rocky grey ->
// snow white, by normalised elevation t in [0,1].
QColor hypsometric(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  struct Stop {
    float t;
    int r, g, b;
  };
  static const Stop stops[] = {{0.00f, 36, 74, 38},    {0.30f, 86, 120, 46},
                               {0.55f, 170, 160, 86},  {0.75f, 150, 120, 80},
                               {0.90f, 130, 120, 116}, {1.00f, 236, 236, 240}};
  for (int i = 1; i < 6; ++i) {
    if (t <= stops[i].t) {
      const Stop &a = stops[i - 1];
      const Stop &b = stops[i];
      const float u = (t - a.t) / (b.t - a.t);
      return QColor(int(a.r + (b.r - a.r) * u), int(a.g + (b.g - a.g) * u),
                    int(a.b + (b.b - a.b) * u));
    }
  }
  return QColor(stops[5].r, stops[5].g, stops[5].b);
}
} // namespace

ContourMinimapWidget::ContourMinimapWidget(QWidget *parent) : QWidget(parent) {
  // Clicks pass through to the renderer underneath (matches the HUD overlays).
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setMinimumSize(80, 80);
}

void ContourMinimapWidget::setSampler(std::function<float(float, float)> s) {
  sampler_ = std::move(s);
  update();
}

void ContourMinimapWidget::setView(float north, float east, float headingRad) {
  n_ = north;
  e_ = east;
  headingRad_ = headingRad;
  update();
}

void ContourMinimapWidget::setRangeM(float r) {
  rangeM_ = r > 1.0f ? r : 1.0f;
  update();
}

void ContourMinimapWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const int side = std::min(width(), height());
  const QRect box((width() - side) / 2, (height() - side) / 2, side, side);

  p.fillRect(rect(), QColor(17, 20, 27));
  if (!sampler_) {
    p.setPen(QColor(138, 146, 166));
    p.drawText(rect(), Qt::AlignCenter, tr("no procedural terrain"));
    return;
  }

  // Sample a north-up height grid centred on (n_, e_). Screen +x = east,
  // screen +y = south (so north is up). Cell (gx,gy):
  //   east  = e_ - range + (gx/(G-1)) * 2*range
  //   north = n_ + range - (gy/(G-1)) * 2*range
  const int G = kGrid;
  const float span = 2.0f * rangeM_;
  const float stepM = span / static_cast<float>(G - 1);
  std::vector<float> h(static_cast<size_t>(G) * G);
  float hmin = 1e9f, hmax = -1e9f;
  for (int gy = 0; gy < G; ++gy) {
    const float north = n_ + rangeM_ - gy * stepM;
    for (int gx = 0; gx < G; ++gx) {
      const float east = e_ - rangeM_ + gx * stepM;
      const float v = sampler_(north, east);
      h[static_cast<size_t>(gy) * G + gx] = v;
      hmin = std::min(hmin, v);
      hmax = std::max(hmax, v);
    }
  }

  // Light from the NW for hillshade (screen space: up-left).
  const float lx = -0.6f, ly = -0.6f, lz = 1.0f;
  QImage img(G, G, QImage::Format_RGB32);
  for (int gy = 0; gy < G; ++gy) {
    for (int gx = 0; gx < G; ++gx) {
      const float c = h[static_cast<size_t>(gy) * G + gx];
      const float hl = h[static_cast<size_t>(gy) * G + std::max(0, gx - 1)];
      const float hr = h[static_cast<size_t>(gy) * G + std::min(G - 1, gx + 1)];
      const float hu = h[static_cast<size_t>(std::max(0, gy - 1)) * G + gx];
      const float hd = h[static_cast<size_t>(std::min(G - 1, gy + 1)) * G + gx];
      // Surface gradient in screen space (y grows downward = south).
      const float dzdx = (hr - hl) / (2.0f * stepM);
      const float dzdy = (hd - hu) / (2.0f * stepM);
      float nx = -dzdx, ny = -dzdy, nz = 1.0f;
      const float inv = 1.0f / std::sqrt(nx * nx + ny * ny + nz * nz);
      float ndl = (nx * lx + ny * ly + nz * lz) * inv * 0.57735f;
      ndl = std::clamp(ndl, 0.0f, 1.0f);
      const float shade = 0.55f + 0.45f * ndl;

      QColor col = hypsometric(c / refHeightM_);
      // Contour line: darken where the contour band index changes vs. the east
      // / south neighbour.
      const int band = static_cast<int>(std::floor(c / contourM_));
      const int bandR = static_cast<int>(std::floor(hr / contourM_));
      const int bandD = static_cast<int>(std::floor(hd / contourM_));
      float k = shade;
      if (band != bandR || band != bandD)
        k *= 0.55f; // contour ink
      img.setPixel(gx, gy,
                   qRgb(int(std::clamp(col.red() * k, 0.0f, 255.0f)),
                        int(std::clamp(col.green() * k, 0.0f, 255.0f)),
                        int(std::clamp(col.blue() * k, 0.0f, 255.0f))));
    }
  }
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.drawImage(box, img);

  // Frame.
  p.setPen(QColor(62, 68, 82));
  p.setBrush(Qt::NoBrush);
  p.drawRect(box.adjusted(0, 0, -1, -1));

  // Centre marker: a triangle pointing along heading (north-up, +heading east =
  // clockwise). Drawn at the box centre.
  const QPointF c(box.center());
  p.save();
  p.translate(c);
  p.rotate(headingRad_ * 180.0f / static_cast<float>(M_PI));
  QPolygonF tri;
  tri << QPointF(0, -9) << QPointF(6, 7) << QPointF(0, 3) << QPointF(-6, 7);
  p.setPen(QPen(QColor(20, 24, 30), 1.5));
  p.setBrush(QColor(90, 200, 120));
  p.drawPolygon(tri);
  p.restore();

  // Readouts: centre elevation, scale, and a north tick.
  p.setPen(QColor(220, 224, 232));
  QFont f = p.font();
  f.setPixelSize(10);
  p.setFont(f);
  const float centreH = sampler_(n_, e_);
  p.drawText(box.adjusted(4, 2, -4, -2), Qt::AlignTop | Qt::AlignLeft,
             QStringLiteral("elev %1 m").arg(centreH, 0, 'f', 0));
  p.drawText(box.adjusted(4, 2, -4, -2), Qt::AlignTop | Qt::AlignRight,
             QStringLiteral("N"));
  p.drawText(box.adjusted(4, 2, -4, -3), Qt::AlignBottom | Qt::AlignLeft,
             QStringLiteral("±%1 m").arg(rangeM_, 0, 'f', 0));
  p.drawText(box.adjusted(4, 2, -4, -3), Qt::AlignBottom | Qt::AlignRight,
             QStringLiteral("%1 m contours").arg(contourM_, 0, 'f', 0));
}
