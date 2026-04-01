#include "RealTimeGraph.h"
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>
#include <random>

RealTimeGraph::RealTimeGraph(QWidget *parent, int numSeries) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);
  setMinimumHeight(40);
  m_seriesData.resize(numSeries);
  m_colors.resize(numSeries, QColor("#61AFEF"));
  m_penStyles.resize(numSeries, Qt::SolidLine);
}

void RealTimeGraph::setMode(Mode mode) {
  m_mode = mode;
  update();
}

void RealTimeGraph::setWindowSeconds(int seconds) {
  m_windowSeconds = std::max(1, seconds);
  pruneData();
  update();
}

void RealTimeGraph::setDropoutRate(double rate) {
  m_dropoutRate = std::clamp(rate, 0.0, 1.0);
}

void RealTimeGraph::setColor(int index, const QColor &color) {
  if (index >= 0 && index < static_cast<int>(m_colors.size())) {
    m_colors[index] = color;
    update();
  }
}

void RealTimeGraph::setPenStyle(int index, Qt::PenStyle style) {
  if (index >= 0 && index < static_cast<int>(m_penStyles.size())) {
    m_penStyles[index] = style;
    update();
  }
}

void RealTimeGraph::appendData(float value, int index) {
  if (index < 0 || index >= static_cast<int>(m_seriesData.size()))
    return;

  if (m_dropoutRate > 0.0) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    if (dis(gen) < m_dropoutRate) {
      return;
    }
  }

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  m_seriesData[index].push_back({now, value});

  // Incremental update of min/max
  if (m_seriesData[index].size() == 1 && m_min == -1.0f && m_max == 1.0f) {
    m_min = value;
    m_max = value;
    if (std::abs(m_max - m_min) < 0.001f) {
      m_min -= 0.1f;
      m_max += 0.1f;
    }
  } else {
    m_min = std::min(m_min, value);
    m_max = std::max(m_max, value);
  }

  pruneData();
  update();
}

void RealTimeGraph::clear() {
  for (auto &series : m_seriesData) {
    series.clear();
  }
  m_min = -1.0f;
  m_max = 1.0f;
  update();
}

void RealTimeGraph::pruneData() {
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 limit = now - (m_windowSeconds * 1000);

  bool minMaxPruned = false;
  bool anyPopped = false;

  for (auto &series : m_seriesData) {
    while (!series.empty() && series.front().timestamp < limit) {
      float val = series.front().value;
      if (std::abs(val - m_min) < 0.0001f || std::abs(val - m_max) < 0.0001f) {
        minMaxPruned = true;
      }
      series.pop_front();
      anyPopped = true;
    }
  }

  // Only recalculate global min/max if the points we pruned were the min or max
  if (minMaxPruned || (m_min == m_max && anyPopped)) {
    bool first = true;
    for (const auto &series : m_seriesData) {
      for (const auto &dp : series) {
        if (first) {
          m_min = dp.value;
          m_max = dp.value;
          first = false;
        } else {
          m_min = std::min(m_min, dp.value);
          m_max = std::max(m_max, dp.value);
        }
      }
    }
    if (first) { // all empty
      m_min = -1.0f;
      m_max = 1.0f;
    } else if (std::abs(m_max - m_min) < 0.001f) {
      m_min -= 0.1f;
      m_max += 0.1f;
    }
  }
}

void RealTimeGraph::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Background
  painter.fillRect(rect(), QColor(33, 37, 43));

  // Find if we have any data
  bool anyData = false;
  for (const auto &series : m_seriesData) {
    if (!series.empty()) {
      anyData = true;
      break;
    }
  }
  if (!anyData)
    return;

  if (m_mode == Mode::LinePlot) {
    // Grid lines
    painter.setPen(QColor(62, 68, 82));
    painter.drawLine(0, height() / 2, width(), height() / 2);

    float range = m_max - m_min;
    if (range < 0.001f)
      range = 0.001f;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    qint64 startTime = now - (m_windowSeconds * 1000);

    auto toX = [&](qint64 ts) {
      return width() * (ts - startTime) / (m_windowSeconds * 1000.0);
    };

    auto toY = [&](float val) {
      return height() - (height() * (val - m_min) / range);
    };

    // Plot each series
    for (size_t i = 0; i < m_seriesData.size(); ++i) {
      const auto &series = m_seriesData[i];
      if (series.empty())
        continue;

      QPainterPath path;
      bool first = true;
      for (const auto &dp : series) {
        float x = toX(dp.timestamp);
        float y = toY(dp.value);
        if (first) {
          path.moveTo(x, y);
          first = false;
        } else {
          path.lineTo(x, y);
        }
      }

      // Fill under path (optional, maybe only for single series or subtle)
      QPainterPath fillPath = path;
      fillPath.lineTo(toX(series.back().timestamp), height());
      fillPath.lineTo(toX(series.front().timestamp), height());
      fillPath.closeSubpath();

      QLinearGradient gradient(0, 0, 0, height());
      QColor fillColor = m_colors[i];
      fillColor.setAlpha(20); // More transparent for multiple series
      gradient.setColorAt(0, fillColor);
      fillColor.setAlpha(0);
      gradient.setColorAt(1, fillColor);
      painter.fillPath(fillPath, gradient);

      QPen pen(m_colors[i], 1.5, m_penStyles[i]);
      painter.setPen(pen);
      painter.drawPath(path);
    }

    // Grid labels
    QFont font = painter.font();
    font.setPointSize(7);
    painter.setFont(font);
    painter.setPen(QColor(171, 178, 191, 150));

    auto drawYLabel = [&](float val, int yPos) {
      QString label = QString::number(static_cast<double>(val), 'f', 2);
      painter.drawText(2, yPos, label);
    };

    drawYLabel(m_max, 10);
    drawYLabel(m_min, height() - 2);
    drawYLabel((m_max + m_min) / 2.0f, height() / 2 - 2);

  } else {
    // Horizontal Bar mode (Waterfall) - Only supports first series for now
    const auto &series = m_seriesData[0];
    float range = m_max - m_min;
    if (range < 0.001f)
      range = 0.1f;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    float barHeight =
        std::max(1.0f, static_cast<float>(height()) / series.size());

    for (const auto &dp : series) {
      float relativeTime =
          static_cast<float>(now - dp.timestamp) / (m_windowSeconds * 1000.0f);
      float y = height() * relativeTime;
      float barWidth = width() * (dp.value - m_min) / range;

      QRectF bar(0, y, barWidth, barHeight);
      QColor c = m_colors[0];
      c.setAlpha(static_cast<int>(255 * (1.0f - relativeTime)));
      painter.fillRect(bar, c);
    }
    // ... rest of labels ...
  }
}
