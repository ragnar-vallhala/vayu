#include "RealTimeGraph.h"
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>
#include <random>

RealTimeGraph::RealTimeGraph(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);
  setMinimumHeight(40);
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

void RealTimeGraph::setColor(const QColor &color) {
  m_color = color;
  update();
}

void RealTimeGraph::appendData(float value) {
  if (m_dropoutRate > 0.0) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    if (dis(gen) < m_dropoutRate) {
      return;
    }
  }

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  m_data.push_back({now, value});

  // Update min/max for scaling
  if (m_data.size() == 1) {
    m_min = value - 0.1f;
    m_max = value + 0.1f;
  } else {
    m_min = std::min(m_min, value);
    m_max = std::max(m_max, value);
  }

  pruneData();
  update();
}

void RealTimeGraph::clear() {
  m_data.clear();
  m_min = -1.0f;
  m_max = 1.0f;
  update();
}

void RealTimeGraph::pruneData() {
  if (m_data.empty())
    return;

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 limit = now - (m_windowSeconds * 1000);

  bool changed = false;
  while (!m_data.empty() && m_data.front().timestamp < limit) {
    m_data.pop_front();
    changed = true;
  }

  if (changed && !m_data.empty()) {
    // Only recalculate min/max if we removed stuff
    m_min = m_data[0].value;
    m_max = m_data[0].value;
    for (const auto &dp : m_data) {
      m_min = std::min(m_min, dp.value);
      m_max = std::max(m_max, dp.value);
    }
    if (std::abs(m_max - m_min) < 0.001f) {
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

  if (m_data.empty())
    return;

  if (m_mode == Mode::LinePlot) {
    // Grid lines
    painter.setPen(QColor(62, 68, 82));
    painter.drawLine(0, height() / 2, width(), height() / 2);

    // Scaling
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

    // Plot path
    QPainterPath path;
    bool first = true;
    for (const auto &dp : m_data) {
      float x = toX(dp.timestamp);
      float y = toY(dp.value);
      if (first) {
        path.moveTo(x, y);
        first = false;
      } else {
        path.lineTo(x, y);
      }
    }

    // Fill under path
    QPainterPath fillPath = path;
    fillPath.lineTo(toX(m_data.back().timestamp), height());
    fillPath.lineTo(toX(m_data.front().timestamp), height());
    fillPath.closeSubpath();

    QLinearGradient gradient(0, 0, 0, height());
    QColor fillColor = m_color;
    fillColor.setAlpha(40);
    gradient.setColorAt(0, fillColor);
    fillColor.setAlpha(0);
    gradient.setColorAt(1, fillColor);
    painter.fillPath(fillPath, gradient);

    painter.setPen(QPen(m_color, 1.5));
    painter.drawPath(path);

    // Draw grid and labels
    painter.setPen(QColor(62, 68, 82, 100));
    painter.drawLine(0, height() / 2, width(), height() / 2);

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
    // Horizontal Bar mode (Waterfall)
    float range = m_max - m_min;
    if (range < 0.001f)
      range = 0.1f;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    float barHeight =
        std::max(1.0f, static_cast<float>(height()) / m_data.size());

    for (const auto &dp : m_data) {
      float relativeTime =
          static_cast<float>(now - dp.timestamp) / (m_windowSeconds * 1000.0f);
      float y = height() * relativeTime;
      float barWidth = width() * (dp.value - m_min) / range;

      QRectF bar(0, y, barWidth, barHeight);
      QColor c = m_color;
      c.setAlpha(static_cast<int>(255 * (1.0f - relativeTime)));
      painter.fillRect(bar, c);
    }

    // Draw horizontal scale labels
    QFont font = painter.font();
    font.setPointSize(7);
    painter.setFont(font);
    painter.setPen(QColor(171, 178, 191, 180));

    painter.drawText(2, height() - 5,
                     QString::number(static_cast<double>(m_min), 'f', 1));
    painter.drawText(
        width() / 2 - 15, height() - 5,
        QString::number(static_cast<double>((m_min + m_max) / 2.0f), 'f', 1));
    QString maxLabel = QString::number(static_cast<double>(m_max), 'f', 1);
    painter.drawText(width() -
                         painter.fontMetrics().horizontalAdvance(maxLabel) - 2,
                     height() - 5, maxLabel);
  }
}
