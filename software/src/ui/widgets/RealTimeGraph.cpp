#include "RealTimeGraph.h"

#include <QPainter>
#include <QPainterPath>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

RealTimeGraph::RealTimeGraph(QWidget *parent, int numSeries) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);
  setMinimumHeight(40);
  m_seriesData.resize(numSeries);
  m_sigmaData.resize(numSeries);
  m_colors.resize(numSeries, QColor("#61AFEF"));
  m_penStyles.resize(numSeries, Qt::SolidLine);
}

void RealTimeGraph::setStateBandEnabled(bool on) {
  m_stateBand = on;
  update();
}

void RealTimeGraph::pushState(const QColor &color) {
  if (!m_stateBand)
    return;
  m_stateHist.push_back(color);
  while (static_cast<int>(m_stateHist.size()) > kStateCells)
    m_stateHist.pop_front();
  update();
}

void RealTimeGraph::setSigmaAxis(bool on, float sigmaMax) {
  m_sigmaAxis = on;
  m_sigmaMax = sigmaMax > 1e-6f ? sigmaMax : 1.0f;
  update();
}

void RealTimeGraph::appendSigma(float sigma, int index) {
  if (index < 0 || index >= static_cast<int>(m_sigmaData.size()))
    return;
  m_sigmaData[index].push_back({QDateTime::currentMSecsSinceEpoch(), sigma});
  const qint64 cutoff =
      QDateTime::currentMSecsSinceEpoch() - m_windowSeconds * 1000;
  while (!m_sigmaData[index].empty() &&
         m_sigmaData[index].front().timestamp < cutoff)
    m_sigmaData[index].pop_front();
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

void RealTimeGraph::setYRange(float lo, float hi) {
  m_min = lo;
  m_max = hi;
  m_fixedRange = true;
  m_dynamicYAxis = false;
  update();
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

  // Incremental update of min/max (skipped when the range is pinned).
  if (!m_fixedRange) {
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
  }

  pruneData();
  update();
}

void RealTimeGraph::clear() {
  for (auto &series : m_seriesData) {
    series.clear();
  }
  for (auto &s : m_sigmaData) s.clear();
  m_stateHist.clear();
  if (!m_fixedRange) {
    m_min = -1.0f;
    m_max = 1.0f;
  }
  update();
}

void RealTimeGraph::setDynamicYAxis(bool enabled) {
  m_dynamicYAxis = enabled;
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
  // (never when the range is pinned via setYRange()).
  if (!m_fixedRange && (minMaxPruned || (m_min == m_max && anyPopped))) {
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

  // Vehicle-state colour band behind the traces (mockup .g-status): oldest
  // cell at the left, newest at the right. Tinted so traces stay readable.
  if (m_stateBand && !m_stateHist.empty()) {
    const double cw = double(width()) / kStateCells;
    const int n = static_cast<int>(m_stateHist.size());
    const int off = kStateCells - n;  // right-align the rolling window
    for (int i = 0; i < n; ++i) {
      QColor c = m_stateHist[i];
      c.setAlpha(55);
      painter.fillRect(QRectF((off + i) * cw, 0, cw + 1.0, height()), c);
    }
  }

  // No data yet, or the newest sample has scrolled off the left of the rolling
  // window (the feed stopped) → the plot has nothing live to show. Mark it with
  // a large faded "NA" so a dead/stale trace is unmistakable, then stop.
  qint64 newestTs = std::numeric_limits<qint64>::min();
  for (const auto &series : m_seriesData)
    if (!series.empty())
      newestTs = std::max(newestTs, series.back().timestamp);
  const qint64 nowMsNA = QDateTime::currentMSecsSinceEpoch();
  const bool stale = newestTs < (nowMsNA - qint64(m_windowSeconds) * 1000);
  if (newestTs == std::numeric_limits<qint64>::min() || stale) {
    QFont f = painter.font();
    f.setBold(true);
    f.setPixelSize(std::max(18, height() / 3));
    painter.setFont(f);
    painter.setPen(QColor(0xE8, 0xF0, 0xFE, 40));  // faded watermark
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("NA"));
    return;
  }

  if (m_mode == Mode::LinePlot) {
    // Grid lines
    painter.setPen(QColor(62, 68, 82));
    painter.drawLine(0, height() / 2, width(), height() / 2);

    float drawMin = m_min;
    float drawMax = m_max;

    if (m_dynamicYAxis) {
      drawMin = 9999999.0f;
      drawMax = -9999999.0f;
      for (const auto &series : m_seriesData) {
        for (const auto &dp : series) {
          drawMin = std::min(drawMin, dp.value);
          drawMax = std::max(drawMax, dp.value);
        }
      }
      if (drawMin > drawMax) {
        drawMin = -1.0f;
        drawMax = 1.0f;
      }
      float r = drawMax - drawMin;
      if (r < 1e-6f) {
        drawMin -= 1e-4f;
        drawMax += 1e-4f;
      } else {
        drawMin -= r * 0.1f; // 10% padding
        drawMax += r * 0.1f;
      }
    }

    float range = drawMax - drawMin;
    if (range < 1e-6f)
      range = 1e-6f;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    qint64 startTime = now - (m_windowSeconds * 1000);

    auto toX = [&](qint64 ts) {
      return width() * (ts - startTime) / (m_windowSeconds * 1000.0);
    };

    auto toY = [&](float val) {
      return height() - (height() * (val - drawMin) / range);
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

    // Rolling-σ traces on the right-hand axis [0, sigmaMax] (mockup dotted σ):
    // dotted, reduced opacity, in each series' colour.
    if (m_sigmaAxis) {
      auto toYsig = [&](float s) {
        return height() - (height() * (s / m_sigmaMax));
      };
      for (size_t i = 0; i < m_sigmaData.size(); ++i) {
        if (m_sigmaData[i].empty())
          continue;
        QPainterPath sp;
        bool first = true;
        for (const auto &dp : m_sigmaData[i]) {
          const float x = toX(dp.timestamp);
          const float y = toYsig(dp.value);
          if (first) { sp.moveTo(x, y); first = false; }
          else sp.lineTo(x, y);
        }
        QColor sc = m_colors[i];
        sc.setAlpha(140);
        QPen spen(sc, 1.2, Qt::DotLine);
        spen.setCapStyle(Qt::RoundCap);
        painter.setPen(spen);
        painter.drawPath(sp);
      }
      // Right-hand σ axis labels (σ at top, 0 at bottom).
      QFont sf = painter.font();
      sf.setPointSize(7);
      painter.setFont(sf);
      painter.setPen(QColor(92, 99, 112));
      for (int k = 0; k <= 4; ++k) {
        const float sval = m_sigmaMax - m_sigmaMax * (k / 4.0f);
        const int yPos = static_cast<int>(k / 4.0 * height());
        const QString lbl = (k == 0 ? "σ " : "") +
                            QString::number(double(sval), 'f', 2);
        painter.drawText(QRectF(width() - 42, yPos, 40, 12),
                         Qt::AlignRight | Qt::AlignTop, lbl);
      }
    }

    // Grid labels
    QFont font = painter.font();
    font.setPointSize(7);
    painter.setFont(font);
    painter.setPen(QColor(171, 178, 191, 150));

    auto drawYLabel = [&](float val, int yPos) {
      QString label = QString::number(static_cast<double>(val), 'f', 4);
      painter.drawText(2, yPos, label);
    };

    drawYLabel(drawMax, 10);
    drawYLabel(drawMin, height() - 2);
    drawYLabel((drawMax + drawMin) / 2.0f, height() / 2 - 2);

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

int RealTimeGraph::writeCsv(QTextStream &out, const QStringList &headers) const {
  const int n = static_cast<int>(m_seriesData.size());

  // Header row.
  out << "timestamp_ms";
  for (int i = 0; i < n; ++i) {
    out << ',';
    if (i < headers.size() && !headers[i].isEmpty()) {
      // Quote-escape headers that contain commas or quotes.
      QString h = headers[i];
      if (h.contains(',') || h.contains('"')) {
        h.replace('"', "\"\"");
        out << '"' << h << '"';
      } else {
        out << h;
      }
    } else {
      out << "series_" << i;
    }
  }
  out << '\n';

  // Build a unified timestamp axis: walk all series in parallel, pick
  // the smallest unconsumed timestamp, write one row per distinct ts.
  // Using vector indices instead of iterators so we can advance cheaply.
  std::vector<size_t> idx(n, 0);
  int rows = 0;
  while (true) {
    qint64 next = std::numeric_limits<qint64>::max();
    bool any = false;
    for (int i = 0; i < n; ++i) {
      if (idx[i] < m_seriesData[i].size()) {
        any = true;
        next = std::min(next, m_seriesData[i][idx[i]].timestamp);
      }
    }
    if (!any) break;

    out << next;
    for (int i = 0; i < n; ++i) {
      out << ',';
      if (idx[i] < m_seriesData[i].size() &&
          m_seriesData[i][idx[i]].timestamp == next) {
        // 6 sig figs is plenty for telemetry; saves bytes vs default.
        out << QString::number(m_seriesData[i][idx[i]].value, 'g', 6);
        ++idx[i];
      }
    }
    out << '\n';
    ++rows;
  }
  return rows;
}
