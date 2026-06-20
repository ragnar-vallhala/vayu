#include "RealTimeGraph.h"

#include <QPainter>
#include <QPainterPath>
#include <QTextStream>
#include <QTimer>
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
  m_rightAxis.resize(numSeries, false);

  // Fixed-rate repaint pump (~30 Hz). Started on show, stopped on hide. Only
  // repaints when new data has been buffered since the last paint, so an idle
  // or hidden graph costs nothing.
  m_repaintTimer = new QTimer(this);
  m_repaintTimer->setInterval(33);
  connect(m_repaintTimer, &QTimer::timeout, this, [this] {
    // New samples arrived this tick: paint them and reset the idle counter.
    if (m_dirty) {
      m_dirty = false;
      m_idleTicks = 0;
      update();
      return;
    }
    // The stale "NA" frame is already on screen → nothing to animate; idle.
    if (m_lastPaintStale)
      return;
    // Feed stopped but a trace is still buffered: each idle tick repaints so the
    // line scrolls toward the left edge on the live time axis (washing the
    // residual out of the window). Once the idle ticks exceed one window's worth
    // — enough for the last sample to have fully scrolled off — flush the
    // buffers so the graph is definitively "NA" and the memory is freed.
    bool anyData = false;
    for (const auto &s : m_seriesData)
      if (!s.empty()) { anyData = true; break; }
    if (!anyData)
      return;
    ++m_idleTicks;
    const int washTicks =
        m_windowSeconds * 1000 / m_repaintTimer->interval() + 2;  // +margin
    if (m_idleTicks >= washTicks) {
      for (auto &s : m_seriesData) s.clear();
      for (auto &s : m_sigmaData) s.clear();
      m_stateHist.clear();
    }
    update();
  });
}

void RealTimeGraph::showEvent(QShowEvent *event) {
  QWidget::showEvent(event);
  m_repaintTimer->start();
}

void RealTimeGraph::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
  m_repaintTimer->stop();
}

void RealTimeGraph::setStateBandEnabled(bool on) {
  m_stateBand = on;
  update();
}

void RealTimeGraph::pushState(const QColor &color) {
  if (!m_stateBand)
    return;
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  m_stateHist.push_back({now, color});
  // Drop cells older than the window, but keep the one active at window-start so
  // the band still fills to the left edge (the cell whose successor is also out
  // of the window is the redundant one to evict).
  const qint64 cutoff = now - qint64(m_windowSeconds) * 1000;
  while (m_stateHist.size() > 1 && m_stateHist[1].ts < cutoff)
    m_stateHist.pop_front();
  markDirty();
}

void RealTimeGraph::setSigmaAxis(bool on, float sigmaMax) {
  m_sigmaAxis = on;
  m_sigmaMax = sigmaMax > 1e-6f ? sigmaMax : 1.0f;
  update();
}

void RealTimeGraph::setSigmaEnabled(bool on) {
  m_sigmaAxis = on;
  update();
}

void RealTimeGraph::setTraceWidth(double w) {
  m_traceWidth = std::max(0.1, w);
  update();
}

void RealTimeGraph::setAntialias(bool on) {
  m_antialias = on;
  update();
}

void RealTimeGraph::appendSigma(float sigma, int index) {
  if (index < 0 || index >= static_cast<int>(m_sigmaData.size()))
    return;
  m_sigmaData[index].push_back({QDateTime::currentMSecsSinceEpoch(), sigma});
  // Grow the σ-axis to keep the trace on-scale (never shrinks; mockup stdMax).
  if (sigma > m_sigmaMax)
    m_sigmaMax = sigma * 1.15f;
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

void RealTimeGraph::setTitle(const QString &title, const QString &unit) {
  m_title = title;
  m_unit = unit;
  update();
}

void RealTimeGraph::setSeriesLabels(const QStringList &labels) {
  m_seriesLabels = labels;
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

void RealTimeGraph::setSeriesAxis(int index, bool rightAxis) {
  if (index < 0 || index >= static_cast<int>(m_rightAxis.size()))
    return;
  m_rightAxis[index] = rightAxis;
  m_hasRightAxis = false;
  for (bool r : m_rightAxis)
    m_hasRightAxis = m_hasRightAxis || r;
  update();
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
  markDirty();
}

void RealTimeGraph::clear() {
  for (auto &series : m_seriesData) {
    series.clear();
  }
  for (auto &s : m_sigmaData) s.clear();
  m_stateHist.clear();
  m_idleTicks = 0;
  m_lastPaintStale = false;
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
  painter.setRenderHint(QPainter::Antialiasing, m_antialias);

  // Background
  painter.fillRect(rect(), QColor(33, 37, 43));

  // Shared time→x mapping for the rolling window. The state band AND the traces
  // both use this so they scroll in lockstep (was: band on a fixed 60-cell
  // buffer, traces on time → they drifted apart).
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  const qint64 startTime = now - qint64(m_windowSeconds) * 1000;
  auto toX = [&](qint64 ts) {
    return width() * (ts - startTime) / (m_windowSeconds * 1000.0);
  };

  // Title (bold, accent) — drawn even on a dead/NA graph so each cell is still
  // identifiable; the rest of the chrome only paints once there's live data.
  if (!m_title.isEmpty()) {
    QFont tf = painter.font();
    tf.setPointSize(8);
    tf.setBold(true);
    painter.setFont(tf);
    painter.setPen(QColor(0x61, 0xAF, 0xEF));
    painter.drawText(4, 13, m_title);
  }

  // No data yet, or the newest sample has scrolled off the left of the rolling
  // window (the feed stopped) → the plot has nothing live to show. Mark it with
  // a large faded "NA" so a dead/stale trace is unmistakable, then stop.
  qint64 newestTs = std::numeric_limits<qint64>::min();
  for (const auto &series : m_seriesData)
    if (!series.empty())
      newestTs = std::max(newestTs, series.back().timestamp);
  const bool stale = newestTs < startTime;
  if (newestTs == std::numeric_limits<qint64>::min() || stale) {
    m_lastPaintStale = true;  // let the repaint pump idle until data resumes
    QFont f = painter.font();
    f.setBold(true);
    f.setPixelSize(std::max(18, height() / 3));
    painter.setFont(f);
    painter.setPen(QColor(0xE8, 0xF0, 0xFE, 40));  // faded watermark
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("NA"));
    return;
  }
  m_lastPaintStale = false;  // live data on screen → keep animating

  // Vehicle-state colour band behind the traces (mockup .g-status). Each cell is
  // painted from its own timestamp to the next on the SAME axis as the traces;
  // the oldest cell is clamped to the left edge so there's no gap.
  if (m_stateBand && !m_stateHist.empty()) {
    const int n = static_cast<int>(m_stateHist.size());
    for (int i = 0; i < n; ++i) {
      const double x0 = (i == 0) ? 0.0 : toX(m_stateHist[i].ts);
      const double x1 =
          (i + 1 < n) ? toX(m_stateHist[i + 1].ts) : double(width());
      if (x1 <= x0)
        continue;
      QColor c = m_stateHist[i].color;
      c.setAlpha(55);
      painter.fillRect(QRectF(x0, 0, x1 - x0 + 0.5, height()), c);
    }
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
      for (size_t i = 0; i < m_seriesData.size(); ++i) {
        if (m_hasRightAxis && m_rightAxis[i])
          continue;  // right-axis series scale on their own axis below
        for (const auto &dp : m_seriesData[i]) {
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

    // Independent right-hand axis: scaled only from its own series, so a series
    // with a very different magnitude (e.g. MSL ~485 m) stays readable next to a
    // small left-axis series (AGL ~0 m).
    float rMin = 0.0f, rMax = 1.0f, rRange = 1.0f;
    if (m_hasRightAxis) {
      rMin = 9999999.0f;
      rMax = -9999999.0f;
      for (size_t i = 0; i < m_seriesData.size(); ++i) {
        if (!m_rightAxis[i])
          continue;
        for (const auto &dp : m_seriesData[i]) {
          rMin = std::min(rMin, dp.value);
          rMax = std::max(rMax, dp.value);
        }
      }
      if (rMin > rMax) {
        rMin = -1.0f;
        rMax = 1.0f;
      }
      float rr = rMax - rMin;
      if (rr < 1e-6f) {
        rMin -= 1e-4f;
        rMax += 1e-4f;
      } else {
        rMin -= rr * 0.1f;
        rMax += rr * 0.1f;
      }
      rRange = rMax - rMin;
      if (rRange < 1e-6f)
        rRange = 1e-6f;
    }

    // now / startTime / toX are hoisted above (shared with the state band).
    auto toY = [&](float val, bool rightAxis) -> float {
      if (rightAxis)
        return height() - (height() * (val - rMin) / rRange);
      return height() - (height() * (val - drawMin) / range);
    };

    // Plot each series
    for (size_t i = 0; i < m_seriesData.size(); ++i) {
      const auto &series = m_seriesData[i];
      if (series.empty())
        continue;
      const bool ra = m_hasRightAxis && m_rightAxis[i];

      QPainterPath path;
      bool first = true;
      for (const auto &dp : series) {
        float x = toX(dp.timestamp);
        float y = toY(dp.value, ra);
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

      // Right-axis series are drawn DOTTED (like the σ traces) so it's obvious
      // at a glance which traces read against the right-hand scale vs the left.
      QPen pen(m_colors[i], m_traceWidth, ra ? Qt::DotLine : m_penStyles[i]);
      painter.setPen(pen);
      painter.drawPath(path);
    }

    // Rolling-σ traces on an INDEPENDENT right-hand axis (mockup dotted σ):
    // dotted, reduced opacity, in each series' colour. The σ axis autoscales to
    // the σ data's own [min,max] each frame (not a fixed [0,σmax]) so the traces
    // use the full panel height instead of clamping at the bottom when σ is small
    // — the σ magnitudes are unrelated to the main trace scale.
    if (m_sigmaAxis) {
      float sMin = std::numeric_limits<float>::max();
      float sMax = -std::numeric_limits<float>::max();
      for (const auto &s : m_sigmaData)
        for (const auto &dp : s) {
          sMin = std::min(sMin, dp.value);
          sMax = std::max(sMax, dp.value);
        }
      if (sMin <= sMax) {  // at least one σ sample in the window
        float sr = sMax - sMin;
        if (sr < 1e-6f) {           // flat trace: give it a sliver of range
          sMin -= 1e-4f;
          sMax += 1e-4f;
        } else {                    // 10% padding top & bottom
          sMin -= sr * 0.1f;
          sMax += sr * 0.1f;
        }
        float srange = sMax - sMin;
        if (srange < 1e-6f)
          srange = 1e-6f;
        auto toYsig = [&](float s) {
          return height() - (height() * (s - sMin) / srange);
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
        // Right-hand σ axis labels (σ max at top, min at bottom).
        QFont sf = painter.font();
        sf.setPointSize(7);
        painter.setFont(sf);
        painter.setPen(QColor(92, 99, 112));
        for (int k = 0; k <= 4; ++k) {
          const float sval = sMax - (sMax - sMin) * (k / 4.0f);
          const int yPos = static_cast<int>(k / 4.0 * height());
          const QString lbl = (k == 0 ? "σ " : "") +
                              QString::number(double(sval), 'f', 2);
          painter.drawText(QRectF(width() - 42, yPos, 40, 12),
                           Qt::AlignRight | Qt::AlignTop, lbl);
        }
      }
    }

    // Independent right-hand axis tick labels (right edge), tinted with the
    // right-axis series' colour so it's clear which trace they belong to.
    if (m_hasRightAxis) {
      QColor axc(120, 128, 142);
      for (size_t i = 0; i < m_rightAxis.size(); ++i)
        if (m_rightAxis[i] && i < m_colors.size()) {
          axc = m_colors[i];
          break;
        }
      QFont rf = painter.font();
      rf.setPointSize(7);
      painter.setFont(rf);
      painter.setPen(axc);
      for (int k = 0; k <= 4; ++k) {
        const float rval = rMax - (rMax - rMin) * (k / 4.0f);
        const int yPos = k == 0 ? 14 : static_cast<int>(k / 4.0 * height());
        painter.drawText(QRectF(width() - 46, yPos, 44, 12),
                         Qt::AlignRight | Qt::AlignTop,
                         QString::number(double(rval), 'f', 1));
      }
    }

    // ---- In-graph chrome (mockup .graph): y-ticks, title, unit, legend, time
    // axis. All dim overlays; the traces fill the full rect underneath. ----
    QFont font = painter.font();
    font.setPointSize(7);
    painter.setFont(font);

    auto fmt = [](float v) {
      return QString::number(double(v), 'f',
                             std::abs(v - std::round(v)) < 0.05f ? 0 : 2);
    };

    // Y-axis tick labels (left edge). Shift the top one down when a title sits
    // in the corner so they don't collide.
    painter.setPen(QColor(171, 178, 191, 150));
    const int yTop = m_title.isEmpty() ? 10 : 24;
    painter.drawText(3, yTop, fmt(drawMax));
    painter.drawText(3, height() / 2 + 3, fmt((drawMax + drawMin) / 2.0f));
    painter.drawText(3, height() - 16, fmt(drawMin));
    // (title is drawn earlier, above the NA check)

    // Legend (top-right): coloured series names right-to-left, then a dotted σ.
    {
      QFontMetrics fm(font);
      int lx = width() - 6;
      if (m_sigmaAxis) {
        painter.setPen(QColor(150, 150, 150));
        lx -= fm.horizontalAdvance(QStringLiteral("σ")) + 7;
        painter.drawText(lx, 12, QStringLiteral("σ"));
      } else if (!m_unit.isEmpty()) {
        painter.setPen(QColor(120, 128, 142));
        lx -= fm.horizontalAdvance(m_unit) + 7;
        painter.drawText(lx, 12, m_unit);
      }
      for (int i = m_seriesLabels.size() - 1; i >= 0; --i) {
        painter.setPen(i < static_cast<int>(m_colors.size()) ? m_colors[i]
                                                             : QColor(200, 200, 200));
        lx -= fm.horizontalAdvance(m_seriesLabels[i]) + 7;
        painter.drawText(lx, 12, m_seriesLabels[i]);
      }
    }

    // Time axis (bottom): -Ns at the left, 0s at the right.
    painter.setPen(QColor(120, 128, 142));
    painter.drawText(3, height() - 3, QString("-%1s").arg(m_windowSeconds));
    painter.drawText(QRectF(0, height() - 13, width() - 4, 11),
                     Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("0s"));

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
