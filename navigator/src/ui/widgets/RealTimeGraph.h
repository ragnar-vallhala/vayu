#pragma once

#include <QColor>
#include <QDateTime>
#include <QStringList>
#include <QWidget>
#include <deque>
#include <vector>

class QTextStream;
class QTimer;

class RealTimeGraph : public QWidget {
  Q_OBJECT

public:
  enum class Mode { LinePlot, HorizontalBar };

  explicit RealTimeGraph(QWidget *parent = nullptr, int numSeries = 1);

  void setMode(Mode mode);
  void setWindowSeconds(int seconds);
  void setDropoutRate(double rate);
  void setColor(int index, const QColor &color);
  void setPenStyle(int index, Qt::PenStyle style);
  // Assign a series to an independent right-hand Y axis (default: left). With a
  // right-axis series present, the left and right axes auto-scale from their own
  // series so two very different magnitudes (e.g. AGL ~0 m and MSL ~485 m) are
  // both readable. Requires dynamic-Y. Right-axis ticks render on the right edge.
  void setSeriesAxis(int index, bool rightAxis);
  // In-graph chrome (mockup .graph): a bold title + unit shown top-left and a
  // top-right legend of the per-series names. Drawn as dim overlays — the
  // traces still fill the whole rect, no margins reserved.
  void setTitle(const QString &title, const QString &unit = QString());
  void setSeriesLabels(const QStringList &labels);
  // Pin the Y axis to a fixed [lo, hi] range. Disables the automatic min/max
  // tracking (and dynamic-Y) so the scale stays put — e.g. 0..100 for a
  // percentage. Pass it once after construction.
  void setYRange(float lo, float hi);
  void appendData(float value, int index = 0);
  int numSeries() const { return static_cast<int>(m_seriesData.size()); }
  void clear();
  void setDynamicYAxis(bool enabled);

  // ---- Mockup-parity extensions --------------------------------------------
  // Vehicle-state colour band painted behind the traces (mockup .g-status): a
  // rolling window of status colours, oldest at the left. pushState() appends
  // one cell (call it once per UI tick alongside appendData).
  void setStateBandEnabled(bool on);
  void pushState(const QColor &color);

  // Rolling-σ traces on a right-hand axis [0, sigmaMax] (mockup dotted σ + gvR
  // ticks). appendSigma() feeds the σ sample for a series; the trace is drawn
  // dotted at reduced opacity in the series colour.
  void setSigmaAxis(bool on, float sigmaMax);
  // Toggle the σ overlay on/off without disturbing the configured σ-axis max
  // (used by the Settings "Rolling σ traces" switch).
  void setSigmaEnabled(bool on);
  void appendSigma(float sigma, int index = 0);

  // ---- Settings ▸ Plots & Graphs -------------------------------------------
  // Pen width for the trace lines, and whether painting is antialiased.
  void setTraceWidth(double w);
  void setAntialias(bool on);

  // ---- CSV export (FR-LOG-04 / Phase-1 1d) ---------------------------------
  //
  // Writes the currently-buffered data to `out` as comma-separated
  // rows. First column is `timestamp_ms` (Unix ms UTC), then one
  // column per series. `headers`, if provided, names the per-series
  // columns; missing names fall back to "series_N". Rows are emitted
  // for the union of all series' timestamps, sorted ascending; cells
  // without a sample at that instant are left empty.
  //
  // Returns the number of data rows written (excluding the header).
  int writeCsv(QTextStream &out, const QStringList &headers = {}) const;

  // Public POD so CSV-export helpers in core/CsvExport can iterate
  // without friending or copying.
  struct DataPoint {
    qint64 timestamp;
    float value;
  };

  // Read-only view of the in-memory series buffers. Useful for the
  // shared CSV exporter — single-graph callers should prefer writeCsv()
  // below.
  const std::vector<std::deque<DataPoint>> &series() const {
    return m_seriesData;
  }

protected:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  // Decouple repaints from the append rate: appendData()/pushState() only buffer
  // and set m_dirty; an internal timer repaints at most ~30 Hz, and only while
  // the graph is shown. Keeps per-packet callers from forcing a paint each
  // sample (see docs/ui-rendering-decoupling.md).
  void markDirty() { m_dirty = true; }
  QTimer *m_repaintTimer = nullptr;
  bool m_dirty = false;
  // Set by paintEvent when it draws the "NA" (no-data / scrolled-off) frame, so
  // the repaint pump can stop animating once a dead trace has fully aged out.
  bool m_lastPaintStale = false;
  // Repaint-pump ticks since the last sample. The pump animates the residual
  // scrolling out; once this exceeds one window's worth of ticks it flushes the
  // buffers to "NA" (see the timer lambda). Reset on every new sample.
  int m_idleTicks = 0;

  std::vector<std::deque<DataPoint>> m_seriesData;
  Mode m_mode = Mode::LinePlot;
  int m_windowSeconds = 5;
  double m_dropoutRate = 0.0;
  bool m_dynamicYAxis = false;
  bool m_fixedRange = false; // true once setYRange() pins m_min/m_max
  std::vector<QColor> m_colors;
  std::vector<Qt::PenStyle> m_penStyles;
  // Per-series Y-axis assignment (false = left, true = independent right axis).
  // m_hasRightAxis caches whether any series is on the right axis.
  std::vector<bool> m_rightAxis;
  bool m_hasRightAxis = false;
  QString m_title;
  QString m_unit;
  QStringList m_seriesLabels;

  float m_min = -1.0f;
  float m_max = 1.0f;

  // Plots & Graphs appearance (Settings-driven; defaults match the mockup).
  double m_traceWidth = 1.4;
  bool m_antialias = false;

  // State band (mockup .g-status): timestamped status colours so the band
  // scrolls on the SAME time axis as the traces (same toX mapping + window).
  bool m_stateBand = false;
  struct StateCell {
    qint64 ts;
    QColor color;
  };
  std::deque<StateCell> m_stateHist;

  // Rolling-σ right axis (mockup dotted σ traces + gvR ticks).
  bool m_sigmaAxis = false;
  float m_sigmaMax = 1.0f;
  std::vector<std::deque<DataPoint>> m_sigmaData;

  void pruneData();
};
