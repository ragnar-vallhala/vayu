#pragma once

#include <QColor>
#include <QDateTime>
#include <QStringList>
#include <QWidget>
#include <deque>
#include <vector>

class QTextStream;

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
  void appendSigma(float sigma, int index = 0);

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
  const std::vector<std::deque<DataPoint>>& series() const {
    return m_seriesData;
  }

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  std::vector<std::deque<DataPoint>> m_seriesData;
  Mode m_mode = Mode::LinePlot;
  int m_windowSeconds = 5;
  double m_dropoutRate = 0.0;
  bool m_dynamicYAxis = false;
  bool m_fixedRange = false; // true once setYRange() pins m_min/m_max
  std::vector<QColor> m_colors;
  std::vector<Qt::PenStyle> m_penStyles;

  float m_min = -1.0f;
  float m_max = 1.0f;

  // State band (mockup .g-status): rolling status colours, capped at kStateCells.
  bool m_stateBand = false;
  std::deque<QColor> m_stateHist;
  static constexpr int kStateCells = 60;

  // Rolling-σ right axis (mockup dotted σ traces + gvR ticks).
  bool m_sigmaAxis = false;
  float m_sigmaMax = 1.0f;
  std::vector<std::deque<DataPoint>> m_sigmaData;

  void pruneData();
};
