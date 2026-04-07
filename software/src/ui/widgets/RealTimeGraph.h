#pragma once

#include <QColor>
#include <QDateTime>
#include <QWidget>
#include <deque>
#include <vector>

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
  void appendData(float value, int index = 0);
  int numSeries() const { return static_cast<int>(m_seriesData.size()); }
  void clear();
  void setDynamicYAxis(bool enabled);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  struct DataPoint {
    qint64 timestamp;
    float value;
  };

  std::vector<std::deque<DataPoint>> m_seriesData;
  Mode m_mode = Mode::LinePlot;
  int m_windowSeconds = 5;
  double m_dropoutRate = 0.0;
  bool m_dynamicYAxis = false;
  std::vector<QColor> m_colors;
  std::vector<Qt::PenStyle> m_penStyles;

  float m_min = -1.0f;
  float m_max = 1.0f;

  void pruneData();
};
