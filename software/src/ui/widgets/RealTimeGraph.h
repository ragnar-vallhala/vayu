#pragma once

#include <QColor>
#include <QDateTime>
#include <QWidget>
#include <deque>

class RealTimeGraph : public QWidget {
  Q_OBJECT

public:
  enum class Mode { LinePlot, HorizontalBar };

  explicit RealTimeGraph(QWidget *parent = nullptr);

  void setMode(Mode mode);
  void setWindowSeconds(int seconds);
  void setDropoutRate(double rate);
  void setColor(const QColor &color);
  void appendData(float value);
  void clear();

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  struct DataPoint {
    qint64 timestamp;
    float value;
  };

  std::deque<DataPoint> m_data;
  Mode m_mode = Mode::LinePlot;
  int m_windowSeconds = 5;
  double m_dropoutRate = 0.0;
  QColor m_color = QColor("#61AFEF");

  float m_min = -1.0f;
  float m_max = 1.0f;

  void pruneData();
};
