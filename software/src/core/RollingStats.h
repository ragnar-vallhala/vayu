#pragma once

#include <QQueue>
#include <cmath>

/**
 * Helper class to calculate rolling mean and standard deviation.
 * Uses a sliding window and maintains running sums for O(1) performance.
 */
class RollingStats {
public:
  RollingStats(int windowSize = 50) : m_size(windowSize) {}

  void push(float val) {
    m_window.enqueue(val);
    m_sum += val;
    m_sumSq += (val * val);

    if (m_window.size() > m_size) {
      float old = m_window.dequeue();
      m_sum -= old;
      m_sumSq -= (old * old);
    }
  }

  float mean() const {
    if (m_window.isEmpty())
      return 0.0f;
    return m_sum / m_window.size();
  }

  float stdDev() const {
    if (m_window.size() < 2)
      return 0.0f;
    float n = static_cast<float>(m_window.size());
    float variance = (m_sumSq - (m_sum * m_sum) / n) / (n - 1.0f);
    return std::sqrt(std::max(0.0f, variance));
  }

  void reset() {
    m_window.clear();
    m_sum = 0.0f;
    m_sumSq = 0.0f;
  }

private:
  int m_size;
  QQueue<float> m_window;
  double m_sum = 0.0;
  double m_sumSq = 0.0;
};
