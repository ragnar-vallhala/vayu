#pragma once

#include <cmath>
#include <vector>

/**
 * Helper class to calculate rolling mean and standard deviation.
 * Uses a fixed-size circular buffer (ring buffer) for O(1) push with
 * zero heap allocations after construction.
 */
class RollingStats {
public:
  explicit RollingStats(int windowSize = 50)
      : m_size(windowSize), m_buf(windowSize, 0.0f) {}

  void push(float val) {
    if (m_count == m_size) {
      // Window is full: evict the oldest value sitting at m_head
      float old = m_buf[m_head];
      m_sum -= old;
      m_sumSq -= (old * old);
    } else {
      ++m_count;
    }

    m_buf[m_head] = val;
    m_head = (m_head + 1) % m_size;

    m_sum += val;
    m_sumSq += (val * val);
  }

  float mean() const {
    if (m_count == 0)
      return 0.0f;
    return static_cast<float>(m_sum / m_count);
  }

  float stdDev() const {
    if (m_count < 2)
      return 0.0f;
    float n = static_cast<float>(m_count);
    float variance =
        static_cast<float>((m_sumSq - (m_sum * m_sum) / n) / (n - 1.0f));
    return std::sqrt(std::max(0.0f, variance));
  }

  void reset() {
    std::fill(m_buf.begin(), m_buf.end(), 0.0f);
    m_head = 0;
    m_count = 0;
    m_sum = 0.0;
    m_sumSq = 0.0;
  }

  int count() const { return m_count; }
  int capacity() const { return m_size; }

private:
  int m_size;
  std::vector<float> m_buf; // pre-allocated ring buffer
  int m_head = 0;  // index of next write position (oldest slot when full)
  int m_count = 0; // number of valid samples (≤ m_size)
  double m_sum = 0.0;
  double m_sumSq = 0.0;
};
