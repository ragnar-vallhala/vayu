#include "TimeSyncEstimator.h"

#include <cmath>

bool TimeSyncEstimator::addSample(const Sample &s) {
  // NTP four-timestamp offset/delay. Work in doubles so the large absolute
  // wall-clock magnitudes (~1.7e12 ms) and the sub-ms slope both stay sane.
  const double offset = (double(s.t2 - s.t1) + double(s.t3 - s.t4)) / 2.0;
  const double delay = double(s.t4 - s.t1) - double(s.t3 - s.t2);

  // Reject impossible/absurd round-trips: a negative delay means a clock ran
  // backward or the sample is mismatched; an enormous one is a stalled link.
  if (delay < 0.0 || delay > double(kMaxDelayMs))
    return false;

  m_win.push_back({double(s.t1), offset, delay});
  while (m_win.size() > size_t(kWindow))
    m_win.pop_front();
  if (m_count < kMinSamples)
    ++m_count;

  refit();
  return true;
}

void TimeSyncEstimator::refit() {
  if (m_win.empty())
    return;

  // Reference the fit at the newest sample so offsetMs() reports "now" and the
  // intercept stays well-conditioned regardless of absolute time magnitude.
  m_tRef = m_win.back().t;
  m_minDelay = m_win.front().delay;
  for (const Pt &p : m_win)
    if (p.delay < m_minDelay)
      m_minDelay = p.delay;

  // Weighted least squares: weight ~ 1/(delay + d0)^2 so the lowest-latency
  // (least-jittered) samples dominate the offset while the slope still tracks
  // skew. d0 keeps the weight finite for a zero-delay (loopback/SITL) sample.
  constexpr double d0 = 1.0;
  double sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0;
  for (const Pt &p : m_win) {
    const double w = 1.0 / ((p.delay + d0) * (p.delay + d0));
    const double x = p.t - m_tRef;
    sw += w;
    swx += w * x;
    swy += w * p.offset;
    swxx += w * x * x;
    swxy += w * x * p.offset;
  }

  const double denom = sw * swxx - swx * swx;
  if (std::fabs(denom) < 1e-9) {
    // No usable time spread (single sample, or all at one instant): flat fit.
    m_b = 0.0;
    m_a = swy / sw;
  } else {
    m_b = (sw * swxy - swx * swy) / denom;
    m_a = (swy - m_b * swx) / sw;
  }
}

qint64 TimeSyncEstimator::offsetMs() const {
  return llround(m_a); // offset at m_tRef
}

qint64 TimeSyncEstimator::delayMs() const {
  return llround(m_minDelay / 2.0); // round-trip min -> one-way latency
}

qint64 TimeSyncEstimator::fcToGcs(qint64 fcMs, qint64 gcsNowMs) const {
  if (m_win.empty())
    return fcMs; // no estimate yet: identity
  const double offsetAt = m_a + m_b * (double(gcsNowMs) - m_tRef);
  return fcMs - llround(offsetAt); // GCS = FC - offset
}

void TimeSyncEstimator::reset() {
  m_win.clear();
  m_count = 0;
  m_a = m_b = m_tRef = m_minDelay = 0.0;
}
