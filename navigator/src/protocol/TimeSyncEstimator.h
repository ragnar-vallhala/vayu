#pragma once

#include <QtGlobal>

#include <deque>

// Estimates the FC <-> GCS clock relationship from NTP-style 4-timestamp
// samples (see docs/telemetry/time_sync.md). Pure + Qt-free (only fixed-width
// ints) so it is unit-testable headless and cheap to run on the worker thread.
//
// Per sample (all absolute ms; t1 = GCS send, t2 = FC receive, t3 = FC send,
// t4 = GCS receive):
//   offset = ((t2 - t1) + (t3 - t4)) / 2   // FC clock - GCS clock
//   delay  = (t4 - t1) - (t3 - t2)         // round-trip transit (>= 0)
//
// WiFi jitter inflates delay asymmetrically, so each accepted sample is weighted
// by 1/(delay + d0)^2 in a least-squares fit of offset against GCS time:
// min-delay samples dominate (classic SNTP best-sample selection) while the fit
// slope recovers clock skew, letting fcToGcs() project the offset between syncs.
class TimeSyncEstimator {
public:
  struct Sample {
    qint64 t1 = 0; // GCS send    (wall-clock ms)
    qint64 t2 = 0; // FC receive  (FC ms)
    qint64 t3 = 0; // FC send     (FC ms)
    qint64 t4 = 0; // GCS receive (wall-clock ms)
  };

  // Feed one exchange. Returns false (and ignores the sample) when the
  // round-trip delay is implausible (negative, or beyond kMaxDelayMs).
  bool addSample(const Sample &s);

  // True once at least kMinSamples plausible samples have been accumulated.
  bool synced() const { return m_count >= kMinSamples; }

  // Filtered FC - GCS offset (ms) at the most recent sample's GCS time.
  qint64 offsetMs() const;
  // Best-case one-way latency (ms): the minimum round-trip in the window / 2.
  qint64 delayMs() const;
  // Tracked clock-rate drift of the FC clock vs the GCS clock, in ppm.
  double skewPpm() const { return m_b * 1e6; }

  // Map an FC timestamp (ms) onto the GCS timeline, with the offset projected
  // (via skew) to gcsNowMs. With no samples yet this is the identity.
  qint64 fcToGcs(qint64 fcMs, qint64 gcsNowMs) const;

  void reset();

  static constexpr int kWindow = 16;           // sliding-window depth
  static constexpr int kMinSamples = 3;        // samples before "synced"
  static constexpr qint64 kMaxDelayMs = 60000; // reject absurd round-trips

private:
  struct Pt {
    double t;      // GCS send time (t1), the regression abscissa
    double offset; // FC - GCS for this sample
    double delay;  // round-trip transit for this sample
  };

  std::deque<Pt> m_win;
  int m_count = 0;

  // Weighted-regression result, recomputed on every accepted sample:
  // offset(t) ~= m_a + m_b * (t - m_tRef).
  double m_a = 0;        // offset at m_tRef
  double m_b = 0;        // slope = skew (ms per ms)
  double m_tRef = 0;     // reference time (latest t1)
  double m_minDelay = 0; // smallest round-trip in the window

  void refit();
};
