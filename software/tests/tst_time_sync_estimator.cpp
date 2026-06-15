#include <QtTest>

#include "TimeSyncEstimator.h"

// TimeSyncEstimator: NTP four-timestamp offset/delay, min-delay jitter
// rejection, skew tracking, and the fcToGcs() timeline projection.
// (docs/telemetry/time_sync.md)
class TstTimeSyncEstimator : public QObject {
  Q_OBJECT

private slots:
  void exactOffsetAndDelay();
  void syncedAfterThree();
  void minDelaySelection();
  void skewRecoveryAndProjection();
  void rejectsImplausibleDelay();
  void resetClears();
};

// Build a sample with a chosen FC-GCS offset O and round-trip delay D
// (symmetric link, zero FC processing). Inverts the estimator's own math:
//   offset = (t2 - t1) - D/2,  delay = D,  so t2 = t1 + O + D/2.
static TimeSyncEstimator::Sample make(qint64 t1, double offset, double delay) {
  TimeSyncEstimator::Sample s;
  s.t1 = t1;
  s.t2 = qint64(t1 + offset + delay / 2.0);
  s.t3 = s.t2;
  s.t4 = qint64(t1 + delay);
  return s;
}

void TstTimeSyncEstimator::exactOffsetAndDelay() {
  TimeSyncEstimator e;
  // Textbook vector: offset 1045 ms, round-trip 10 ms (one-way 5 ms).
  QVERIFY(e.addSample(make(1000, 1045.0, 10.0)));
  QCOMPARE(e.offsetMs(), qint64(1045));
  QCOMPARE(e.delayMs(), qint64(5));
}

void TstTimeSyncEstimator::syncedAfterThree() {
  TimeSyncEstimator e;
  QVERIFY(!e.synced());
  e.addSample(make(1000, 100.0, 8.0));
  QVERIFY(!e.synced());
  e.addSample(make(2000, 100.0, 8.0));
  QVERIFY(!e.synced());
  e.addSample(make(3000, 100.0, 8.0));
  QVERIFY(e.synced());
}

void TstTimeSyncEstimator::minDelaySelection() {
  TimeSyncEstimator e;
  // Same instant (no skew): one clean low-latency sample at offset 1000, three
  // heavily-jittered ones at offset 1100. The 1/(delay)^2 weighting must let
  // the clean sample dominate the filtered offset.
  e.addSample(make(50000, 1100.0, 200.0));
  e.addSample(make(50000, 1000.0, 4.0));  // the good one
  e.addSample(make(50000, 1100.0, 220.0));
  e.addSample(make(50000, 1100.0, 180.0));
  QVERIFY(std::llabs(e.offsetMs() - 1000) < 5);
  QCOMPARE(e.delayMs(), qint64(2));  // min round-trip 4 -> one-way 2
}

void TstTimeSyncEstimator::skewRecoveryAndProjection() {
  TimeSyncEstimator e;
  // FC clock runs 1000 ppm fast: offset(t1) = O0 + skew*(t1 - gcs0).
  const double gcs0 = 100000, O0 = 5000, skew = 0.001 /*=1000 ppm*/, d = 10;
  qint64 lastT1 = 0;
  for (int k = 0; k <= 10; ++k) {
    const qint64 t1 = qint64(gcs0) + qint64(k) * 1000;
    const double offset = O0 + skew * (double(t1) - gcs0);
    QVERIFY(e.addSample(make(t1, offset, 2 * d)));  // round-trip 2d
    lastT1 = t1;
  }
  QVERIFY(e.synced());
  // Slope recovered to within a ppm of the truth.
  QVERIFY(std::fabs(e.skewPpm() - 1000.0) < 5.0);

  // An FC reading taken at GCS instant (lastT1 + d) maps back to that instant.
  const double offAtLast = O0 + skew * (double(lastT1 + qint64(d)) - gcs0);
  const qint64 fcReading = qint64(lastT1 + d + offAtLast);
  const qint64 gcs = e.fcToGcs(fcReading, lastT1 + qint64(d));
  QVERIFY(std::llabs(gcs - (lastT1 + qint64(d))) <= 2);
}

void TstTimeSyncEstimator::rejectsImplausibleDelay() {
  TimeSyncEstimator e;
  // Negative round-trip (t4 before t1): rejected, not counted.
  TimeSyncEstimator::Sample neg;
  neg.t1 = 1000;
  neg.t4 = 990;
  neg.t2 = neg.t3 = 1500;
  QVERIFY(!e.addSample(neg));

  // Absurdly large round-trip beyond the cap: rejected.
  QVERIFY(!e.addSample(make(2000, 100.0, double(TimeSyncEstimator::kMaxDelayMs) + 1)));

  QVERIFY(!e.synced());
  // A plausible one still lands.
  QVERIFY(e.addSample(make(3000, 100.0, 8.0)));
}

void TstTimeSyncEstimator::resetClears() {
  TimeSyncEstimator e;
  for (int k = 0; k < 4; ++k)
    e.addSample(make(1000 + k * 1000, 42.0, 6.0));
  QVERIFY(e.synced());
  e.reset();
  QVERIFY(!e.synced());
  QCOMPARE(e.offsetMs(), qint64(0));
  QCOMPARE(e.fcToGcs(12345, 99999), qint64(12345));  // identity again
}

QTEST_APPLESS_MAIN(TstTimeSyncEstimator)
#include "tst_time_sync_estimator.moc"
