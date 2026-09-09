#include <QtTest>
#include <cmath>

#include "Cost.h"

using namespace autotune;

// Pure autotune cost scoring (port of autotune.py): tracking + overshoot +
// chatter, the too-few-samples (NoData) and divergence cases, and yaw rate.
class TstCost : public QObject {
  Q_OBJECT

  static Sample lvl(double sp, double cur, double out) {
    Sample s; // roll axis carries the trace; others left at 0
    s.rollAngleSp = sp;
    s.rollAngleCurr = cur;
    s.rollOut = out;
    return s;
  }

private slots:
  void chatterIsMeanAbsSwing();
  void tooFewSamplesIsNoData();
  void perfectTrackingIsCheap();
  void divergenceIsBig();
  void chatterRaisesCost();
  void yawRateScores();
};

void TstCost::chatterIsMeanAbsSwing() {
  QCOMPARE(chatter({}), 0.0);
  QCOMPARE(chatter({5.0}), 0.0);
  // swings: |2-1|+|1-2| = 2, /3 samples
  QVERIFY(std::fabs(chatter({1.0, 2.0, 1.0}) - (2.0 / 3.0)) < 1e-12);
}

void TstCost::tooFewSamplesIsNoData() {
  std::vector<Sample> s(4, lvl(0, 0, 0)); // < 5
  QVERIFY(!axisCost(s, 0).has_value());
}

void TstCost::perfectTrackingIsCheap() {
  // Measured == setpoint, output dead steady -> near-zero cost.
  std::vector<Sample> s;
  for (int i = 0; i < 20; ++i)
    s.push_back(lvl(10.0, 10.0, 0.5));
  auto c = axisCost(s, 0);
  QVERIFY(c.has_value() && *c < 0.01);
}

void TstCost::divergenceIsBig() {
  std::vector<Sample> s;
  for (int i = 0; i < 10; ++i)
    s.push_back(lvl(0.0, 95.0, 0.0)); // |angle| > 80
  auto c = axisCost(s, 0);
  QVERIFY(c.has_value());
  QCOMPARE(c.value_or(0.0), kBig);
}

void TstCost::chatterRaisesCost() {
  // Same tracking, but a buzzing output: cost must be strictly higher.
  std::vector<Sample> calm, buzz;
  for (int i = 0; i < 20; ++i) {
    calm.push_back(lvl(5.0, 5.0, 0.0));
    buzz.push_back(lvl(5.0, 5.0, (i % 2) ? 1.0 : -1.0)); // big swings
  }
  auto a = axisCost(calm, 0);
  auto b = axisCost(buzz, 0);
  QVERIFY(a && b && *b > *a);
}

void TstCost::yawRateScores() {
  std::vector<Sample> s;
  for (int i = 0; i < 20; ++i) {
    Sample k;
    k.yawRateSp = 100.0;
    k.yawRateCurr = 100.0; // perfect rate tracking
    k.yawOut = 0.2;
    s.push_back(k);
  }
  auto c = yawRateCost(s);
  QVERIFY(c.has_value() && *c < 0.01);

  // Saturation -> BIG.
  std::vector<Sample> sat(10);
  for (auto &k : sat)
    k.yawRateCurr = 2500.0;
  const auto satCost = yawRateCost(sat);
  QVERIFY(satCost.has_value());
  QCOMPARE(satCost.value_or(0.0), kBig);
}

QTEST_APPLESS_MAIN(TstCost)
#include "tst_cost.moc"
