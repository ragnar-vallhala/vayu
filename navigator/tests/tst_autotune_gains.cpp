#include <QtTest>

#include "AutotuneGains.h"

// AT-1 apply mapping: an autotuner result (param names + best_x) must expand to
// the same CMD_SET_PID applies apply_gains() does — rate + angle-P for roll &
// pitch, plus the yaw rate loop when present.
class TstAutotuneGains : public QObject {
  Q_OBJECT

private slots:
  void rollPitchRateAndAngle();
  void yawAddsRateLoop();
  void missingAngleParamSkipsAngleApplies();
};

void TstAutotuneGains::rollPitchRateAndAngle() {
  const QStringList names = {"rate_kp", "rate_ki", "rate_kd", "angle_kp",
                             "gyro_lpf"};
  const QVector<double> x = {0.05, 0.002, 0.0005, 2.0, 0.0};
  const auto c = autotuneGainsToCommands(names, x);
  QCOMPARE(c.size(), 4); // roll{rate,angle}, pitch{rate,angle}

  QCOMPARE(c[0].controller, 1); // roll rate
  QCOMPARE(c[0].axis, 0);
  QCOMPARE(c[0].kp, 0.05f);
  QCOMPARE(c[0].ki, 0.002f);
  QCOMPARE(c[0].kd, 0.0005f);
  QCOMPARE(c[1].controller, 0); // roll angle-P
  QCOMPARE(c[1].axis, 0);
  QCOMPARE(c[1].kp, 2.0f);
  QCOMPARE(c[1].ki, 0.0f);
  QCOMPARE(c[2].controller, 1); // pitch rate
  QCOMPARE(c[2].axis, 1);
  QCOMPARE(c[3].controller, 0); // pitch angle-P
  QCOMPARE(c[3].axis, 1);
}

void TstAutotuneGains::yawAddsRateLoop() {
  const QStringList names = {"rate_kp",     "rate_ki",     "rate_kd",
                             "angle_kp",    "gyro_lpf",    "yaw_rate_kp",
                             "yaw_rate_ki", "yaw_rate_kd", "yaw_gyro_lpf"};
  const QVector<double> x = {0.05, 0.002, 0.0005, 2.0, 0.0,
                             0.01, 0.003, 0.0004, 0.0};
  const auto c = autotuneGainsToCommands(names, x);
  QCOMPARE(c.size(), 5);
  const PidSetCmd yaw = c.last();
  QCOMPARE(yaw.controller, 1);
  QCOMPARE(yaw.axis, 2);
  QCOMPARE(yaw.kp, 0.01f);
  QCOMPARE(yaw.ki, 0.003f);
  QCOMPARE(yaw.kd, 0.0004f);
}

void TstAutotuneGains::missingAngleParamSkipsAngleApplies() {
  const QStringList names = {"rate_kp", "rate_ki", "rate_kd"};
  const QVector<double> x = {0.05, 0.002, 0.0005};
  const auto c = autotuneGainsToCommands(names, x);
  QCOMPARE(c.size(), 2); // only roll/pitch rate, no angle loop
  QCOMPARE(c[0].controller, 1);
  QCOMPARE(c[1].controller, 1);
}

QTEST_APPLESS_MAIN(TstAutotuneGains)
#include "tst_autotune_gains.moc"
