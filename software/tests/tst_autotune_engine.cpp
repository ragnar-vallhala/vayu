#include <QSignalSpy>
#include <QtTest>

#include "AutotuneEngine.h"
#include "Space.h"

// AutotuneEngine orchestration (no sim): with an injected synthetic rollout it
// must run the optimizer, stream evaluated() each step, and finish with the
// best gains. Also checks the Space param layout the engine exposes.
class TstAutotuneEngine : public QObject {
  Q_OBJECT

private slots:
  void spaceLayout();
  void runsStreamsAndFinishes();
  void yawWidensTheSpace();
};

void TstAutotuneEngine::spaceLayout() {
  autotune::Space s(false);
  QCOMPARE(s.dim(), 5);
  QCOMPARE(QString::fromStdString(s.names().front()), QString("rate_kp"));
  QCOMPARE(QString::fromStdString(s.names().back()), QString("gyro_lpf"));
  const auto b = s.bounds();
  QCOMPARE(b.front().first, 0.0005);  // rate_kp lo
  QCOMPARE(b.front().second, 0.012);  // rate_kp hi
  QCOMPARE(s.seed().front(), 0.007);

  autotune::Space y(true);
  QCOMPARE(y.dim(), 9);
  QCOMPARE(QString::fromStdString(y.names().back()), QString("yaw_gyro_lpf"));
}

void TstAutotuneEngine::runsStreamsAndFinishes() {
  // Synthetic rollout: a bowl centred on a point inside the bounds. coordinate
  // descent should drive the engine's best near it.
  const autotune::Space space(false);
  const autotune::Vec target = {0.006, 0.004, 0.0004, 1.5, 0.004};
  auto rollout = [&](const QVector<double> &x) -> std::optional<double> {
    double s = 0;
    for (int i = 0; i < x.size(); ++i)
      s += (x[i] - target[i]) * (x[i] - target[i]);
    return s;
  };

  AutotuneEngine eng(false, "coordinate", 300, 7, rollout);
  QSignalSpy evalSpy(&eng, &AutotuneEngine::evaluated);
  QSignalSpy finSpy(&eng, &AutotuneEngine::finished);
  eng.run();

  QVERIFY(evalSpy.count() > 0);
  QVERIFY(evalSpy.count() <= 300);  // never exceeds budget
  QCOMPARE(finSpy.count(), 1);

  const QList<QVariant> fin = finSpy.first();
  const auto bestX = fin.at(0).value<QVector<double>>();
  const auto names = fin.at(1).toStringList();
  const double bestCost = fin.at(2).toDouble();
  QCOMPARE(bestX.size(), 5);
  QCOMPARE(names.size(), 5);
  QVERIFY(bestCost < 0.001);  // converged near the synthetic optimum
}

void TstAutotuneEngine::yawWidensTheSpace() {
  AutotuneEngine eng(
      true, "random", 10, 1,
      [](const QVector<double> &) -> std::optional<double> { return 1.0; });
  QCOMPARE(eng.paramNames().size(), 9);
}

QTEST_MAIN(TstAutotuneEngine)
#include "tst_autotune_engine.moc"
