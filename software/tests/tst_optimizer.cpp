#include <QtTest>
#include <cmath>

#include "Optimizer.h"

using namespace autotune;

// C++ autotune optimizer port: the Evaluator's budget/best bookkeeping and that
// each optimizer actually minimises a noisy-free quadratic bowl well under
// budget (the search itself, independent of the sim).
class TstOptimizer : public QObject {
  Q_OBJECT

  // f(x) = sum (x_i - t_i)^2 over a fixed target inside the bounds.
  static double bowl(const Vec &x, const Vec &t) {
    double s = 0;
    for (size_t i = 0; i < x.size(); ++i)
      s += (x[i] - t[i]) * (x[i] - t[i]);
    return s;
  }

private slots:
  void budgetIsEnforced();
  void tracksRunningBest();
  void onEvalFiresEachEval();
  void registryHasEight();
  void everyOptimizerImprovesOnSeed();
  void strongOptimizersConvergeNearTarget();
};

void TstOptimizer::budgetIsEnforced() {
  Evaluator ev([](const Vec &) { return 1.0; }, 3);
  ev.eval({0});
  ev.eval({0});
  ev.eval({0});
  QCOMPARE(ev.count(), 3);
  QCOMPARE(ev.left(), 0);
  bool threw = false;
  try {
    ev.eval({0});
  } catch (const BudgetExhausted &) {
    threw = true;
  }
  QVERIFY(threw);
}

void TstOptimizer::tracksRunningBest() {
  Evaluator ev([](const Vec &x) { return x[0]; }, 10);
  ev.eval({5.0});
  ev.eval({2.0});
  ev.eval({9.0});
  QVERIFY(ev.hasBest());
  QCOMPARE(ev.best(), 2.0);
  QCOMPARE(ev.bestX()[0], 2.0);
  QCOMPARE(int(ev.history().size()), 3);
  QCOMPARE(ev.history().back().best, 2.0);
}

void TstOptimizer::onEvalFiresEachEval() {
  int fired = 0;
  Evaluator ev([](const Vec &) { return 0.0; }, 5);
  ev.onEval = [&](const Vec &, const Vec &, double, double, int) { ++fired; };
  for (int i = 0; i < 5; ++i)
    ev.eval({double(i)});
  QCOMPARE(fired, 5);
}

void TstOptimizer::registryHasEight() {
  QCOMPARE(int(optimizerNames().size()), 8);
}

void TstOptimizer::everyOptimizerImprovesOnSeed() {
  const Vec target = {1.0, -2.0, 0.5};
  const Bounds bounds = {{-5, 5}, {-5, 5}, {-5, 5}};
  const Vec x0 = {-5, -5, -5};  // a corner, far from the target
  const double seedCost = bowl(x0, target);

  for (const auto &name : optimizerNames()) {
    Evaluator ev([&](const Vec &x) { return bowl(x, target); }, 400);
    Rng rng(12345);
    run(name, ev, x0, bounds, rng);
    QVERIFY2(ev.hasBest(), name.c_str());
    QVERIFY2(ev.best() < seedCost,
             (name + ": best " + std::to_string(ev.best()) + " !< seed " +
              std::to_string(seedCost))
                 .c_str());
  }
}

void TstOptimizer::strongOptimizersConvergeNearTarget() {
  // The reliable descent methods should land essentially on the bowl's
  // minimum; this proves the framework finds the optimum, not just improves.
  // (SPSA/fdgd are noisy-gradient methods — covered by improvesOnSeed.)
  const Vec target = {1.0, -2.0, 0.5};
  const Bounds bounds = {{-5, 5}, {-5, 5}, {-5, 5}};
  for (const char *name : {"coordinate", "nelder-mead"}) {
    Evaluator ev([&](const Vec &x) { return bowl(x, target); }, 600);
    Rng rng(7);
    run(name, ev, {-5, -5, -5}, bounds, rng);
    QVERIFY2(ev.best() < 1.0, name);
  }
}

QTEST_APPLESS_MAIN(TstOptimizer)
#include "tst_optimizer.moc"
