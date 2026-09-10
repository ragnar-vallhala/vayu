#include <QtTest>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#include "Optimizer.h"
#include "Space.h"

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
  void allOptimizersConvergeOnRealSpace();
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
  const Vec x0 = {-5, -5, -5}; // a corner, far from the target
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

// The symmetric-bowl tests above use equal [-5,5] spans. The *real* autotune
// Space has wildly unequal per-gain spans (angle_kp's bound is ~2.95 wide;
// rate_kd's ~7e-4), which is the case that actually bites: a raw sum-of-squares
// cost is then dominated by the widest gain, so a "low" cost can't certify the
// narrow gains converged. This test runs EVERY registered optimizer on the real
// bounds with a span-NORMALIZED bowl (each gain weighted by 1/span — the
// suitable convergence metric) and holds each to a bound matching its mechanism:
// strong local minimizers land ~0, stochastic descent gets close, pure explorers
// only partially refine. Thresholds are empirical (budget 300, seeds 1/7/42)
// with ~2x margin; the map must cover the whole registry, so a newly added
// optimizer without a documented bound fails here on purpose.
void TstOptimizer::allOptimizersConvergeOnRealSpace() {
  const autotune::Space space(false);
  const Bounds bounds = space.bounds();
  const Vec seed = space.seed();
  const Vec target = {0.006, 0.004, 0.0004, 1.5, 0.004}; // inside the bounds
  Vec span(bounds.size());
  for (size_t i = 0; i < bounds.size(); ++i)
    span[i] = bounds[i].second - bounds[i].first;
  auto nbowl = [&](const Vec &x) {
    double s = 0;
    for (size_t i = 0; i < x.size(); ++i) {
      const double d = (x[i] - target[i]) / span[i];
      s += d * d;
    }
    return s;
  };
  const double seedCost = nbowl(seed); // ~0.208

  // name -> max normalized cost the optimizer must reach within budget 300.
  const std::map<std::string, double> bound = {
      {"nelder-mead", 0.005}, {"portfolio", 0.005}, {"coordinate", 0.005},
      {"spsa", 0.02},         {"hybrid", 0.02},     {"random", 0.15},
      {"structured", 0.13},   {"fdgd", 0.19},
  };

  for (const auto &name : optimizerNames()) {
    const auto it = bound.find(name);
    QVERIFY2(it != bound.end(),
             (name + ": no convergence bound in test — add one").c_str());

    // Worst over a few seeds: deterministic methods ignore the seed; stochastic
    // ones (random/spsa/hybrid) must hold the bound regardless of it.
    double worst = 0;
    for (uint64_t s : {uint64_t(1), uint64_t(7), uint64_t(42)}) {
      Evaluator ev([&](const Vec &x) { return nbowl(x); }, 300);
      Rng rng(s);
      run(name, ev, seed, bounds, rng);
      QVERIFY2(ev.hasBest(), name.c_str());
      worst = std::max(worst, ev.best());
    }
    QVERIFY2(worst < it->second,
             (name + ": worst normCost " + std::to_string(worst) +
              " !< bound " + std::to_string(it->second))
                 .c_str());
    QVERIFY2(worst < seedCost,
             (name + ": did not improve on seed " + std::to_string(seedCost))
                 .c_str());
  }
}

QTEST_APPLESS_MAIN(TstOptimizer)
#include "tst_optimizer.moc"
