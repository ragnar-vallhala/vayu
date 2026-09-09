#pragma once

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <utility>
#include <vector>

// Derivative-free / estimated-gradient optimizers for in-GCS PID autotuning —
// a faithful C++ port of tools/autotune/optimizers.py, so the GCS runs the
// search natively with no python3 subprocess. Each optimizer minimises a noisy
// black-box cost f(x) over box bounds, sharing one Evaluator (global budget,
// running best, per-eval callback for the live current/best UI). Pure: no Qt,
// no sim — unit-tested against known cost bowls.
namespace autotune {

using Vec = std::vector<double>;
using Bounds = std::vector<std::pair<double, double>>; // (lo, hi) per dim
using CostFn = std::function<double(const Vec &)>;

// Deterministic seeded RNG (mirrors Python random.Random's role, not its exact
// stream — only determinism per seed is relied upon).
class Rng {
public:
  explicit Rng(uint64_t seed) : m_gen(seed) {}
  double random() { return m_unit(m_gen); } // [0, 1)
  double uniform(double lo, double hi) { return lo + (hi - lo) * random(); }
  bool coin() { return random() < 0.5; }

private:
  std::mt19937_64 m_gen;
  std::uniform_real_distribution<double> m_unit{0.0, 1.0};
};

// Thrown to unwind an optimizer once the global evaluation budget is spent.
struct BudgetExhausted {};

// Wraps the cost fn: counts evals, tracks the running best, records history,
// and reports each eval (onEval) for the live convergence chart + current/best
// gains table.
class Evaluator {
public:
  Evaluator(CostFn fn, int budget) : m_fn(std::move(fn)), m_budget(budget) {}

  // Evaluate x. Throws BudgetExhausted if the budget is already spent.
  double eval(const Vec &x);

  int count() const { return m_n; }
  int budget() const { return m_budget; }
  int left() const { return m_budget - m_n; }
  bool hasBest() const { return m_hasBest; }
  double best() const { return m_best; }
  const Vec &bestX() const { return m_bestX; }

  struct Hist {
    int n;
    double cost;
    double best;
  };
  const std::vector<Hist> &history() const { return m_history; }

  // (current x, best x, cost, best cost, eval index). Fires after each eval.
  std::function<void(const Vec &, const Vec &, double, double, int)> onEval;

private:
  CostFn m_fn;
  int m_budget;
  int m_n = 0;
  double m_best = std::numeric_limits<double>::infinity();
  Vec m_bestX;
  bool m_hasBest = false;
  std::vector<Hist> m_history;
};

// Run one optimizer by registry name, seeded from x0 over bounds. An unknown
// name falls back to random search. Catches BudgetExhausted internally, so the
// caller just reads ev.best()/ev.bestX() afterward.
void run(const std::string &name, Evaluator &ev, const Vec &x0,
         const Bounds &bounds, Rng &rng);

// Registry names (mirror optimizers.py REGISTRY).
std::vector<std::string> optimizerNames();

} // namespace autotune
