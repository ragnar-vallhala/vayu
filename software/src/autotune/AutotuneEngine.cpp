#include "AutotuneEngine.h"

#include "Cost.h"       // kBig
#include "Optimizer.h"

using namespace autotune;

namespace {
QVector<double> toQv(const Vec &v) {
  QVector<double> q;
  q.reserve(int(v.size()));
  for (double x : v)
    q.push_back(x);
  return q;
}
}  // namespace

AutotuneEngine::AutotuneEngine(bool tuneYaw, QString optimizer, int budget,
                               quint64 seed, Rollout rollout, QObject *parent)
    : QObject(parent), m_space(tuneYaw), m_optimizer(std::move(optimizer)),
      m_budget(budget), m_seed(seed), m_rollout(std::move(rollout)) {
  // Registered so evaluated()/finished() can cross a thread boundary (the UI
  // runs the engine in a worker thread).
  qRegisterMetaType<QVector<double>>("QVector<double>");
}

QStringList AutotuneEngine::paramNames() const {
  QStringList n;
  for (const std::string &s : m_space.names())
    n << QString::fromStdString(s);
  return n;
}

void AutotuneEngine::run() {
  const QStringList names = paramNames();

  Evaluator ev(
      [this](const Vec &x) -> double {
        if (m_cancel)
          throw BudgetExhausted{};
        // nullopt (un-scorable rollout) -> divergence penalty, search moves on.
        return m_rollout(toQv(x)).value_or(kBig);
      },
      m_budget);

  ev.onEval = [this](const Vec &cur, const Vec &best, double cost,
                     double bestCost, int n) {
    emit evaluated(toQv(cur), toQv(best), cost, bestCost, n);
  };

  Rng rng(m_seed);
  // Qualify: an unqualified run() would recurse into this method.
  autotune::run(m_optimizer.toStdString(), ev, m_space.seed(), m_space.bounds(),
                rng);

  emit finished(ev.hasBest() ? toQv(ev.bestX()) : QVector<double>(), names,
                ev.hasBest() ? ev.best() : kBig);
}
