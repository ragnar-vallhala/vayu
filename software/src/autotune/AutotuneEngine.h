#pragma once

#include <atomic>
#include <functional>
#include <optional>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include "Space.h"

// Runs a PID autotune search natively in the GCS (no python3): an optimizer
// from Optimizer.h drives a cost provided by an injected *rollout*
// (gains -> scalar cost). Emits the current + best gain vectors each
// evaluation (the live convergence chart + current-vs-best table, AT-2) and the
// final best. The rollout is injected, so the engine is unit-tested with a
// synthetic cost; SimulatorWidget supplies the real SITL rollout (apply gains,
// excite, score the telemetry window). run() is blocking — move the engine to a
// QThread for the UI so it doesn't stall the event loop.
class AutotuneEngine : public QObject {
  Q_OBJECT

public:
  // rollout(x) -> cost, or nullopt when the rollout couldn't score (treated as
  // a divergence penalty so the search moves on).
  using Rollout = std::function<std::optional<double>(const QVector<double> &)>;

  // fastRtos restricts the param space to the gains the in-process RTOS backend
  // can set (see Space). Trailing default so existing positional callers (and
  // the realtime SITL path) are unaffected.
  AutotuneEngine(bool tuneYaw, QString optimizer, int budget, quint64 seed,
                 Rollout rollout, bool fastRtos = false,
                 QObject *parent = nullptr);

  QStringList paramNames() const;

public slots:
  void run();
  void cancel() { m_cancel = true; }

signals:
  void evaluated(QVector<double> current, QVector<double> best, double cost,
                 double bestCost, int n);
  void finished(QVector<double> bestX, QStringList names, double bestCost);

private:
  autotune::Space m_space;
  QString m_optimizer;
  int m_budget;
  quint64 m_seed;
  Rollout m_rollout;
  std::atomic<bool> m_cancel{false};
};
