#pragma once

#include <atomic>
#include <mutex>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include "AutotuneEngine.h"
#include "Rollout.h"
#include "SitlStack.h"

// Bundles the C++ autotune run for the GUI: owns a SitlStack + AutotuneEngine,
// and run() (invoked in a worker thread) brings up the SITL, runs the optimizer
// over a SitlStack-backed rollout, then tears the SITL down — replacing the
// python3 autotune subprocess. Re-emits the engine's per-eval and final signals
// (queued to the GUI) plus log/failed. cancel() is thread-safe.
class AutotuneWorker : public QObject {
  Q_OBJECT

public:
  struct Params {
    SitlStack::Config sitl;
    bool tuneYaw = false;
    QString optimizer = "structured";
    int budget = 30;
    int repeats = 5;  // rollouts averaged per eval (distinct noise seeds)
    quint64 optSeed = 1;
    autotune::RolloutParams rollout;
    // Fast backend: when set, run the search on the in-process real-vaios SITL
    // (`vayu_sitl_rtos`, ~70x realtime, deterministic) instead of the realtime
    // vsim_d+vayu_sitl FIFO stack — same doublet, same vehicle geometry, and the
    // same Cost.cpp scoring (RtosEval), just much faster. The SitlStack is not
    // spawned; the world-tab sim is unaffected and stays realtime.
    bool fastRtos = false;
    QString rtosBin;  // path to vayu_sitl_rtos (required when fastRtos)
    // Fast-backend cost: false = angle tracking (the exact realtime cost);
    // true = angle + roll/pitch rate-loop tracking (values the inner loop too).
    bool rtosRateCost = false;
  };

  explicit AutotuneWorker(Params p, QObject *parent = nullptr);

public slots:
  void run();
  void cancel();

signals:
  void evaluated(QVector<double> current, QVector<double> best, double cost,
                 double bestCost, int n);
  // Roll-axis excitation window of the latest eval (setpoint vs measured angle,
  // deg) for the live response plot.
  void responseWindow(QVector<double> sp, QVector<double> measured);
  void finished(QVector<double> bestX, QStringList names, double bestCost);
  void failed(QString err);
  void log(QString line);
  void done();  // run() has fully returned (success or failure) — safe to quit

private:
  // Fast path: drive the search on the deterministic in-process vayu_sitl_rtos
  // backend (RtosEval) instead of the realtime SitlStack. Selected by
  // Params::fastRtos. Shares the optimizer/engine; only the evaluate differs
  // (rate-tracking corr from a doublet vs the SITL telemetry IAE cost).
  void runRtos();

  Params m_p;
  std::atomic<bool> m_cancel{false};
  std::mutex m_engMtx;
  AutotuneEngine *m_engine = nullptr;  // valid only during run(), for cancel()
};
