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
  };

  explicit AutotuneWorker(Params p, QObject *parent = nullptr);

public slots:
  void run();
  void cancel();

signals:
  void evaluated(QVector<double> current, QVector<double> best, double cost,
                 double bestCost, int n);
  void finished(QVector<double> bestX, QStringList names, double bestCost);
  void failed(QString err);
  void log(QString line);
  void done();  // run() has fully returned (success or failure) — safe to quit

private:
  Params m_p;
  std::atomic<bool> m_cancel{false};
  std::mutex m_engMtx;
  AutotuneEngine *m_engine = nullptr;  // valid only during run(), for cancel()
};
