#pragma once

#include <atomic>
#include <mutex>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include "AutotuneEngine.h"
#include "Rollout.h"
#include "vsim_proto.h" // vsim_ctl_geometry_t (vehicle geometry blob)

// Bundles the C++ autotune run for the GUI: owns an AutotuneEngine and drives
// the search on the in-process real-vaios backend (`vayu_sitl_rtos`, RtosEval).
// run() (invoked in a worker thread) flies the seeded doublet/chirp rollouts,
// re-emitting the engine's per-eval and final signals (queued to the GUI) plus
// log/failed — replacing the python3 autotune subprocess. cancel() is
// thread-safe.
class AutotuneWorker : public QObject {
  Q_OBJECT

public:
  struct Params {
    // VSIM_FIFO_SUFFIX isolation tag so a concurrent world-tab/other sim can't
    // collide on the backend's /tmp advert paths.
    QString suffix;
    // Tune the loaded airframe (physics + firmware mix) instead of the
    // reference quad: the physics-frame geometry the backend applies via
    // VAYU_RTOS_GEOMETRY. Leave hasGeometry false to tune the reference quad.
    bool hasGeometry = false;
    vsim_ctl_geometry_t geometry{};
    bool tuneYaw = false;
    QString optimizer = "structured";
    int budget = 30;
    int repeats = 5; // rollouts averaged per eval (distinct noise seeds)
    quint64 optSeed = 1;
    autotune::RolloutParams rollout;
    QString rtosBin; // path to vayu_sitl_rtos (required)
    // Fast-backend cost: false = angle tracking (the exact realtime cost);
    // true = angle + roll/pitch rate-loop tracking (values the inner loop too).
    bool rtosRateCost = false;
    // System-ID engine: instead of an optimizer search, fly ONE chirp on the
    // fast backend, fit a plant model per axis, and compute the gains
    // analytically (loop-shaping). Sidesteps the degenerate "don't-move" cost
    // minimum entirely. Implies the fast backend (uses rtosBin). See SysId.h.
    bool sysId = false;
    double sysIdBwFrac = 0.33; // crossover as a fraction of the actuator BW
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
  void done(); // run() has fully returned (success or failure) — safe to quit

private:
  // Optimizer search: drive the deterministic in-process vayu_sitl_rtos backend
  // (RtosEval) over the optimizer, scoring each candidate's doublet with the
  // realtime Cost.cpp metric. The default path.
  void runRtos();

  // System-ID path: fly one chirp on the backend, fit a per-axis plant (SysId),
  // and design the gains analytically — no optimizer loop. Selected by
  // Params::sysId. Emits the same finished()/log() so the UI applies the result.
  void runSysId();

  Params m_p;
  std::atomic<bool> m_cancel{false};
  std::mutex m_engMtx;
  AutotuneEngine *m_engine = nullptr; // valid only during run(), for cancel()
};
