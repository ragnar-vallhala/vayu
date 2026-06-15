#include "AutotuneWorker.h"

#include <algorithm>

#include "Optimizer.h"  // autotune::Vec
#include "Space.h"

AutotuneWorker::AutotuneWorker(Params p, QObject *parent)
    : QObject(parent), m_p(std::move(p)) {}

void AutotuneWorker::cancel() {
  m_cancel = true;
  std::lock_guard<std::mutex> lk(m_engMtx);
  if (m_engine)
    m_engine->cancel();
}

void AutotuneWorker::run() {
  SitlStack stack(m_p.sitl);
  emit log(QStringLiteral("launching SITL stack (vsim_d + vayu_sitl) ..."));
  QString err;
  if (!stack.start(&err)) {
    emit failed(err);
    emit done();
    return;
  }
  emit log(QStringLiteral("SITL up — tuning %1 params with '%2' (budget %3)")
               .arg(autotune::Space(m_p.tuneYaw).dim())
               .arg(m_p.optimizer)
               .arg(m_p.budget));

  const autotune::Space space(m_p.tuneYaw);
  const std::vector<std::string> names = space.names();

  AutotuneEngine engine(
      m_p.tuneYaw, m_p.optimizer, m_p.budget, m_p.optSeed,
      [&](const QVector<double> &xq) -> std::optional<double> {
        autotune::Vec x;
        x.reserve(xq.size());
        for (double v : xq)
          x.push_back(v);
        // Average `repeats` rollouts over distinct noise realizations
        // (sim_seed + i), mirroring autotune.py: include divergences (kBig) in
        // the mean, drop only harness failures (nullopt). This smooths the
        // noisy SITL cost so the search isn't misled by one unlucky rollout.
        double sum = 0.0;
        int scored = 0;
        const int reps = std::max(1, m_p.repeats);
        std::vector<autotune::Sample> resp;  // last rollout's roll-axis window
        for (int i = 0; i < reps && !m_cancel.load(); ++i) {
          autotune::RolloutParams rp = m_p.rollout;
          rp.seed = m_p.rollout.seed + quint32(i);
          // Capture the response window only on the final repeat (for the plot).
          std::vector<autotune::Sample> *out = (i == reps - 1) ? &resp : nullptr;
          // &m_cancel makes the rollout abort within ~10 ms of Stop.
          if (auto c = autotune::runRollout(stack, names, x, m_p.tuneYaw, rp, out,
                                            &m_cancel)) {
            sum += *c;
            ++scored;
          }
        }
        // Push the latest excitation window to the live response plot.
        if (!resp.empty()) {
          QVector<double> sp, meas;
          sp.reserve(int(resp.size()));
          meas.reserve(int(resp.size()));
          for (const autotune::Sample &s : resp) {
            sp.push_back(s.rollAngleSp);
            meas.push_back(s.rollAngleCurr);
          }
          emit responseWindow(sp, meas);
        }
        if (scored == 0)
          return std::nullopt;  // every attempt was a harness failure
        return sum / scored;
      });

  // Re-emit the engine's signals (this worker lives in the worker thread, so
  // they queue across to the GUI).
  connect(&engine, &AutotuneEngine::evaluated, this, &AutotuneWorker::evaluated);
  connect(&engine, &AutotuneEngine::finished, this, &AutotuneWorker::finished);

  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = &engine;
    if (m_cancel)
      engine.cancel();
  }
  engine.run();  // blocking: optimizer -> rollout -> stack
  {
    std::lock_guard<std::mutex> lk(m_engMtx);
    m_engine = nullptr;
  }

  stack.stop();
  emit log(QStringLiteral("SITL stack stopped."));
  emit done();
}
