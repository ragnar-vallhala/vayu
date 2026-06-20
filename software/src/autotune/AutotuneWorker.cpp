#include "AutotuneWorker.h"

#include <algorithm>
#include <cmath>

#include "Cost.h"       // autotune::kBig (divergence threshold)
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
        // State + POSE log so the operator (and offline diagnosis) can see WHAT
        // each eval did, not just the cost: the gains tried, the mean cost, and
        // the peak attitude the firmware reported during the roll-axis window
        // (rollAngleCurr/pitchAngleCurr are the estimated pose the controller
        // sees and what diverges; yawRateCurr is the yaw body rate). A cost
        // >= kBig means the craft tumbled past 80° on some axis.
        {
          double mxRoll = 0.0, mxPitch = 0.0, mxYawRate = 0.0;
          for (const autotune::Sample &s : resp) {
            mxRoll = std::max(mxRoll, std::fabs(s.rollAngleCurr));
            mxPitch = std::max(mxPitch, std::fabs(s.pitchAngleCurr));
            mxYawRate = std::max(mxYawRate, std::fabs(s.yawRateCurr));
          }
          QStringList gs;
          for (int gi = 0; gi < int(names.size()) && gi < int(x.size()); ++gi)
            gs << QStringLiteral("%1=%2")
                      .arg(QString::fromStdString(names[size_t(gi)]))
                      .arg(x[size_t(gi)], 0, 'g', 4);
          const bool diverged = (scored == 0) || (sum / std::max(1, scored)) >= autotune::kBig;
          const QString costStr =
              (scored == 0) ? QStringLiteral("n/a (harness fail)")
                            : QString::number(sum / scored, 'f', 2);
          emit log(QStringLiteral("  eval [%1]  cost=%2  pose: max|roll|=%3° "
                                  "max|pitch|=%4° peak|yawRate|=%5°/s%6")
                       .arg(gs.join(QStringLiteral(", ")))
                       .arg(costStr)
                       .arg(mxRoll, 0, 'f', 1)
                       .arg(mxPitch, 0, 'f', 1)
                       .arg(mxYawRate, 0, 'f', 1)
                       .arg(diverged ? QStringLiteral("  *** DIVERGED (>80°)")
                                     : QString()));
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
