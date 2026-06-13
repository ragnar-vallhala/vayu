#include "AutotuneWorker.h"

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
        return autotune::runRollout(stack, names, x, m_p.tuneYaw, m_p.rollout);
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
