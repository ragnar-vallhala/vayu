#include "SourceController.h"

void SourceController::teardown(SourceState s) {
  switch (s) {
  case SourceState::Fc:
    if (m_hooks.teardownFc) m_hooks.teardownFc();
    break;
  case SourceState::Sim:
    if (m_hooks.teardownSim) m_hooks.teardownSim();
    break;
  case SourceState::Autotune:
    if (m_hooks.teardownAutotune) m_hooks.teardownAutotune();
    break;
  case SourceState::Replay:
    if (m_hooks.teardownReplay) m_hooks.teardownReplay();
    break;
  case SourceState::Idle:
    break;  // nothing to tear down
  }
}

bool SourceController::transition(SourceState to,
                                 const std::function<bool()> &setup) {
  // Strict single source: drop whatever is active before bringing up the new
  // one. teardown hooks are idempotent (safe even if that source isn't live).
  teardown(m_state);
  const bool ok = setup ? setup() : true;
  m_prev = m_state;
  m_state = ok ? to : SourceState::Idle;  // a failed setup lands in Idle
  if (m_hooks.applyFeed) m_hooks.applyFeed(m_state);
  emit stateChanged(m_state, m_prev);
  return ok;
}

bool SourceController::requestIdle() {
  return transition(SourceState::Idle, {});
}

bool SourceController::requestFc(const QString &port, int baud) {
  return transition(SourceState::Fc, [&] {
    return m_hooks.setupFc ? m_hooks.setupFc(port, baud) : true;
  });
}

bool SourceController::requestSim() {
  return transition(SourceState::Sim,
                    [&] { return m_hooks.setupSim ? m_hooks.setupSim() : true; });
}

bool SourceController::requestAutotune() {
  return transition(SourceState::Autotune, [&] {
    return m_hooks.setupAutotune ? m_hooks.setupAutotune() : true;
  });
}

bool SourceController::requestReplay(const QString &path) {
  return transition(SourceState::Replay, [&] {
    return m_hooks.setupReplay ? m_hooks.setupReplay(path) : true;
  });
}

void SourceController::forceIdle() {
  teardown(m_state);
  m_prev = m_state;
  m_state = SourceState::Idle;
  if (m_hooks.applyFeed) m_hooks.applyFeed(m_state);
  emit stateChanged(m_state, m_prev);
}
