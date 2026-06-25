#pragma once

#include <QObject>
#include <QString>

#include <functional>
#include <utility>

#include "SourceState.h"

// Finite state machine that owns which telemetry source is active and is the
// single authority for tx-gating, the engine feed, and the status pill
// (software/docs/roadmap/gcs-source-state-machine.md). Replaces the scattered
// MainWindow flags + the 2-state SessionState.
//
// The graph is fully connected: every request<State>() moves from the current
// state to the target by (1) tearing down the current source, (2) setting up the
// target, then (3) pointing the engine feed at the target — so callers never
// juggle the transition matrix. Collaboration with the engine/widgets is via
// injected std::function hooks, so the FSM has no Qt-widget/engine dependency and
// is unit-testable headless.
class SourceController : public QObject {
  Q_OBJECT

public:
  using QObject::QObject;

  // Each hook performs the real teardown/setup using existing MainWindow/engine
  // code; setup hooks return false on failure (e.g. bad replay file, missing
  // SITL binaries), which lands the FSM in Idle. applyFeed points the engine at
  // the new state's source (setSource/setLiveFeed). All default to no-ops so a
  // test can drive the FSM with pure spies.
  struct Hooks {
    std::function<void()> teardownFc;
    std::function<void()> teardownSim;
    std::function<void()> teardownAutotune;
    std::function<void()> teardownReplay;
    std::function<bool(const QString &port, int baud)> setupFc;
    std::function<bool()> setupSim;
    std::function<bool()> setupAutotune;
    std::function<bool(const QString &path)> setupReplay;
    std::function<void(SourceState)> applyFeed;
  };
  void setHooks(Hooks h) { m_hooks = std::move(h); }

  SourceState state() const { return m_state; }
  SourceState previous() const { return m_prev; }

  // tx is allowed only when a commandable link is active.
  bool txAllowed() const {
    return m_state == SourceState::Fc || m_state == SourceState::Sim;
  }
  bool isReplay() const { return m_state == SourceState::Replay; }
  // A source is feeding the parser (drives the UI pills / render clock).
  bool feedActive() const {
    return m_state == SourceState::Fc || m_state == SourceState::Sim ||
           m_state == SourceState::Replay;
  }

public slots:
  // Each returns true on success; a failed setup leaves the FSM in Idle.
  bool requestIdle();
  bool requestFc(const QString &port, int baud);
  bool requestSim();
  bool requestAutotune();
  bool requestReplay(const QString &path);
  // Demote to Idle on link loss / source death (teardown, no setup).
  void forceIdle();

signals:
  void stateChanged(SourceState now, SourceState prev);

private:
  bool transition(SourceState to, const std::function<bool()> &setup);
  void teardown(SourceState s);

  Hooks m_hooks;
  SourceState m_state = SourceState::Idle;
  SourceState m_prev = SourceState::Idle;
};
