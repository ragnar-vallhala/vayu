#include <QtTest>

#include "SourceController.h"

// SourceController: the telemetry-source FSM. Driven with spy hooks so the full
// transition matrix (any state -> any state), the teardown-before-setup ordering,
// tx-gating, setup-failure fallback, and forceIdle are all verified headless.
// (navigator/docs/roadmap/gcs-source-state-machine.md)
class TstSourceController : public QObject {
  Q_OBJECT

private slots:
  void defaultsToIdle();
  void txAndFeedPerState();
  void fullMatrixTearsDownThenSetsUp();
  void setupFailureLandsInIdle();
  void forceIdleFromEveryState();
  void stateChangedFires();
  // Teardown cleanliness: the current source is released exactly once on exit,
  // never a source we aren't in, and a failed setup still releases the previous.
  void teardownExactlyOncePerTransition();
  void teardownTouchesOnlyCurrentState();
  void selfReentryTearsDownThenSetsUp();
  void forceIdleTearsDownOnlyCurrent();
  void setupFailureStillTearsDownPrevious();
};

namespace {

// Records every hook call into `log` so order + presence can be asserted.
struct Spy {
  QStringList log;
  bool setupOk = true;

  SourceController::Hooks hooks() {
    SourceController::Hooks h;
    h.teardownFc = [this] { log << "td:Fc"; };
    h.teardownSim = [this] { log << "td:Sim"; };
    h.teardownAutotune = [this] { log << "td:Autotune"; };
    h.teardownReplay = [this] { log << "td:Replay"; };
    h.setupFc = [this](const QString &, int) {
      log << "su:Fc";
      return setupOk;
    };
    h.setupSim = [this] {
      log << "su:Sim";
      return setupOk;
    };
    h.setupAutotune = [this] {
      log << "su:Autotune";
      return setupOk;
    };
    h.setupReplay = [this](const QString &) {
      log << "su:Replay";
      return setupOk;
    };
    h.applyFeed = [this](SourceState s) { log << ("feed:" + name(s)); };
    return h;
  }

  static QString name(SourceState s) {
    switch (s) {
    case SourceState::Idle:
      return "Idle";
    case SourceState::Fc:
      return "Fc";
    case SourceState::Sim:
      return "Sim";
    case SourceState::Autotune:
      return "Autotune";
    case SourceState::Replay:
      return "Replay";
    }
    return "?";
  }
};

const SourceState kAll[] = {SourceState::Idle, SourceState::Fc,
                            SourceState::Sim, SourceState::Autotune,
                            SourceState::Replay};

void drive(SourceController &c, SourceState s) {
  switch (s) {
  case SourceState::Idle:
    c.requestIdle();
    break;
  case SourceState::Fc:
    c.requestFc("COM1", 115200);
    break;
  case SourceState::Sim:
    c.requestSim();
    break;
  case SourceState::Autotune:
    c.requestAutotune();
    break;
  case SourceState::Replay:
    c.requestReplay("flight.bin");
    break;
  }
}

} // namespace

void TstSourceController::defaultsToIdle() {
  SourceController c;
  QCOMPARE(c.state(), SourceState::Idle);
  QVERIFY(!c.txAllowed());
  QVERIFY(!c.feedActive());
  QVERIFY(!c.isReplay());
}

void TstSourceController::txAndFeedPerState() {
  SourceController c;
  Spy spy;
  c.setHooks(spy.hooks());

  drive(c, SourceState::Fc);
  QVERIFY(c.txAllowed());
  QVERIFY(c.feedActive());

  drive(c, SourceState::Sim);
  QVERIFY(c.txAllowed()); // sim is commandable
  QVERIFY(c.feedActive());

  drive(c, SourceState::Replay);
  QVERIFY(!c.txAllowed()); // read-only
  QVERIFY(c.feedActive());
  QVERIFY(c.isReplay());

  drive(c, SourceState::Autotune);
  QVERIFY(!c.txAllowed());  // isolated; apply-gains needs an explicit FC switch
  QVERIFY(!c.feedActive()); // tuner does not feed the GCS engine

  drive(c, SourceState::Idle);
  QVERIFY(!c.txAllowed());
  QVERIFY(!c.feedActive());
}

void TstSourceController::fullMatrixTearsDownThenSetsUp() {
  for (SourceState from : kAll) {
    for (SourceState to : kAll) {
      Spy spy;
      SourceController c;
      c.setHooks(spy.hooks());
      drive(c, from);
      spy.log.clear();
      drive(c, to);

      QCOMPARE(c.state(), to);
      QCOMPARE(c.previous(), from);

      const QString td = "td:" + Spy::name(from);
      const QString su = "su:" + Spy::name(to);
      if (from != SourceState::Idle)
        QVERIFY2(spy.log.contains(td),
                 qPrintable(td + " from " + Spy::name(from)));
      if (to != SourceState::Idle)
        QVERIFY2(spy.log.contains(su), qPrintable(su + " to " + Spy::name(to)));
      // The new feed is always pointed at the target last.
      QVERIFY(spy.log.contains("feed:" + Spy::name(to)));
      // Old source is always torn down before the new one is set up.
      if (from != SourceState::Idle && to != SourceState::Idle)
        QVERIFY(spy.log.indexOf(td) < spy.log.indexOf(su));
    }
  }
}

void TstSourceController::setupFailureLandsInIdle() {
  Spy spy;
  spy.setupOk = false;
  SourceController c;
  c.setHooks(spy.hooks());
  drive(c, SourceState::Fc); // setup fails

  QVERIFY(c.requestReplay("x.bin") == false);
  QCOMPARE(c.state(), SourceState::Idle);
  QVERIFY(!c.txAllowed());
  QVERIFY(spy.log.contains("su:Replay"));
  QVERIFY(spy.log.contains("feed:Idle")); // engine pointed at nothing
}

void TstSourceController::forceIdleFromEveryState() {
  for (SourceState from : kAll) {
    Spy spy;
    SourceController c;
    c.setHooks(spy.hooks());
    drive(c, from);
    spy.log.clear();
    c.forceIdle();

    QCOMPARE(c.state(), SourceState::Idle);
    if (from != SourceState::Idle)
      QVERIFY(spy.log.contains("td:" + Spy::name(from)));
    QVERIFY(spy.log.contains("feed:Idle"));
  }
}

void TstSourceController::stateChangedFires() {
  SourceController c;
  Spy spy;
  c.setHooks(spy.hooks());
  qRegisterMetaType<SourceState>();
  QSignalSpy sig(&c, &SourceController::stateChanged);

  drive(c, SourceState::Fc);
  drive(c, SourceState::Replay);
  QCOMPARE(sig.count(), 2);
  const auto args = sig.takeLast();
  QCOMPARE(args.at(0).value<SourceState>(), SourceState::Replay);
  QCOMPARE(args.at(1).value<SourceState>(), SourceState::Fc);
}

// Count teardown calls in the log (entries starting "td:").
static int teardownCount(const QStringList &log) {
  int n = 0;
  for (const QString &e : log)
    if (e.startsWith("td:"))
      ++n;
  return n;
}

void TstSourceController::teardownExactlyOncePerTransition() {
  for (SourceState from : kAll) {
    for (SourceState to : kAll) {
      Spy spy;
      SourceController c;
      c.setHooks(spy.hooks());
      drive(c, from);
      spy.log.clear();
      drive(c, to);
      // Exactly one source is released per transition (the one we were in), and
      // none when leaving Idle.
      const int expected = (from == SourceState::Idle) ? 0 : 1;
      QCOMPARE(teardownCount(spy.log), expected);
      if (from != SourceState::Idle)
        QCOMPARE(spy.log.count("td:" + Spy::name(from)), 1);
    }
  }
}

void TstSourceController::teardownTouchesOnlyCurrentState() {
  Spy spy;
  SourceController c;
  c.setHooks(spy.hooks());
  drive(c, SourceState::Fc);
  spy.log.clear();
  drive(c, SourceState::Replay);
  // Leaving Fc must not tear down sources we were never in.
  QCOMPARE(spy.log.count("td:Fc"), 1);
  QCOMPARE(spy.log.count("td:Sim"), 0);
  QCOMPARE(spy.log.count("td:Autotune"), 0);
  QCOMPARE(spy.log.count("td:Replay"), 0);
}

void TstSourceController::selfReentryTearsDownThenSetsUp() {
  // Re-entry (e.g. re-open a replay, re-connect Fc) releases the old instance
  // before setting up the new one — exactly once each, in order.
  Spy spy;
  SourceController c;
  c.setHooks(spy.hooks());
  drive(c, SourceState::Replay);
  spy.log.clear();
  drive(c, SourceState::Replay);
  QCOMPARE(c.state(), SourceState::Replay);
  QCOMPARE(spy.log.count("td:Replay"), 1);
  QCOMPARE(spy.log.count("su:Replay"), 1);
  QVERIFY(spy.log.indexOf("td:Replay") < spy.log.indexOf("su:Replay"));
}

void TstSourceController::forceIdleTearsDownOnlyCurrent() {
  for (SourceState from : kAll) {
    Spy spy;
    SourceController c;
    c.setHooks(spy.hooks());
    drive(c, from);
    spy.log.clear();
    c.forceIdle();
    const int expected = (from == SourceState::Idle) ? 0 : 1;
    QCOMPARE(teardownCount(spy.log), expected);
    if (from != SourceState::Idle)
      QCOMPARE(spy.log.count("td:" + Spy::name(from)), 1);
  }
}

void TstSourceController::setupFailureStillTearsDownPrevious() {
  Spy spy;
  SourceController c;
  c.setHooks(spy.hooks());
  drive(c, SourceState::Fc); // succeeds (setupOk default true)
  spy.setupOk = false;       // the next setup fails
  spy.log.clear();
  QVERIFY(!c.requestReplay("x.bin"));
  // The previous source is released exactly once even though the new setup
  // failed, and the engine feed is cleared (no dangling source).
  QCOMPARE(c.state(), SourceState::Idle);
  QCOMPARE(spy.log.count("td:Fc"), 1);
  QCOMPARE(spy.log.count("feed:Idle"), 1);
}

QTEST_APPLESS_MAIN(TstSourceController)
#include "tst_source_controller.moc"
