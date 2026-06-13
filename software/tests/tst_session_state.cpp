#include <QSignalSpy>
#include <QtTest>

#include "SessionMode.h"

// Phase-1D read-only authority: SessionState is the single place every tx site
// consults. Tests the txAllowed/isReplay invariants and that changed() fires
// exactly once per real transition (not on a no-op set).
class TstSessionState : public QObject {
  Q_OBJECT

private slots:
  void defaultsToLiveAndTxAllowed();
  void replayBlocksTx();
  void changedFiresOncePerTransition();
  void returningToLiveReEnablesTx();
};

void TstSessionState::defaultsToLiveAndTxAllowed() {
  SessionState s;
  QCOMPARE(s.mode(), SessionMode::Live);
  QVERIFY(s.txAllowed());
  QVERIFY(!s.isReplay());
}

void TstSessionState::replayBlocksTx() {
  SessionState s;
  s.setMode(SessionMode::Replay);
  QVERIFY(s.isReplay());
  QVERIFY(!s.txAllowed());
}

void TstSessionState::changedFiresOncePerTransition() {
  SessionState s;
  QSignalSpy spy(&s, &SessionState::changed);
  s.setMode(SessionMode::Replay);  // transition -> 1 signal
  s.setMode(SessionMode::Replay);  // no-op -> no signal
  s.setMode(SessionMode::Live);    // transition -> 1 signal
  QCOMPARE(spy.count(), 2);
  QCOMPARE(spy.at(0).at(0).value<SessionMode>(), SessionMode::Replay);
  QCOMPARE(spy.at(1).at(0).value<SessionMode>(), SessionMode::Live);
}

void TstSessionState::returningToLiveReEnablesTx() {
  SessionState s;
  s.setMode(SessionMode::Replay);
  s.setMode(SessionMode::Live);
  QVERIFY(s.txAllowed());
  QVERIFY(!s.isReplay());
}

QTEST_MAIN(TstSessionState)
#include "tst_session_state.moc"
