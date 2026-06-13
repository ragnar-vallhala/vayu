#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include "RecordSink.h"
#include "ReplayBar.h"
#include "ReplaySource.h"

// Phase-2E GUI smoke: the bar drives a bound ReplaySource (play toggles it) and
// emits exitRequested. Buttons are reached via objectName-independent text
// lookup. Transport correctness itself is covered by tst_replay_source.
class TstReplayBar : public QObject {
  Q_OBJECT

private:
  QTemporaryDir dir;
  QString logPath;

  QPushButton *button(QWidget *w, const QString &text) {
    for (auto *b : w->findChildren<QPushButton *>())
      if (b->text() == text)
        return b;
    return nullptr;
  }

private slots:
  void initTestCase() {
    logPath = dir.filePath("rec.bin");
    RecordSink s;
    QVERIFY(s.open(logPath, 1, 0));
    s.writeFrame(0, QByteArray("a"));
    s.writeFrame(100000, QByteArray("b"));
    s.close();
  }

  void playButtonTogglesSource();
  void exitButtonEmitsSignal();
  void unbindIsSafe();
};

void TstReplayBar::playButtonTogglesSource() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  ReplayBar bar;
  bar.bind(&src);

  auto *play = button(&bar, "▶");
  QVERIFY(play);
  play->click();
  QVERIFY(src.isPlaying());
  // After starting, the button flips to the pause glyph.
  QVERIFY(button(&bar, "⏸"));
  button(&bar, "⏸")->click();
  QVERIFY(!src.isPlaying());
}

void TstReplayBar::exitButtonEmitsSignal() {
  ReplayBar bar;
  QSignalSpy spy(&bar, &ReplayBar::exitRequested);
  auto *exit = button(&bar, "Exit Replay");
  QVERIFY(exit);
  exit->click();
  QCOMPARE(spy.count(), 1);
}

void TstReplayBar::unbindIsSafe() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  ReplayBar bar;
  bar.bind(&src);
  bar.bind(nullptr);  // must not crash; controls become inert
  auto *play = button(&bar, "▶");
  QVERIFY(play);
  play->click();  // no bound source -> no effect, no crash
  QVERIFY(!src.isPlaying());
}

QTEST_MAIN(TstReplayBar)
#include "tst_replay_bar.moc"
