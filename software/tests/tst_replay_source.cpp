#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include "RecordSink.h"
#include "ReplaySource.h"

// Phase-2D: a recorded .bin replays as an ITelemetrySource. The wall-clock is
// bypassed by driving advanceTo() with explicit virtual times, so playback is
// deterministic: due-frame emission, end/finished, crop range, loop wrap, and
// seek clamping.
class TstReplaySource : public QObject {
  Q_OBJECT

private:
  QTemporaryDir dir;
  QString logPath;

  // Frames at absolute 1000,1100,1200,1300 -> normalised 0,100,200,300.
  void writeLog() {
    logPath = dir.filePath("rec.bin");
    RecordSink sink;
    QVERIFY(sink.open(logPath, 1, 0));
    sink.writeFrame(1000, QByteArray("f0"));
    sink.writeFrame(1100, QByteArray("f1"));
    sink.writeFrame(1200, QByteArray("f2"));
    sink.writeFrame(1300, QByteArray("f3"));
    sink.close();
  }

private slots:
  void initTestCase() { writeLog(); }

  void openNormalisesToZeroBase();
  void advanceEmitsDueFramesInOrder();
  void finishedFiresAtEndWhenNotLooping();
  void loopWrapsWithoutFinishing();
  void cropRangeLimitsEmission();
  void seekClampsIntoRange();
  void speedRejectsNonPositive();
};

void TstReplaySource::openNormalisesToZeroBase() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  QCOMPARE(src.frameCount(), 4);
  QCOMPARE(src.durationUs(), qint64(300));
  QCOMPARE(src.positionUs(), qint64(0));
}

void TstReplaySource::advanceEmitsDueFramesInOrder() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  QSignalSpy spy(&src, &ITelemetrySource::bytesReceived);

  QCOMPARE(src.advanceTo(50), 1);   // frame at t=0
  QCOMPARE(src.advanceTo(250), 2);  // t=100, t=200
  QCOMPARE(spy.count(), 3);
  QCOMPARE(spy.at(0).at(0).toByteArray(), QByteArray("f0"));
  QCOMPARE(spy.at(1).at(0).toByteArray(), QByteArray("f1"));
  QCOMPARE(spy.at(2).at(0).toByteArray(), QByteArray("f2"));
}

void TstReplaySource::finishedFiresAtEndWhenNotLooping() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  QSignalSpy spyData(&src, &ITelemetrySource::bytesReceived);
  QSignalSpy spyFin(&src, &ReplaySource::finished);

  src.advanceTo(10000);  // past the end
  QCOMPARE(spyData.count(), 4);
  QCOMPARE(spyFin.count(), 1);
  QVERIFY(!src.isPlaying());
}

void TstReplaySource::loopWrapsWithoutFinishing() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  src.setLoop(true);
  QSignalSpy spyData(&src, &ITelemetrySource::bytesReceived);
  QSignalSpy spyFin(&src, &ReplaySource::finished);

  src.advanceTo(10000);            // emits all 4, wraps to start
  QCOMPARE(spyData.count(), 4);
  QCOMPARE(spyFin.count(), 0);     // looping never "finishes"
  QCOMPARE(src.positionUs(), qint64(0));
  src.advanceTo(10000);            // replays the whole crop again
  QCOMPARE(spyData.count(), 8);
}

void TstReplaySource::cropRangeLimitsEmission() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  src.setRange(100, 200);          // auto-seeks to 100
  QCOMPARE(src.positionUs(), qint64(100));
  QSignalSpy spy(&src, &ITelemetrySource::bytesReceived);

  src.advanceTo(10000);
  QCOMPARE(spy.count(), 2);        // only t=100 and t=200
  QCOMPARE(spy.at(0).at(0).toByteArray(), QByteArray("f1"));
  QCOMPARE(spy.at(1).at(0).toByteArray(), QByteArray("f2"));
}

void TstReplaySource::seekClampsIntoRange() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  src.seek(99999);
  QCOMPARE(src.positionUs(), src.durationUs());
  src.seek(-50);
  QCOMPARE(src.positionUs(), qint64(0));

  src.setRange(100, 200);
  src.seek(0);
  QCOMPARE(src.positionUs(), qint64(100));  // clamped up into the crop
}

void TstReplaySource::speedRejectsNonPositive() {
  ReplaySource src;
  QVERIFY(src.open(logPath));
  src.setSpeed(2.0);
  QCOMPARE(src.speed(), 2.0);
  src.setSpeed(-1.0);
  QCOMPARE(src.speed(), 2.0);  // ignored
  src.setSpeed(0.0);
  QCOMPARE(src.speed(), 2.0);  // ignored
}

QTEST_MAIN(TstReplaySource)
#include "tst_replay_source.moc"
