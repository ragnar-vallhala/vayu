#include <QTemporaryDir>
#include <QtTest>

#include "RecordSink.h"

// Phase-1C record format round-trip (FR-LOG-05): frames written by RecordSink
// must read back byte-for-byte (and in order) through RecordReader, with the
// header preserved. Also covers empty payloads, the empty-session case, and
// rejection of a non-record file.
class TstRecordSink : public QObject {
  Q_OBJECT

private:
  QTemporaryDir m_dir;
  QString path(const QString &name) { return m_dir.filePath(name); }

private slots:
  void roundTripFramesAndHeader();
  void emptySessionHasHeaderNoFrames();
  void readerRejectsNonRecordFile();
  void writeFrameAfterCloseIsNoOp();
};

void TstRecordSink::roundTripFramesAndHeader() {
  const QString p = path("rt.bin");
  const QList<RecordFormat::Frame> frames = {
      {10, QByteArray("\x55\x01hello", 7)},
      {2500, QByteArray()}, // empty payload is legal
      {99999, QByteArray("\x00\xFF\x10", 3)},
  };

  {
    RecordSink sink;
    QVERIFY(
        sink.open(p, /*protocolVersion=*/7, /*startWallClockMs=*/123456789ull));
    QVERIFY(sink.isOpen());
    for (const auto &f : frames)
      sink.writeFrame(f.tUs, f.bytes);
    sink.close();
    QVERIFY(!sink.isOpen());
  }

  RecordReader reader;
  QVERIFY(reader.open(p));
  const RecordFormat::Header h = reader.header();
  QCOMPARE(h.magic, RecordFormat::kMagic);
  QCOMPARE(h.formatVersion, RecordFormat::kVersion);
  QCOMPARE(h.protocolVersion, quint32(7));
  QCOMPARE(h.startWallClockMs, quint64(123456789ull));

  QList<RecordFormat::Frame> got;
  RecordFormat::Frame f;
  while (reader.next(f))
    got.append(f);
  QCOMPARE(got.size(), frames.size());
  for (int i = 0; i < frames.size(); ++i) {
    QCOMPARE(got[i].tUs, frames[i].tUs);
    QCOMPARE(got[i].bytes, frames[i].bytes);
  }
}

void TstRecordSink::emptySessionHasHeaderNoFrames() {
  const QString p = path("empty.bin");
  {
    RecordSink sink;
    QVERIFY(sink.open(p, 1, 0));
    sink.close();
  }
  RecordReader reader;
  QVERIFY(reader.open(p));
  RecordFormat::Frame f;
  QVERIFY(!reader.next(f)); // clean EOF immediately after the header
}

void TstRecordSink::readerRejectsNonRecordFile() {
  const QString p = path("garbage.bin");
  QFile g(p);
  QVERIFY(g.open(QIODevice::WriteOnly));
  g.write("not a recording at all, just some bytes");
  g.close();

  RecordReader reader;
  QVERIFY(!reader.open(p)); // bad magic -> refused
}

void TstRecordSink::writeFrameAfterCloseIsNoOp() {
  const QString p = path("closed.bin");
  RecordSink sink;
  QVERIFY(sink.open(p, 1, 0));
  sink.close();
  sink.writeFrame(1, QByteArray("ignored")); // must not crash or reopen

  RecordReader reader;
  QVERIFY(reader.open(p));
  RecordFormat::Frame f;
  QVERIFY(!reader.next(f)); // nothing was appended
}

QTEST_MAIN(TstRecordSink)
#include "tst_record_sink.moc"
