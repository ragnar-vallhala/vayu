#include <QHostAddress>
#include <QSignalSpy>
#include <QUdpSocket>
#include <QtTest>

#include "ITelemetrySource.h"
#include "LiveSource.h"
#include "SerialManager.h"
#include "UdpManager.h"

// Phase-1B telemetry-source seam: the inbound byte path must be identical to
// the old direct serial/UDP -> DroneProtocol wiring, just routed through
// ITelemetrySource. Tests:
//  1) the ITelemetrySource contract (a source emits bytesReceived verbatim);
//  2) LiveSource forwards a real UDP datagram end-to-end (loopback).
class TstLiveSource : public QObject {
  Q_OBJECT

private slots:
  void interfaceEmitsBytesVerbatim();
  void liveSourceForwardsUdp();
  void nullTransportsAreSafe();
};

// Minimal concrete source to exercise the interface signal deterministically.
class FakeSource : public ITelemetrySource {
  Q_OBJECT
public:
  void feed(const QByteArray &b) { emit bytesReceived(b); }
};

void TstLiveSource::interfaceEmitsBytesVerbatim() {
  FakeSource src;
  QSignalSpy spy(&src, &ITelemetrySource::bytesReceived);
  const QByteArray payload("\xAB\x12\x00\xFFhello", 9);
  src.feed(payload);
  QCOMPARE(spy.count(), 1);
  QCOMPARE(spy.at(0).at(0).toByteArray(), payload);
}

void TstLiveSource::liveSourceForwardsUdp() {
  SerialManager serial;  // never opened — its branch stays silent
  UdpManager udp;
  // Bind an ephemeral port; if the sandbox forbids UDP bind, skip rather
  // than fail.
  quint16 port = 0;
  for (quint16 p : {45821, 45822, 45823, 45824}) {
    if (udp.bind(p)) { port = p; break; }
  }
  if (port == 0)
    QSKIP("UDP bind unavailable in this environment");

  LiveSource live(&serial, &udp);
  QSignalSpy spy(&live, &ITelemetrySource::bytesReceived);

  const QByteArray frame("\x55\x01\x02\x03frame", 9);
  QUdpSocket sender;
  qint64 sent =
      sender.writeDatagram(frame, QHostAddress::LocalHost, port);
  QCOMPARE(sent, qint64(frame.size()));

  // UDP loopback delivers via the event loop; QTRY spins it.
  QTRY_VERIFY_WITH_TIMEOUT(spy.count() >= 1, 2000);
  QCOMPARE(spy.at(0).at(0).toByteArray(), frame);
}

void TstLiveSource::nullTransportsAreSafe() {
  // Constructing over null transports must not crash (defensive guard).
  LiveSource live(nullptr, nullptr);
  QSignalSpy spy(&live, &ITelemetrySource::bytesReceived);
  QCOMPARE(spy.count(), 0);
}

QTEST_MAIN(TstLiveSource)
#include "tst_live_source.moc"
