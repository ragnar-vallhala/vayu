#pragma once

#include <QByteArray>
#include <QObject>

// Abstract source of inbound telemetry bytes feeding DroneProtocol's parser.
//
// MainWindow connects the *active* source's bytesReceived() into the engine's
// feedBytes(), so the entire decode -> typed-signal -> widget chain is identical
// regardless of where the bytes came from. SimSource wraps the in-app sim;
// ReplaySource replays a recorded .bin; the live serial/UDP feed is internal to
// TelemetryEngine (toggled via setLiveFeed). The SourceController picks exactly
// one at a time (docs/roadmap/gcs-source-state-machine.md).
class ITelemetrySource : public QObject {
  Q_OBJECT

public:
  using QObject::QObject;
  ~ITelemetrySource() override = default;

signals:
  // Raw inbound bytes — a live stream chunk or a replayed frame — to hand to
  // the protocol parser verbatim. The parser owns reassembly/framing, so the
  // source never has to.
  void bytesReceived(const QByteArray &data);
};
