#pragma once

#include <QByteArray>
#include <QObject>

// Abstract source of inbound telemetry bytes feeding DroneProtocol's parser.
//
// MainWindow connects the *active* source's bytesReceived() into
// DroneProtocol::processData, so the entire decode -> typed-signal -> widget
// chain is identical regardless of where the bytes came from. LiveSource wraps
// the serial/UDP transports; ReplaySource (Phase 2D) replays a recorded .bin.
// Keeping the swap at this seam — not in the widgets — is what makes
// whole-GCS replay structural-but-contained (docs/roadmap/gcs-log-replay.md).
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
