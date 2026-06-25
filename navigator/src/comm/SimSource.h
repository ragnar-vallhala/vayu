#pragma once

#include "ITelemetrySource.h"

// Wraps the in-app simulator's telemetry as a first-class ITelemetrySource so
// Sim plugs into the same engine.setSource() seam as Replay
// (gcs-source-state-machine.md). MainWindow connects SimulatorWidget::dataReceived
// to feed(); the controller attaches/detaches this source as the Sim state is
// entered/left, so stale sim bytes can't reach the parser once Sim is torn down.
class SimSource : public ITelemetrySource {
  Q_OBJECT

public:
  using ITelemetrySource::ITelemetrySource;

public slots:
  void feed(const QByteArray &bytes) { emit bytesReceived(bytes); }
};
