#pragma once

#include "ITelemetrySource.h"

class SerialManager;
class UdpManager;

// The live telemetry source: forwards inbound bytes from the serial and UDP
// transports as bytesReceived(). It does not own the transports — MainWindow
// still owns them and drives open/close/write — it only unifies their inbound
// streams behind ITelemetrySource so the active source can be swapped for a
// ReplaySource without touching the decode path or any widget.
class LiveSource : public ITelemetrySource {
  Q_OBJECT

public:
  LiveSource(SerialManager *serial, UdpManager *udp,
             QObject *parent = nullptr);
};
