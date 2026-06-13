#include "LiveSource.h"

#include "SerialManager.h"
#include "UdpManager.h"

LiveSource::LiveSource(SerialManager *serial, UdpManager *udp, QObject *parent)
    : ITelemetrySource(parent) {
  // Both transports already emit raw inbound bytes; fan them into the one
  // ITelemetrySource signal. The parser (DroneProtocol) is indifferent to
  // which transport a chunk came from, exactly as before this seam existed.
  if (serial)
    connect(serial, &SerialManager::dataReceived, this,
            &LiveSource::bytesReceived);
  if (udp)
    connect(udp, &UdpManager::dataReceived, this,
            &LiveSource::bytesReceived);
}
