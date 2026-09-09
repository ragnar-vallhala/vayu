#pragma once

#include "DroneProtocol.h"
#include "VehicleState.h"
#include "RollingStats.h" // src/ui/widgets is on the include path
#include "RecordSink.h"   // src/ui/main is on the include path

#include <QByteArray>
#include <QElapsedTimer>
#include <QMutex>
#include <QObject>
#include <QString>

class SerialManager;
class UdpManager;

// ---------------------------------------------------------------------------
// TelemetryEngine — single processing engine that owns the parser, the live
// transports (serial + UDP), the recorder, and the canonical VehicleState store
// (docs/telemetry-engine-architecture.md).
//
// Phase B: the engine + everything it owns lives on a worker QThread, so the
// socket is drained the instant data arrives regardless of GUI/GL load (fixes
// the climbing Diff + attitude jitter). The GUI only ever:
//   - pulls snapshot() at the render rate (mutex-guarded, safe cross-thread),
//   - drives the link via the command slots below (queued invokes),
//   - reacts to the forwarded signals (auto-queued onto the GUI thread).
// Replay stays on the GUI thread and feeds feedBytes() via a queued connection;
// setReplayMode() mutes the live feed into the parser while replaying.
// ---------------------------------------------------------------------------
class TelemetryEngine : public QObject {
  Q_OBJECT

public:
  explicit TelemetryEngine(QObject *parent = nullptr);

  // Cheap immutable copy of the latest vehicle state for the UI render clock.
  VehicleState snapshot() const;

  // The owned parser. Exposed for the low-rate / event / raw consumers that
  // subscribe directly (log, heartbeat/time-sync, packet analyzer, control loop,
  // calibration, perf). Cross-thread connections to these auto-become queued.
  DroneProtocol *protocol() const { return m_protocol; }

public slots:
  // The single byte input into the parser. Live transports call it internally
  // (same thread); replay connects its bytesReceived here (queued).
  void feedBytes(const QByteArray &data);

  // ---- Command API: invoked from the GUI via QMetaObject::invokeMethod(
  //      ..., Qt::QueuedConnection) so all transport I/O runs on the worker. ----
  void openSerial(const QString &port, int baud);
  // int (not quint16) so the queued invoke uses a guaranteed-registered metatype.
  void bindUdp(int port);
  void closeLinks();
  void send(const QByteArray &pkt); // routes UDP-else-serial
  void setAutoReconnect(bool on);
  void setReconnectInterval(int ms); // base auto-reconnect retry delay
  void setReplayMode(bool on);       // compat shim: setLiveFeed(!on)
  // Source-state-machine seam (gcs-source-state-machine.md): enable/disable the
  // owned live transports' feed into the parser (true for Fc, false for
  // Idle/Sim/Autotune/Replay). The non-live sources (Replay/Sim) are connected
  // to feedBytes() by MainWindow on the GUI thread.
  void setLiveFeed(bool on);
  // qulonglong = a built-in metatype name for the queued invoke.
  void startRecording(const QString &path, qulonglong startWallClockMs);
  void stopRecording();

  // Live, packet-type-filtered export to a replayable .bin. `typeMask` is a
  // bitmask of packet types to keep (bit N => packet type N). Each matching
  // wire packet is written verbatim, so the result replays like a recording.
  void startExport(const QString &path, int typeMask);
  void stopExport();

signals:
  // Forwarded to the GUI (auto-queued — the engine lives on the worker thread).
  void connectionStateChanged(bool connected, bool isUdp);
  void errorOccurred(const QString &msg, bool isUdp);
  // Result of an openSerial/bindUdp attempt (async — open() can't return a bool
  // across the thread hop). label = serial port name / UDP label.
  void linkOpened(bool ok, const QString &label, bool isUdp);
  // TX frames (serial + UDP dataSent) for the packet analyzer.
  void txPacket(const QByteArray &pkt);
  // Live export started/stopped (path is empty when stopped) — drives the menu.
  void exportStateChanged(bool active, const QString &path);

private:
  // Live transport bytes: tee to the recorder, then (unless replaying) parse.
  void onLiveBytes(const QByteArray &b);

  DroneProtocol *m_protocol = nullptr;
  SerialManager *m_serial = nullptr;
  UdpManager *m_udp = nullptr;

  RecordSink m_recorder;
  QElapsedTimer m_elapsed;  // monotonic base for record-frame timestamps
  bool m_acceptLive = true; // false while a non-live source feeds (Sim/Replay)

  // Live, packet-type-filtered export (separate from the full recorder).
  RecordSink m_exporter;
  bool m_exporting = false;
  quint16 m_exportMask = 0; // bit N set => keep packet type N

  mutable QMutex m_mutex; // guards m_state (snapshot() ↔ updaters)
  VehicleState m_state;   // canonical store

  // Rolling attitude std-dev accumulators (kept here, not in the snapshot, so
  // accumulation stays per-packet). 50-sample window matches the old MainWindow.
  RollingStats m_attStats[3];
};
