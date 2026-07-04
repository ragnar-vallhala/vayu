#pragma once

#include <QElapsedTimer>
#include <QHash>
#include <QLabel>
#include <QMainWindow>
#include <QSplitter>
#include <QStackedWidget>
#include <QThread>
#include <QTimer>

#include "../widgets/CalibrationWidget.h"
#include "../widgets/ControlLoopPlot.h"
#include "../widgets/GyroNotchWidget.h"
#include "../../audio/ChimeAudio.h"
#include "../widgets/PerfWidget.h"
#include "AttitudeWidget.h"
#include "CommandRegistry.h"
#include "ShortcutsManager.h"
#include "SourceController.h"
#include "SimSource.h"
#include <functional>
#include "ViewHistory.h"
#include "Drone3DWidget.h" // Added
#include "TelemetryEngine.h" // owns DroneProtocol + the VehicleState store
#include "TimeSyncEstimator.h" // NTP clock-sync filter for the time-sync handshake
#include "ImuPanel.h"
#include "ITelemetrySource.h"  // replay feeds the engine via bytesReceived
#include "LogPanel.h"
#include "MainStatusBar.h"
#include "MainToolbar.h"
#include "MotorStatusWidget.h"
#include "PacketAnalyzerWidget.h"
#include "RcChannelsWidget.h"
#include "SettingsWidget.h"

#ifdef NAVIGATOR_HAS_SITL
#include "SimulatorWidget.h"
#endif

class QToolBar;
class QAction;
class QMenu;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

protected:
  void closeEvent(QCloseEvent *event) override;

private slots:
  // Navigation
  void showHome();
  void showPacketAnalyzer();
  void showRcMonitor();
  void showSettings();
  void showCalibration();
  void showMotorStatus();
  void showControlLoopPlot();
  void showGyroNotch();
  void showPerf();
  // Send a PERF_TASKNAME request for one task id (FC replies with the name).
  void sendTaskNameRequest(int taskId);
  void showSimulator();
  // Toolbar wire-up. The toolbar emits intent signals; MainWindow owns
  // the serial open/close + persistence side-effects.
  void onConnectRequested(const QString &port, int baud);
  void onDisconnectRequested();
  void onArmClicked();
  void onToggle3d(bool checked);

  // Data callbacks. Per-packet telemetry (imu/attitude/rc/motor/status/flight-
  // mode) is consumed inside TelemetryEngine and pulled by the render clock via
  // snapshot() — only the low-rate log feed is handled by a slot here.
  void onLogReceived(const QString &msg);

  // Serial state
  void onConnectionStateChanged(bool connected);
  void onSerialError(const QString &msg);

  // Periodic UI refresh
  void onUiTimer();

  // New data slots
  void onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId);
  void onTimeSyncRequested();
  void onTimeSyncResponse(quint8 seq, quint64 t1, quint64 t2, quint64 t3,
                          quint64 t4);

  // File ▸ Export Log…: toggles a live, packet-type-filtered .bin export. When
  // idle, prompts for streams + a folder and starts; when active, stops.
  void onExportLogToggle();

private:
  void buildUi();
  void buildMenuBar();
  void installShortcuts();
  // Window ▸ Reset Layout — restore the home splitters to defaults + show home.
  void resetLayout();
  void setConnected(bool on);
  // Refresh the bottom status-bar connection pill from m_connected /
  // m_simRunning. Serial link wins; otherwise shows "Connected: SIM".
  void refreshConnectionPill();
  void updateLiveBlinker();
  // Apply the firmware system-state pill (attitude page + graph state band + sim
  // HUD) and the flight-mode pill. Render-clock driven, called only when the
  // snapshot's value changes (see onUiTimer edge-guards).
  void applyVehicleStatePill(const QString &state);
  void applyFlightModePill(quint8 mode, quint8 source);
  void saveUiState();
  void restoreUiState();
  void persistPortBaud();
  // Telemetry recording (FR-LOG-05): open a timestamped .bin and tee the
  // live stream while connected; close it on disconnect.
  void startRecording();
  void stopRecording();
  // Recent-logs MRU (File ▸ Open Recent Log): persisted list of the last N
  // .bin paths the user opened OR recorded. addRecentLog prepends + dedups +
  // caps; rebuildRecentLogsMenu repopulates the submenu (prunes missing files).
  void addRecentLog(const QString &path);
  void rebuildRecentLogsMenu();
  void openLastRecentLog();  // replay the most recent existing log
  // Export Log is only meaningful with telemetry coming in; enabled when a live
  // link or the sim is feeding, or while an export is already running (to stop).
  void updateExportEnabled();
  // Apply the staged Settings form to the running app (sync timer, graphs,
  // reconnect, link-loss, record-on-connect, recent-views). persist=true also
  // writes them to disk — called from the Settings Apply button.
  void applyAllSettings(bool persist);

public:
  // Open a recorded .bin and drive the whole GCS from it (Phase-2 2E); routes
  // through the SourceController (Replay state) which swaps the active source.
  void enterReplay(const QString &path);
  void exitReplay();

private:
  // Source state machine plumbing (gcs-source-state-machine.md).
  void buildSourceHooks();  // wire SourceController::Hooks to existing functions
  void applyFeed(SourceState s);  // point the engine/parser at the state's source
  // Run a deliberate FSM transition, suppressing re-entrant reconcile requests
  // (e.g. a teardown that stops the sim, which fires simRunningChanged).
  void runTransition(const std::function<void()> &body);

  // Send a command frame to the FC over whichever transport is connected
  // (UDP bridge or serial). Used by every ARM/PID/calibrate/time-sync path.
  void sendToFc(const QByteArray &pkt);

  // ---- Toolbar / status bar (extracted in Phase-1 1a) ----
  MainToolbar   *m_toolbar   = nullptr;
  MainStatusBar *m_statusBar = nullptr;

  // ---- Central panels ----
  ImuPanel *m_imuPanel = nullptr;
  // Baro AGL ground reference (MSL captured while not flying); see onUiTimer.
  float m_baroGroundRefM = 0.0f;
  bool m_haveBaroRef = false;
  AttitudeWidget *m_attitude = nullptr;
  LogPanel *m_logPanel = nullptr;
  PacketAnalyzerWidget *m_analyzerWidget = nullptr;
  RcChannelsWidget *m_rcWidget = nullptr;
  SettingsWidget *m_settingsWidget = nullptr;
  CalibrationWidget *m_calibrationWidget = nullptr;
  MotorStatusWidget *m_motorWidget = nullptr;
  ControlLoopPlot *m_controlLoopWidget = nullptr;
  GyroNotchWidget *m_gyroNotchWidget = nullptr;
  PerfWidget *m_perfWidget = nullptr;
#ifdef NAVIGATOR_HAS_SITL
  SimulatorWidget *m_simulatorWidget = nullptr;
#endif
  QStackedWidget *m_stackedWidget = nullptr;
  QStackedWidget *m_attStack = nullptr;
  Drone3DWidget *m_drone3d = nullptr;
  QWidget *m_homeWidget = nullptr;

  // Splitters retained as members so their state can be saved/restored.
  QSplitter *m_topSplitter = nullptr;
  QSplitter *m_vSplitter = nullptr;

  // ---- Command layer (Phase-1 1g / FR-UX-19) ----
  // Single source of truth for every command; menus, the toolbar, and the
  // keyboard shortcuts all draw their QActions from here.
  CommandRegistry *m_cmds = nullptr;
  // Editable keyboard-shortcut overrides on top of the registry defaults
  // (Phase-2 2A / FR-UX-19); persisted across runs.
  ShortcutsManager *m_shortcuts = nullptr;
  // Recent-views (MRU) switcher (Phase-2 2C / FR-UX-21).
  ViewHistory m_viewHistory;
  class RecentViewsOverlay *m_recentOverlay = nullptr;
  QHash<int, QString> m_viewTitles;  // stacked index -> human label
  void buildViewTitles();
  void showRecentViews();

  // ---- Attitude numeric labels ----
  QLabel *m_rollLabel = nullptr;
  QLabel *m_pitchLabel = nullptr;
  QLabel *m_yawLabel = nullptr;
  QLabel *m_rollStd = nullptr;
  QLabel *m_pitchStd = nullptr;
  QLabel *m_yawStd = nullptr;

  // ---- System-state pill on the attitude page (firmware state, not connection) ----
  QLabel *m_statusLabel = nullptr;
  QLabel *m_flightModeLabel = nullptr;   // STABILISE / ACRO pill (+ RC/GCS source)

  // ---- Back-end ----
  // Single processing engine: owns the parser, the live transports (serial +
  // UDP) and the recorder, and the canonical VehicleState the UI pulls at the
  // render rate. Lives on m_worker (Phase B) so the socket is drained off the
  // GUI thread — see docs/telemetry-engine-architecture.md. The GUI drives it
  // via queued invokes and reads snapshot()/forwarded signals.
  TelemetryEngine *m_engine = nullptr;
  QThread *m_worker = nullptr;
  // Whole-GCS replay (Phase-2 2E): the replay source + transport bar, active
  // only while in SourceState::Replay. Replay stays on the GUI thread and feeds
  // the engine via a queued bytesReceived → feedBytes connection.
  class ReplaySource *m_replaySource = nullptr;
  class ReplayBar *m_replayBar = nullptr;
  QToolBar *m_replayToolbar = nullptr;
  QMenu *m_recentLogsMenu = nullptr;  // File ▸ Open Recent Log (rebuilt on show)
  bool m_recordOnConnect = false;
  // Telemetry-source state machine (gcs-source-state-machine.md) — the single
  // authority for the active source, tx-gating, the engine feed, and the pill.
  SourceController m_source;
  SimSource *m_simSource = nullptr;                   // in-app sim as a source
  ITelemetrySource *m_activeFeedSource = nullptr;     // source wired to feedBytes
  bool m_fsmBusy = false;                             // suppress reconcile re-entry
  QTimer *m_uiTimer = nullptr;
  QTimer *m_syncTimer = nullptr;
  QElapsedTimer m_elapsed;

  // Time-sync handshake (docs/telemetry/time_sync.md). The estimator filters the
  // NTP samples; m_syncCorrection is the modular int32 clock correction pushed
  // to the FC on the next request (INT32_MIN = "no command", before synced).
  TimeSyncEstimator m_tsEst;
  quint8 m_syncSeq = 0;
  qint32 m_syncCorrection = (-2147483647 - 1);  // INT32_MIN sentinel
  // When the full correction exceeds int32 ms (cold-start FC uptime vs epoch),
  // send it via the REQUEST_WIDE two-word path instead of the int32 field.
  bool m_syncWide = false;
  qint64 m_syncWideOffset = 0;
  // Acquisition vs steady-state cadence: poll fast until the clock is locked,
  // then fall back to the configured period so the first correction lands in a
  // second or two instead of after several 5 s round-trips.
  bool m_syncLocked = false;
  int m_syncPeriodMs = 5000;            // steady-state period (from settings)
  static constexpr int kSyncFastMs = 400;  // acquisition period until locked

  // ---- State ----
  bool m_armed = false;
  // Link caches (the transports live on the worker thread; the GUI must not read
  // them directly). Updated from the engine's forwarded signals / at connect.
  bool m_serialOpen = false;
  bool m_udpOpen = false;
  QString m_currentPort;      // label for the active link (serial port / UDP)
  // Recording lives in the engine (worker thread); the GUI remembers the active
  // path (non-empty = recording) for the start/stop guard + log lines.
  QString m_recordingPath;
  // Status-bar "Packets: N" is shown relative to the current connection; the
  // engine's wire-packet counter is monotonic, so we subtract a baseline taken
  // at connect. "Rate: N Hz" is a delta over a 1 s window.
  quint64 m_pktBase = 0;
  quint64 m_pktAtLastRate = 0;
  qint64 m_lastRateTime = 0;
  qint64 m_lastHbTime = 0;
  // Link-loss watchdog: flag the pill disconnected after this many ms with no
  // heartbeat on a live link (non-destructive; the transport stays open).
  int m_linkLossTimeoutMs = 1500;
  bool m_linkLost = false;
  // Units & Display launch behaviour (set by applyAllSettings, read by
  // restoreUiState). startupPage: 0 last-viewed, 1 dashboard, 2 simulator.
  int m_startupPage = 0;
  bool m_restoreLayout = true;
  // Logging & Recording. m_logDir empty ⇒ the ~/vayu-logs default. Recordings
  // and the on-disconnect session-log dump are written here.
  QString m_logDir;
  bool m_exportOnDisconnect = false;
  // Alerts & Audio. Chimes for arm/disarm/failsafe; arm confirmation dialog.
  ChimeAudio m_chime;
  bool m_confirmBeforeArm = true;
  // Resolve the effective log directory (settings override or the default),
  // creating it if needed. Returns the absolute path.
  QString resolveLogDir() const;
  // Edge-guards so the sparse pills only restyle on change — the render clock
  // reads the snapshot at 30 Hz and must not rebuild stylesheets every tick.
  QString m_lastPushedState;
  int m_lastFlightMode = -1;
  int m_lastFlightSrc = -1;
  // Live-export menu action + state (label toggles Export/Stop; engine owns the
  // actual export on the worker thread).
  QAction *m_exportAction = nullptr;
  bool m_exportActive = false;
  static constexpr qint64 kTelemetryStaleMs = 1000;
};
