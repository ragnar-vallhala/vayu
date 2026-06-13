#pragma once

#include <QElapsedTimer>
#include <QHash>
#include <QLabel>
#include <QMainWindow>
#include <QSplitter>
#include <QStackedWidget>
#include <QTimer>

#include "../widgets/CalibrationWidget.h"
#include "../widgets/ControlLoopPlot.h"
#include "../widgets/PerfWidget.h"
#include "AttitudeWidget.h"
#include "CommandRegistry.h"
#include "ShortcutsManager.h"
#include "SessionMode.h"
#include "ViewHistory.h"
#include "Drone3DWidget.h" // Added
#include "DroneProtocol.h"
#include "ImuPanel.h"
#include "LiveSource.h"
#include "LogPanel.h"
#include "MainStatusBar.h"
#include "MainToolbar.h"
#include "MotorStatusWidget.h"
#include "PacketAnalyzerWidget.h"
#include "RecordSink.h"
#include "RcChannelsWidget.h"
#include "RollingStats.h"
#include "SerialManager.h"
#include "SettingsWidget.h"
#include "UdpManager.h"
#include "Types.h"

#ifdef NAVIGATOR_HAS_SITL
#include "SimulatorWidget.h"
#endif

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

  // Data callbacks
  void onImuReceived(const ImuData &data);
  void onAttitudeReceived(const AttitudeData &data);
  void onRcReceived(const RcData &data);
  void onLogReceived(const QString &msg);
  void onStatusReceived(const QString &msg);
  void onFlightModeReceived(quint8 mode, quint8 source);
  void onMotorReceived(const MotorData &data);

  // Serial state
  void onConnectionStateChanged(bool connected);
  void onSerialError(const QString &msg);

  // Periodic UI refresh
  void onUiTimer();

  // New data slots
  void onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId);
  void onTimeSyncRequested();

private:
  void buildUi();
  void buildMenuBar();
  void installShortcuts();
  void setConnected(bool on);
  // Refresh the bottom status-bar connection pill from m_connected /
  // m_simRunning. Serial link wins; otherwise shows "Connected: SIM".
  void refreshConnectionPill();
  void updateLiveBlinker();
  void saveUiState();
  void restoreUiState();
  void persistPortBaud();
  // Telemetry recording (Phase-1 1C): open a timestamped .bin and tee the
  // live stream while connected; close it on disconnect.
  void startRecording();
  void stopRecording();

public:
  // Enter/leave whole-GCS replay (Phase-1 1D). Drives the read-only authority
  // and the toolbar REPLAY pill; the ReplaySource swap arrives in Phase 2E.
  void setSessionMode(SessionMode mode);
  SessionMode sessionMode() const { return m_session.mode(); }

private:

  // Send a command frame to the FC over whichever transport is connected
  // (UDP bridge or serial). Used by every ARM/PID/calibrate/time-sync path.
  void sendToFc(const QByteArray &pkt);

  // ---- Toolbar / status bar (extracted in Phase-1 1a) ----
  MainToolbar   *m_toolbar   = nullptr;
  MainStatusBar *m_statusBar = nullptr;

  // ---- Central panels ----
  ImuPanel *m_imuPanel = nullptr;
  AttitudeWidget *m_attitude = nullptr;
  LogPanel *m_logPanel = nullptr;
  PacketAnalyzerWidget *m_analyzerWidget = nullptr;
  RcChannelsWidget *m_rcWidget = nullptr;
  SettingsWidget *m_settingsWidget = nullptr;
  CalibrationWidget *m_calibrationWidget = nullptr;
  MotorStatusWidget *m_motorWidget = nullptr;
  ControlLoopPlot *m_controlLoopWidget = nullptr;
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
  RollingStats m_attStats[3];

  // ---- System-state pill on the attitude page (firmware state, not connection) ----
  QLabel *m_statusLabel = nullptr;
  QLabel *m_flightModeLabel = nullptr;   // STABILISE / ACRO pill (+ RC/GCS source)

  // ---- Back-end ----
  SerialManager *m_serial = nullptr;
  UdpManager *m_udp = nullptr;
  DroneProtocol *m_protocol = nullptr;
  // Telemetry-source seam (Phase-1 1B): inbound bytes reach the parser
  // through the *active* ITelemetrySource. m_liveSource fans in serial+UDP;
  // a ReplaySource (Phase 2D) swaps in here without touching any widget.
  LiveSource *m_liveSource = nullptr;
  ITelemetrySource *m_source = nullptr;
  // Records the live stream to a .bin for later replay (Phase-1 1C).
  RecordSink m_recorder;
  bool m_recordOnConnect = false;
  // Read-only authority for replay (Phase-1 1D). Lives here so every tx site
  // (all routed through sendToFc) checks one place.
  SessionState m_session;
  QTimer *m_uiTimer = nullptr;
  QTimer *m_syncTimer = nullptr;
  QElapsedTimer m_elapsed;

  // ---- State ----
  bool m_connected = false;   // real serial link
  bool m_simRunning = false;  // in-app SITL active
  bool m_armed = false;
  int m_pktCount = 0;
  ImuData m_latestImu;
  AttitudeData m_latestAtt;
  qint64 m_lastHbTime = 0;
};
