#include "MainWindow.h"
#include "../core/Notify.h"
#include "../core/SettingsManager.h"
#include "../protocol/CommandCodec.h"
#include "../replay/ReplaySource.h"
#include "../ui/widgets/AboutDialog.h"
#include "../ui/widgets/CommandPalette.h"
#include "../ui/widgets/RecentViewsOverlay.h"
#include "../ui/widgets/ReplayBar.h"
#include "../ui/widgets/ShortcutsEditorDialog.h"

#include <QDesktopServices>
#include <QFileDialog>
#include <QToolBar>
#include <QUrl>
#include "../core/Theme.h"
#include "../core/Units.h"
#include "../core/ui/Icons.h"
#include "comm/PortArbiter.h"

#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QSettings>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>
#include <cstdint>
#include <cstring>

#ifdef NAVIGATOR_HAS_SITL
// In-process firmware (libvayu_sitl_core) PID apply — the exact function the
// real FC runs for CMD_SET_PID: validate the payload, push to the live
// controllers, AND persist to 0:pid.bin (host VFS -> $VAYU_VFS_DIR, default
// /tmp/vayu_vfs), which host_lifecycle reloads on boot. Returns VAYU_OK (0).
// Lets "Apply Gains" target the in-app sim persistently, with no FC link.
extern "C" int pid_config_apply_command(const uint8_t *payload,
                                        uint16_t payload_len);
// CMD_SET_PID command id (firmware comm_types.h is not on the GCS include path;
// the dissector hardcodes the same value).
static constexpr uint16_t kCmdSetPid = 0x000A;
#endif

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  setWindowTitle("Vayu GCS — Ground Control Station");
  setMinimumSize(1100, 700);
  resize(1300, 820);

  // Palette + stylesheet are applied in main() via Theme::apply() so
  // every dialog/secondary window picks them up; nothing to do here.

  // Back-end engine (Phase B): owns the parser + the live transports (serial +
  // UDP) + the recorder. It runs on a worker thread so the socket is drained the
  // instant data arrives, regardless of GUI/GL load. Created without a parent so
  // it can move to the worker; deleted when the worker finishes (a cross-thread
  // ~QObject from MainWindow's dtor would crash).
  m_engine = new TelemetryEngine();
  m_worker = new QThread(this);
  m_worker->setObjectName("telemetry");
  m_engine->moveToThread(m_worker);
  m_worker->start();

  // If another subsystem (the in-sim RC bridge) claims our serial port, drop
  // the board connection so we never contend for the same tty.
  connect(&PortArbiter::instance(), &PortArbiter::revoked, this,
          [this](const QString &, QObject *owner) {
            if (owner == this && m_serialOpen) {
              m_logPanel->appendLog(
                  "[GCS] Port taken by the simulator RC — disconnected.");
              QMetaObject::invokeMethod(m_engine, "closeLinks",
                                        Qt::QueuedConnection);
            }
          });
  m_uiTimer = new QTimer(this);
  m_syncTimer = new QTimer(this);
  m_elapsed.start();

  m_stackedWidget = new QStackedWidget(this);
  setCentralWidget(m_stackedWidget);

  // Command layer: built before the menu bar / shortcuts so both draw their
  // QActions from it (Phase-1 1g / FR-UX-19).
  m_cmds = new CommandRegistry(this);

  // Telemetry-source state machine (gcs-source-state-machine.md): the single
  // authority for tx-gating, the engine feed, and the toolbar/pill. In Replay
  // the toolbar goes read-only; the serial controls are usable only in Idle/Fc.
  buildSourceHooks();
  connect(&m_source, &SourceController::stateChanged, this,
          [this](SourceState now, SourceState) {
            if (m_toolbar) {
              m_toolbar->setReplayMode(now == SourceState::Replay);
              m_toolbar->setSerialControlsEnabled(now == SourceState::Idle ||
                                                  now == SourceState::Fc);
            }
            refreshConnectionPill();
            updateExportEnabled();
          });

  // Engine link signals — forwarded from the (worker-thread) transports, so
  // these arrive auto-queued on the GUI thread. The live byte→parser path is
  // entirely inside the engine now (serial+UDP drained on the worker).
  connect(m_engine, &TelemetryEngine::connectionStateChanged, this,
          [this](bool connected, bool isUdp) {
            if (isUdp) m_udpOpen = connected;
            else m_serialOpen = connected;
            onConnectionStateChanged(m_serialOpen || m_udpOpen);
          });
  connect(m_engine, &TelemetryEngine::errorOccurred, this,
          [this](const QString &m, bool isUdp) {
            if (isUdp) m_logPanel->appendLog("[UDP] " + m);
            else onSerialError(m);
          });
  // Async result of openSerial/bindUdp (open() can't return across the thread
  // hop): persist the good setting; release the PortArbiter on serial failure.
  connect(m_engine, &TelemetryEngine::linkOpened, this,
          [this](bool ok, const QString &label, bool isUdp) {
            if (ok) {
              persistPortBaud();
              if (isUdp) m_logPanel->appendLog("[UDP] listening on " + label);
            } else {
              // The optimistic Fc transition didn't actually open — demote to
              // Idle (teardownFc releases the PortArbiter + closes the links).
              runTransition([this] { m_source.forceIdle(); });
            }
          });
  // Live-export state → flip the menu label + log (engine runs it on the worker).
  connect(m_engine, &TelemetryEngine::exportStateChanged, this,
          [this](bool active, const QString &path) {
            m_exportActive = active;
            updateExportEnabled();
            if (m_exportAction)
              m_exportAction->setText(active ? tr("Stop Log &Export")
                                             : tr("&Export Log…"));
            m_logPanel->appendLog(active ? "[GCS] Exporting telemetry → " + path
                                         : "[GCS] Export stopped");
            // The engine has opened the file by now — an exported .bin is a
            // replayable log, so add it to the recent list.
            if (active) addRecentLog(path);
          });

  // Low-rate / event-like protocol signals stay wired directly to the GUI (they
  // are auto-queued across the thread boundary). Per-packet telemetry is consumed
  // inside the engine and surfaced via snapshot() at the render rate — onUiTimer.
  DroneProtocol *proto = m_engine->protocol();
  connect(proto, &DroneProtocol::logReceived, this,
          &MainWindow::onLogReceived);
  connect(proto, &DroneProtocol::unknownPacket, this,
          [this](const QByteArray &raw) {
            onLogReceived(QString("[raw] ") + QString::fromLatin1(raw));
          });
  connect(proto, &DroneProtocol::heartbeatReceived, this,
          &MainWindow::onHeartbeatReceived);
  connect(proto, &DroneProtocol::timeSyncResponse, this,
          &MainWindow::onTimeSyncResponse);
  connect(proto, &DroneProtocol::timeSyncRequested, this,
          &MainWindow::onTimeSyncRequested);
  // HSL_STATUS arrives at 1 Hz -- low-rate and event-like, so it goes straight
  // to the status bar rather than through the engine snapshot.
  connect(proto, &DroneProtocol::hslStatusReceived, this,
          [this](const HslStatusData &d) {
            if (m_statusBar)
              m_statusBar->setHslStatus(d);
          });

  m_rcWidget = new RcChannelsWidget(this);
  m_stackedWidget->addWidget(m_rcWidget);

  connect(m_rcWidget, &RcChannelsWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Periodic status refresh at 20 Hz for smoother fade
  connect(m_uiTimer, &QTimer::timeout, this, &MainWindow::onUiTimer);
  m_uiTimer->start(33);  // ~30 Hz UI render pump (decoupled from packet rate)

  connect(m_syncTimer, &QTimer::timeout, this,
          &MainWindow::onTimeSyncRequested);

  buildUi(); // Built as m_homeWidget

  // Build Packet Analyzer
  m_analyzerWidget = new PacketAnalyzerWidget(this);
  m_analyzerWidget->setProtocol(m_engine->protocol());
  m_stackedWidget->addWidget(m_analyzerWidget);

  // Wire Protocol and Serial to Analyzer
  connect(m_engine->protocol(), &DroneProtocol::packetReceived, m_analyzerWidget,
          &PacketAnalyzerWidget::logRxPacket);
  connect(m_engine->protocol(), &DroneProtocol::unknownPacket, m_analyzerWidget,
          &PacketAnalyzerWidget::logRxPacket);
  // TX frames for the analyzer come from the engine (serial+UDP dataSent are
  // forwarded as txPacket from the worker thread → auto-queued here).
  connect(m_engine, &TelemetryEngine::txPacket, m_analyzerWidget,
          &PacketAnalyzerWidget::logTxPacket);
  connect(m_analyzerWidget, &PacketAnalyzerWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Settings
  m_settingsWidget = new SettingsWidget(this);
  m_stackedWidget->addWidget(m_settingsWidget);
  connect(m_settingsWidget, &SettingsWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  // Settings are edited in a staged form; Apply commits the whole set and
  // persists once. applyAllSettings(true) does both.
  connect(m_settingsWidget, &SettingsWidget::applyRequested, this,
          [this] { applyAllSettings(true); });

  // The recorder now lives inside the engine and tees live bytes on the worker
  // thread (Phase B), so no GUI-side byte tee is needed.

  // Load and apply persistent settings (apply without re-saving).
  GcsSettings savedSettings;
  if (SettingsManager::load(savedSettings)) {
    m_settingsWidget->setSettings(savedSettings);
    applyAllSettings(false);
  }

  buildMenuBar();

  // Toolbar + status bar are pulled out into their own classes
  // (Phase-1 1a). MainWindow only wires their intent signals to
  // serial/protocol side-effects.
  m_toolbar = new MainToolbar(this);
  addToolBar(m_toolbar);
  m_statusBar = new MainStatusBar(this);
  setStatusBar(m_statusBar);

  connect(m_toolbar, &MainToolbar::connectRequested, this,
          &MainWindow::onConnectRequested);
  connect(m_toolbar, &MainToolbar::disconnectRequested, this,
          &MainWindow::onDisconnectRequested);
  connect(m_toolbar, &MainToolbar::armClicked, this, &MainWindow::onArmClicked);

  m_toolbar->refreshPorts();  // populate port list on startup
  setConnected(false);
  // Build Calibration
  m_calibrationWidget = new CalibrationWidget(this);
  m_calibrationWidget->setProtocol(m_engine->protocol());
  m_stackedWidget->addWidget(m_calibrationWidget);
  connect(m_calibrationWidget, &CalibrationWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_calibrationWidget, &CalibrationWidget::commandRequested,
          [this](const QByteArray &data) {
            if (m_source.txAllowed())
              sendToFc(data);
          });

  // Build Motor Status
  m_motorWidget = new MotorStatusWidget(this);
  m_stackedWidget->addWidget(m_motorWidget);
  connect(m_motorWidget, &MotorStatusWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Control Loop Plot
  m_controlLoopWidget = new ControlLoopPlot(this);
  m_controlLoopWidget->setProtocol(m_engine->protocol());
  m_stackedWidget->addWidget(m_controlLoopWidget);
  connect(m_controlLoopWidget, &ControlLoopPlot::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Gyro Notch tuning
  m_gyroNotchWidget = new GyroNotchWidget(this);
  m_gyroNotchWidget->setProtocol(m_engine->protocol());
  m_stackedWidget->addWidget(m_gyroNotchWidget);
  connect(m_gyroNotchWidget, &GyroNotchWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_gyroNotchWidget, &GyroNotchWidget::commandRequested,
          [this](const QByteArray &data) {
            if (m_source.txAllowed())
              sendToFc(data);
          });

  // Build Kernel Observability (vaios perf telemetry)
  m_perfWidget = new PerfWidget(this);
  m_stackedWidget->addWidget(m_perfWidget);
  connect(m_perfWidget, &PerfWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_engine->protocol(), &DroneProtocol::perfReceived, m_perfWidget,
          &PerfWidget::updateReport);
  // On-demand task-name resolution: widget asks, FC replies, widget caches.
  connect(m_engine->protocol(), &DroneProtocol::taskNameReceived, m_perfWidget,
          &PerfWidget::setTaskName);
  connect(m_perfWidget, &PerfWidget::requestTaskName, this,
          &MainWindow::sendTaskNameRequest);

#ifdef NAVIGATOR_HAS_SITL
  // Build Simulator (in-app SITL: physics + sensors + firmware threads).
  // Only present when sim_host was found at configure time — see the
  // CMake guard around vayu_sitl_core. When omitted, the Simulator menu
  // entry below is suppressed and Ctrl+8 has no target.
  m_simulatorWidget = new SimulatorWidget(this);
  m_stackedWidget->addWidget(m_simulatorWidget);
  connect(m_simulatorWidget, &SimulatorWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  // Pipe the firmware's UART2 byte stream into the same DroneProtocol
  // parser the real serial path feeds. This means every telemetry panel
  // (IMU, attitude, log, motor, control loop, ...) lights up off the
  // in-app sim with no further per-widget plumbing. The signature
  // matches SerialManager::dataReceived so the receiver doesn't care
  // which source is feeding it.
  // Route the sim's UART2 byte stream through SimSource so the controller can
  // attach/detach it as the Sim state is entered/left (applyFeed) — same seam as
  // replay. The signature matches; SimSource just re-emits as an ITelemetrySource.
  connect(m_simulatorWidget, &SimulatorWidget::dataReceived, m_simSource,
          &SimSource::feed);
  // AT-1: the autotune tab proposes gains; applying to firmware is an explicit
  // operator action. Turn the proposal into CMD_SET_PID frames and send them
  // over the live link (sendToFc refuses in replay; we also require a link).
  connect(m_simulatorWidget, &SimulatorWidget::applyPidGainsRequested, this,
          [this](const QVector<PidSetCmd> &cmds) {
            // Two apply targets, picked by what's live:
            //   Fc        -> send CMD_SET_PID over the live link (real board).
            //   sim core  -> apply in-process to the in-app firmware, which
            //                persists to 0:pid.bin so the tune survives a sim
            //                restart (same path the real FC uses; no link).
            // A live FC link wins. Otherwise, if the in-process firmware has
            // been started this session (true even in the Idle window right
            // after an autotune run), apply there.
            const SourceState st = m_source.state();
            if (st == SourceState::Fc) {
              const uint32_t now =
                  static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
              for (const PidSetCmd &c : cmds)
                sendToFc(CommandCodec::encodeSetPid(c.controller, c.axis, c.kp,
                                                    c.ki, c.kd, c.kff, 42, now));
              m_logPanel->appendLog(
                  QString(
                      "[GCS] Applied %1 PID slot(s) to firmware (CMD_SET_PID)")
                      .arg(cmds.size()));
              Notify::ok(this, tr("Applied gains to firmware"));
              return;
            }
#ifdef NAVIGATOR_HAS_SITL
            if (m_simulatorWidget && m_simulatorWidget->sitlCoreStarted()) {
              // Build the v1 CMD_SET_PID payload the firmware expects and hand
              // it to the same apply+persist routine the link path triggers:
              //   [0..1] cmd_id LE | [2] argc(6) | [3..] 6 LE floats:
              //   ctrl, axis, kp, ki, kd, kff. Host is little-endian like the
              //   target FC, so a raw memcpy of the floats is wire-correct.
              int ok = 0;
              for (const PidSetCmd &c : cmds) {
                uint8_t buf[3 + 6 * 4];
                buf[0] = static_cast<uint8_t>(kCmdSetPid & 0xFF);
                buf[1] = static_cast<uint8_t>((kCmdSetPid >> 8) & 0xFF);
                buf[2] = 6;  // argc
                const float args[6] = {static_cast<float>(c.controller),
                                       static_cast<float>(c.axis),
                                       c.kp, c.ki, c.kd, c.kff};
                std::memcpy(&buf[3], args, sizeof args);
                if (pid_config_apply_command(buf, sizeof buf) == 0 /*VAYU_OK*/)
                  ++ok;
              }
              m_logPanel->appendLog(
                  QString("[GCS] Applied %1/%2 PID slot(s) to sim "
                          "(persisted to 0:pid.bin)")
                      .arg(ok)
                      .arg(cmds.size()));
              if (ok == cmds.size())
                Notify::ok(this, tr("Applied gains to sim (persisted)"));
              else
                Notify::warn(this, tr("Some sim gains were rejected"));
              return;
            }
#endif
            m_logPanel->appendLog(
                "[GCS] Apply gains ignored — connect an FC or start the sim");
            Notify::warn(this, tr("No FC link or sim — can't apply gains"));
          });
  // Reflect the firmware's reported flight mode (stabilise/acro + RC/GCS source)
  // back onto the simulator's Acro toggle.
  connect(m_engine->protocol(), &DroneProtocol::flightModeReceived,
          m_simulatorWidget, &SimulatorWidget::setFlightModeStatus);
  // The flight-mode pill itself is driven by the render clock from snapshot().
  // Reflect the in-app sim as "Connected: SIM" in the bottom status bar.
  // The sim Start/Stop buttons drive the widget directly; reconcile the FSM to
  // match (Sim when running, Idle when stopped). runTransition suppresses the
  // re-entrant simRunningChanged that a teardown-driven stop would fire. The
  // pill, export-enable, and serial-control lock follow from stateChanged.
  // Reconcile the button-driven sim into the FSM. State-guarded so a stale
  // "stopped" (e.g. fired by a teardown when we've already moved to another
  // state) can't wrongly demote that state to Idle.
  connect(m_simulatorWidget, &SimulatorWidget::simRunningChanged, this,
          [this](bool running) {
            if (running) {
              if (m_source.state() != SourceState::Sim)
                runTransition([this] { m_source.requestSim(); });
            } else if (m_source.state() == SourceState::Sim) {
              runTransition([this] { m_source.requestIdle(); });
            }
          });
  // Autotune is its own source state: an isolated tuner run that doesn't feed
  // the GCS engine (read-only, no pill feed). startAutotune already stopped the
  // interactive sim (→ simRunningChanged(false) → Idle) before this fires.
  connect(m_simulatorWidget, &SimulatorWidget::autotuneRunningChanged, this,
          [this](bool running) {
            if (running) {
              if (m_source.state() != SourceState::Autotune)
                runTransition([this] { m_source.requestAutotune(); });
            } else if (m_source.state() == SourceState::Autotune) {
              runTransition([this] { m_source.requestIdle(); });
            }
          });
#endif

  // Recent-views (MRU) switcher (FR-UX-21). Record every page
  // change centrally and build the index->label map the overlay displays.
  buildViewTitles();
  connect(m_stackedWidget, &QStackedWidget::currentChanged, this,
          [this](int idx) {
            if (idx >= 0) m_viewHistory.visit(idx);
          });
  m_recentOverlay = new RecentViewsOverlay(this);
  connect(m_recentOverlay, &RecentViewsOverlay::activated, this,
          [this](int idx) { m_stackedWidget->setCurrentIndex(idx); });

  // Replay transport bar (FR-LOG-05), docked at the bottom, shown only in
  // replay. Exit returns to the live session.
  m_replayBar = new ReplayBar(this);
  connect(m_replayBar, &ReplayBar::exitRequested, this,
          &MainWindow::exitReplay);
  m_replayToolbar = new QToolBar(tr("Replay"), this);
  // QMainWindow::saveState() needs a stable objectName for every toolbar/dock,
  // else it warns and can't restore this bar's position.
  m_replayToolbar->setObjectName(QStringLiteral("ReplayToolbar"));
  m_replayToolbar->setMovable(false);
  m_replayToolbar->addWidget(m_replayBar);
  addToolBar(Qt::BottomToolBarArea, m_replayToolbar);
  m_replayToolbar->setVisible(false);

  installShortcuts();

  // Editable shortcut overrides on top of the registry defaults (FR-UX-19–23).
  // Created after every command is registered, then load() applies any saved
  // overrides to the shared QActions.
  m_shortcuts = new ShortcutsManager(m_cmds, this);
  m_shortcuts->load();

  // Restore window geometry, splitter sizes, last page, port/baud.
  // Must run after every widget the state references has been built.
  restoreUiState();

  // restoreState() above can bring back the replay toolbar's visibility if the
  // app was last closed mid-replay — but no ReplaySource exists at launch, so
  // that would be an orphaned, undismissable bar. Replay is never active at
  // startup; force it hidden.
  if (m_replayToolbar) m_replayToolbar->setVisible(false);

  // Apply the explicit Link & Connection launch defaults on top of the
  // restored last-used port/baud (an explicit default wins; an empty default
  // port means "Ask each time" → keep whatever restoreUiState picked).
  if (m_toolbar) {
    m_toolbar->setTransport(savedSettings.defaultTransport);
    if (savedSettings.defaultTransport == 1) {  // UDP: defaultPort is a number
      if (!savedSettings.defaultPort.isEmpty())
        m_toolbar->setUdpPort(savedSettings.defaultPort.toInt());
    } else {  // Serial: defaultPort is a device path; baud applies
      if (!savedSettings.defaultPort.isEmpty())
        m_toolbar->setPort(savedSettings.defaultPort);
      m_toolbar->setBaud(savedSettings.defaultBaud);
    }
  }

  // If restoreUiState didn't put us on a page (first run), default to home.
  if (m_stackedWidget->currentWidget() == nullptr) showHome();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  saveUiState();

  // Tear down the worker thread before the engine (and its transports/recorder)
  // are destroyed. forceIdle first so any GUI-side source (replay/sim) stops.
  m_source.forceIdle();
  if (m_worker) {
    m_worker->quit();
    m_worker->wait();
  }

  QMainWindow::closeEvent(event);
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void MainWindow::showHome() { m_stackedWidget->setCurrentWidget(m_homeWidget); }

void MainWindow::showPacketAnalyzer() {
  m_stackedWidget->setCurrentWidget(m_analyzerWidget);
}

void MainWindow::showRcMonitor() {
  m_stackedWidget->setCurrentWidget(m_rcWidget);
}

void MainWindow::showSettings() {
  m_stackedWidget->setCurrentWidget(m_settingsWidget);
}

void MainWindow::showCalibration() {
  m_stackedWidget->setCurrentWidget(m_calibrationWidget);
}

void MainWindow::showMotorStatus() {
  m_stackedWidget->setCurrentWidget(m_motorWidget);
}

void MainWindow::showGyroNotch() {
  m_stackedWidget->setCurrentWidget(m_gyroNotchWidget);
}

void MainWindow::showControlLoopPlot() {
  m_stackedWidget->setCurrentWidget(m_controlLoopWidget);
}

void MainWindow::showPerf() {
  m_stackedWidget->setCurrentWidget(m_perfWidget);
}

void MainWindow::sendTaskNameRequest(int taskId) {
  if (taskId < 0 || taskId > 255)
    return;
  // Background request (the Perf page resolves task names on its own). Skip it
  // silently when tx isn't allowed — e.g. replaying a recording, where it would
  // otherwise spam "Read-only in replay" once per unknown task.
  if (!m_source.txAllowed())
    return;
  sendToFc(CommandCodec::encodeTaskNameRequest(taskId));
}

void MainWindow::showSimulator() {
#ifdef NAVIGATOR_HAS_SITL
  if (m_simulatorWidget) m_stackedWidget->setCurrentWidget(m_simulatorWidget);
#endif
}

// ---------------------------------------------------------------------------
// UI Construction
// ---------------------------------------------------------------------------

void MainWindow::buildUi() {
  m_homeWidget = new QWidget(this);
  auto *rootVBox = new QVBoxLayout(m_homeWidget);
  rootVBox->setSpacing(8);
  rootVBox->setContentsMargins(8, 8, 8, 8);

  // ---- Top row: attitude + attitude labels + IMU panel ----
  m_topSplitter = new QSplitter(Qt::Horizontal, m_homeWidget);
  m_topSplitter->setObjectName("HomeTopSplitter");
  auto *topSplitter = m_topSplitter;

  // ---- Attitude group ----
  auto *attGroup = new QGroupBox("Attitude", m_homeWidget);
  auto *attLayout = new QVBoxLayout(attGroup);

  // Header: 2D / 3D segmented toggle at the top right (mockup .tt).
  auto *headerLayout = new QHBoxLayout();
  headerLayout->addStretch();
  auto *btn2d = new QPushButton("2D", attGroup);
  auto *btn3d = new QPushButton("3D", attGroup);
  auto *viewGroup = new QButtonGroup(this);
  viewGroup->setExclusive(true);
  for (auto *b : {btn2d, btn3d}) {
    b->setObjectName("ToggleButton");
    b->setCheckable(true);
    // Fixed height only; let the width follow the text + padding so the label
    // is never clipped (was setFixedSize, which cropped "2D"/"3D").
    b->setFixedHeight(22);
    b->setMinimumWidth(38);
    b->setCursor(Qt::PointingHandCursor);
  }
  btn2d->setChecked(true);
  btn2d->setToolTip(tr("2D HUD"));
  btn3d->setToolTip(tr("3D airframe view"));
  viewGroup->addButton(btn2d, 0);
  viewGroup->addButton(btn3d, 1);
  connect(viewGroup, &QButtonGroup::idClicked, this,
          [this](int id) { onToggle3d(id == 1); });
  headerLayout->addWidget(btn2d);
  headerLayout->addWidget(btn3d);
  attLayout->addLayout(headerLayout);

  // Firmware system-state pill — ABOVE the ADI (mockup .state-bar). Shows
  // ARMED / STANDBY / FAILSAFE as status packets arrive; starts muted so it
  // doesn't compete with the status-bar connection pill (FR-UX-05).
  m_statusLabel = new QLabel("—", attGroup);
  m_statusLabel->setAlignment(Qt::AlignCenter);
  m_statusLabel->setStyleSheet(
      QString("font-size: 16px; font-weight: bold; color: %1; "
              "background: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 4px; margin-bottom: 6px;")
          .arg(Theme::hex(Theme::kTextDim),
               Theme::hex(Theme::kBg),
               Theme::hex(Theme::kBorder)));
  attLayout->addWidget(m_statusLabel);

  m_attStack = new QStackedWidget(attGroup);
  m_attitude = new AttitudeWidget(m_attStack);
  m_drone3d = new Drone3DWidget(m_attStack);
  m_attStack->addWidget(m_attitude);
  m_attStack->addWidget(m_drone3d);
  attLayout->addWidget(m_attStack, 1);

  // Flight-mode pill: STABILISE / ACRO + source (RC switch or GCS override),
  // driven by SYSTEM_ORIGIN_FLIGHT_MODE telemetry just like the state pill above.
  m_flightModeLabel = new QLabel("—", attGroup);
  m_flightModeLabel->setAlignment(Qt::AlignCenter);
  m_flightModeLabel->setStyleSheet(
      QString("font-size: 13px; font-weight: bold; color: %1; "
              "background: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 3px; margin-bottom: 8px;")
          .arg(Theme::hex(Theme::kTextDim),
               Theme::hex(Theme::kBg),
               Theme::hex(Theme::kBorder)));
  attLayout->addWidget(m_flightModeLabel);

  // Numeric roll/pitch/yaw labels in a horizontal line
  auto *numHBox = new QHBoxLayout;
  numHBox->setContentsMargins(10, 0, 10, 5);
  numHBox->setSpacing(20);

  const char *lblNames[] = {"Roll", "Pitch", "Yaw"};
  QLabel **lblPtrs[] = {&m_rollLabel, &m_pitchLabel, &m_yawLabel};
  const char *colors[] = {"#FF6B6B", "#4ECDC4", "#FFE66D"};

  for (int i = 0; i < 3; ++i) {
    auto *container = new QWidget(attGroup);
    auto *vbox = new QVBoxLayout(container);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(2);

    auto *title = new QLabel(QString("<b>%1</b>").arg(lblNames[i]), container);
    title->setStyleSheet(QString("color: %1; font-size: 11px;").arg(colors[i]));
    title->setAlignment(Qt::AlignCenter);

    *lblPtrs[i] = new QLabel("0.00°", container);
    (*lblPtrs[i])
        ->setStyleSheet(
            QString("color: %1; font-size: 16px; font-weight: bold; "
                    "font-family: Monospace;")
                .arg(colors[i]));
    (*lblPtrs[i])->setAlignment(Qt::AlignCenter);

    QLabel **stdPtrs[] = {&m_rollStd, &m_pitchStd, &m_yawStd};
    *stdPtrs[i] = new QLabel("± σ 0.000", container);
    (*stdPtrs[i])
        ->setStyleSheet(QString(
            "color: #888888; font-size: 10px; font-family: Monospace;"));
    (*stdPtrs[i])->setAlignment(Qt::AlignCenter);

    vbox->addWidget(title);
    vbox->addWidget(*lblPtrs[i]);
    vbox->addWidget(*stdPtrs[i]);
    // Equal-stretch columns: each label is centred in its own third, so the
    // three stay evenly spread + column-aligned as the panel is resized.
    numHBox->addWidget(container, 1);
  }
  attLayout->addLayout(numHBox);
  topSplitter->addWidget(attGroup);

  // ---- IMU Panel ----
  m_imuPanel = new ImuPanel(m_homeWidget);
  topSplitter->addWidget(m_imuPanel);

  topSplitter->setStretchFactor(0, 1);
  topSplitter->setStretchFactor(1, 2);

  // ---- Bottom: Log panel ----
  m_logPanel = new LogPanel(m_homeWidget);
  m_logPanel->setMinimumHeight(180);

  // ---- Vertical splitter: attitude+imu / log ----
  m_vSplitter = new QSplitter(Qt::Vertical, m_homeWidget);
  m_vSplitter->setObjectName("HomeVSplitter");
  m_vSplitter->addWidget(topSplitter);
  m_vSplitter->addWidget(m_logPanel);
  m_vSplitter->setStretchFactor(0, 3);
  m_vSplitter->setStretchFactor(1, 1);

  rootVBox->addWidget(m_vSplitter);
  m_stackedWidget->addWidget(m_homeWidget);

  // Status bar widgets are owned by MainStatusBar, constructed in the
  // ctor after the home page is built.
}

void MainWindow::buildMenuBar() {
  QMenuBar *menu = menuBar();

  // Mockup parity: File / View / Flight / Tools / Window / Help menus, plus a
  // bare "Settings" menu-bar action that opens the page directly. Every item is
  // a registered command (FR-UX-19): the registry owns the QAction and adding it
  // to a menu also activates its shortcut, so a command is hosted in exactly one
  // place (never re-added to the window).

  // ---- File ----
  QMenu *fileMenu = menu->addMenu("&File");
  fileMenu->addAction(m_cmds->add(
      "link.toggle", "&Connect / Disconnect", "Link", QKeySequence("Ctrl+K"),
      CmdContext::Always,
      [this] { if (m_toolbar) m_toolbar->onConnectClicked(); }));
  fileMenu->addSeparator();
  fileMenu->addAction(m_cmds->add(
      "replay.open", "&Open Flight Log…", "Replay", QKeySequence("Ctrl+O"),
      CmdContext::Always, [this] {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Open Telemetry Log"),
            QDir::home().filePath("vayu-logs"),
            tr("Telemetry recordings (*.bin)"));
        if (!path.isEmpty()) enterReplay(path);
      }));
  // Open Recent Log — MRU of the last logs opened or recorded. Rebuilt every
  // time it's shown so it reflects new recordings and prunes deleted files.
  m_recentLogsMenu = fileMenu->addMenu(tr("Open &Recent Log"));
  m_recentLogsMenu->setToolTipsVisible(true);  // show full path on hover
  connect(m_recentLogsMenu, &QMenu::aboutToShow, this,
          &MainWindow::rebuildRecentLogsMenu);
  rebuildRecentLogsMenu();
  // Jump straight to the most recent log (opened, recorded, or exported).
  fileMenu->addAction(m_cmds->add(
      "replay.openLast", "Open &Last Log", "Replay",
      QKeySequence("Ctrl+Shift+O"), CmdContext::Always,
      [this] { openLastRecentLog(); }));
  // Live, packet-type-filtered telemetry export (toggles start/stop). The action
  // text flips to "Stop Log Export" while running (engine.exportStateChanged).
  m_exportAction = m_cmds->add("log.export", "&Export Log…", "Log",
                               QKeySequence("Ctrl+E"), CmdContext::Always,
                               [this] { onExportLogToggle(); });
  fileMenu->addAction(m_exportAction);
  updateExportEnabled();  // disabled until a source is feeding
  fileMenu->addSeparator();
  fileMenu->addAction(m_cmds->add("app.exit", "E&xit", "Application",
                                  QKeySequence(QKeySequence::Quit),
                                  CmdContext::Always, [this] { close(); }));

  // ---- View (page navigation, Ctrl+1‑8 per the mockup) ----
  // Helper: register a command, give it the mockup's line-icon, and add it.
  auto addNav = [&](QMenu *m, const QString &id, const QString &text,
                    const QKeySequence &key, ui::Icon icon,
                    std::function<void()> fn) {
    QAction *a = m_cmds->add(id, text, "View", key, CmdContext::Always,
                             std::move(fn));
    a->setIcon(ui::svgIcon(icon));
    m->addAction(a);
    return a;
  };

  QMenu *viewMenu = menu->addMenu("&View");
  addNav(viewMenu, "nav.dashboard", "Flight &Dashboard", QKeySequence("Ctrl+1"),
         ui::Icon::Dashboard, [this] { showHome(); });
  addNav(viewMenu, "nav.rc", "&RC Channels", QKeySequence("Ctrl+2"),
         ui::Icon::Rc, [this] { showRcMonitor(); });
  addNav(viewMenu, "nav.motors", "&Motors", QKeySequence("Ctrl+3"),
         ui::Icon::Motors, [this] { showMotorStatus(); });
  addNav(viewMenu, "nav.control", "&Control Loop", QKeySequence("Ctrl+4"),
         ui::Icon::Control, [this] { showControlLoopPlot(); });
  viewMenu->addSeparator();
  addNav(viewMenu, "nav.packets", "&Packet Analyzer", QKeySequence("Ctrl+5"),
         ui::Icon::Packets, [this] { showPacketAnalyzer(); });
#ifdef NAVIGATOR_HAS_SITL
  addNav(viewMenu, "nav.simulator", "Si&mulator", QKeySequence("Ctrl+6"),
         ui::Icon::Sim, [this] { showSimulator(); });
#endif
  addNav(viewMenu, "nav.perf", "&Kernel Perf", QKeySequence("Ctrl+8"),
         ui::Icon::Perf, [this] { showPerf(); });
  viewMenu->addSeparator();
  viewMenu->addAction(m_cmds->add(
      "window.fullscreen", "&Fullscreen", "Window", QKeySequence(Qt::Key_F11),
      CmdContext::Always, [this] {
        if (isFullScreen()) showNormal();
        else showFullScreen();
      }));

  // ---- Flight ----
  QMenu *flightMenu = menu->addMenu("F&light");
  {
    QAction *arm =
        m_cmds->add("flight.arm", "&Arm / Disarm", "Flight",
                    QKeySequence("Ctrl+A"), CmdContext::Always,
                    [this] { onArmClicked(); });
    arm->setIcon(ui::svgIcon(ui::Icon::Arm));
    flightMenu->addAction(arm);
  }
  flightMenu->addSeparator();
  flightMenu->addAction(m_cmds->add("flight.runCalib", "&Run Calibration…",
                                    "Flight", QKeySequence(), CmdContext::Always,
                                    [this] { showCalibration(); }));

  // ---- Tools ----
  QMenu *toolsMenu = menu->addMenu("&Tools");
  addNav(toolsMenu, "nav.calibration", "&Calibration", QKeySequence("Ctrl+7"),
         ui::Icon::Calib, [this] { showCalibration(); });
  addNav(toolsMenu, "nav.gyronotch", "&Gyro Notch", QKeySequence("Ctrl+9"),
         ui::Icon::Control, [this] { showGyroNotch(); });

  // ---- Settings (a bare menu-bar action — clicking it opens the page
  // directly, no submenu) ----
  menu->addAction(m_cmds->add("view.settings", "&Settings", "View",
                              QKeySequence(), CmdContext::Always,
                              [this] { showSettings(); }));

  // ---- Window ----
  QMenu *windowMenu = menu->addMenu("&Window");
  windowMenu->addAction(m_cmds->add("window.resetLayout", "&Reset Layout",
                                    "Window", QKeySequence(), CmdContext::Always,
                                    [this] { resetLayout(); }));

  // ---- Help ----
  QMenu *helpMenu = menu->addMenu("&Help");
  helpMenu->addAction(m_cmds->add(
      "help.docs", "&Documentation", "Help", QKeySequence(Qt::Key_F1),
      CmdContext::Always, [this] {
        const QString docs = AboutDialog::docsPath();
        if (docs.isEmpty())
          Notify::warn(this, tr("Bundled documentation not found"));
        else
          QDesktopServices::openUrl(QUrl::fromLocalFile(docs));
      }));
  helpMenu->addAction(m_cmds->add(
      "help.shortcuts", "&Keyboard Shortcuts…", "Help",
      QKeySequence("Ctrl+Alt+K"), CmdContext::Always, [this] {
        ShortcutsEditorDialog dlg(m_cmds, m_shortcuts, this);
        dlg.exec();
      }));
  helpMenu->addSeparator();
  helpMenu->addAction(m_cmds->add(
      "help.about", "&About Navigator", "Help", QKeySequence(),
      CmdContext::Always, [this] {
        AboutDialog dlg(this);
        dlg.exec();
      }));
}

void MainWindow::resetLayout() {
  // Restore the dashboard splitters to their default proportions (attitude:imu
  // = 1:2, panels:log = 3:1) and return to the dashboard. Mirrors the mockup's
  // Window ▸ Reset Layout.
  if (m_topSplitter) {
    const int w = m_topSplitter->width();
    m_topSplitter->setSizes({w / 3, w - w / 3});
  }
  if (m_vSplitter) {
    const int h = m_vSplitter->height();
    m_vSplitter->setSizes({3 * h / 4, h - 3 * h / 4});
  }
  showHome();
  Notify::info(this, tr("Layout reset"));
}

// ---------------------------------------------------------------------------
// Toolbar intent slots — toolbar emits, MainWindow drives serial.
// ---------------------------------------------------------------------------

void MainWindow::onConnectRequested(const QString &port, int baud) {
  // Route through the FSM: Fc setup (buildSourceHooks) tears down any other
  // source and opens the link. Connecting from replay just transitions out.
  runTransition([&] { m_source.requestFc(port, baud); });
}

void MainWindow::onDisconnectRequested() {
  runTransition([&] { m_source.requestIdle(); });
}

void MainWindow::enterReplay(const QString &path) {
  runTransition([&] { m_source.requestReplay(path); });
}

void MainWindow::exitReplay() {
  runTransition([&] { m_source.requestIdle(); });
  // Safety for an orphaned bar (restored visible with no source): the Replay
  // teardown hook hides it, but requestIdle from a non-Replay state won't run it.
  if (m_replayToolbar) m_replayToolbar->setVisible(false);
}

void MainWindow::sendToFc(const QByteArray &pkt) {
  // Single tx choke point — only Fc/Sim are commandable; Replay/Idle/Autotune
  // are read-only, so one guard covers ARM / PID / calibrate / time-sync.
  if (!m_source.txAllowed()) {
    Notify::warn(this, tr("Read-only — not connected"));
    return;
  }
  // The transports live on the worker thread; route the write there. The engine
  // picks the active transport (UDP-else-serial).
  QMetaObject::invokeMethod(m_engine, "send", Qt::QueuedConnection,
                            Q_ARG(QByteArray, pkt));
}

void MainWindow::runTransition(const std::function<void()> &body) {
  // A deliberate transition's teardown may stop the sim, which fires
  // simRunningChanged → a reconcile request; suppress that re-entry.
  if (m_fsmBusy) return;
  m_fsmBusy = true;
  body();
  m_fsmBusy = false;
}

void MainWindow::applyFeed(SourceState s) {
  // Point the parser at the new state's source: detach the previously-wired
  // source, attach the new one (GUI-thread connect, like the old replay path),
  // and enable the live transport feed only for Fc.
  ITelemetrySource *want = nullptr;
  if (s == SourceState::Replay) want = m_replaySource;
  else if (s == SourceState::Sim) want = m_simSource;
  if (want != m_activeFeedSource) {
    if (m_activeFeedSource)
      disconnect(m_activeFeedSource, &ITelemetrySource::bytesReceived, m_engine,
                 &TelemetryEngine::feedBytes);
    m_activeFeedSource = want;
    if (m_activeFeedSource)
      connect(m_activeFeedSource, &ITelemetrySource::bytesReceived, m_engine,
              &TelemetryEngine::feedBytes);
  }
  QMetaObject::invokeMethod(m_engine, "setLiveFeed", Qt::QueuedConnection,
                            Q_ARG(bool, s == SourceState::Fc));
}

void MainWindow::buildSourceHooks() {
  m_simSource = new SimSource(this);

  SourceController::Hooks h;

  // ---- Fc (live serial / UDP). Open is async; setup returns optimistically and
  //      engine.linkOpened(false) demotes back to Idle (see the ctor). ----
  h.setupFc = [this](const QString &port, int baud) -> bool {
    if (port.isEmpty()) {
      m_logPanel->appendLog("[GCS] No port specified");
      return false;
    }
    if (port.startsWith("udp", Qt::CaseInsensitive)) {
      quint16 udpPort = 14555;  // "udp" / "udp:<port>" — ESP8266 WiFi bridge
      const int colon = port.indexOf(':');
      if (colon >= 0) {
        const quint16 p = port.mid(colon + 1).toUShort();
        if (p) udpPort = p;
      }
      m_currentPort = QStringLiteral("udp:%1").arg(udpPort);
      QMetaObject::invokeMethod(m_engine, "bindUdp", Qt::QueuedConnection,
                                Q_ARG(int, int(udpPort)));
      return true;
    }
    // Claim the port (revokes the in-sim RC bridge if it holds the same tty).
    PortArbiter::instance().acquire(port, this);
    m_currentPort = port;
    QMetaObject::invokeMethod(m_engine, "openSerial", Qt::QueuedConnection,
                              Q_ARG(QString, port), Q_ARG(int, baud));
    return true;
  };
  h.teardownFc = [this] {
    if (!m_currentPort.isEmpty())
      PortArbiter::instance().release(m_currentPort, this);
    QMetaObject::invokeMethod(m_engine, "closeLinks", Qt::QueuedConnection);
  };

  // ---- Replay ----
  h.setupReplay = [this](const QString &path) -> bool {
    auto *rs = new ReplaySource(this);
    if (!rs->open(path)) {
      Notify::error(this, tr("Could not open recording: %1").arg(path));
      rs->deleteLater();
      return false;
    }
    stopRecording();  // a live recording must not capture replayed frames
    m_replaySource = rs;
    m_replayBar->bind(rs);
    m_replayBar->setLogName(QFileInfo(path).fileName());
    m_replayToolbar->setVisible(true);
    m_logPanel->appendLog("[GCS] Replay: " + path);
    addRecentLog(path);
    return true;  // applyFeed(Replay) wires rs → feedBytes
  };
  h.teardownReplay = [this] {
    if (m_replaySource) {
      m_replaySource->pause();
      m_replayBar->bind(nullptr);
      m_replaySource->deleteLater();
      m_replaySource = nullptr;
    }
    if (m_replayToolbar) m_replayToolbar->setVisible(false);
    m_logPanel->appendLog("[GCS] Exited replay — live");
  };

  // Sim/Autotune are started by their buttons and reconciled into the FSM via
  // sim/autotuneRunningChanged; the teardowns enforce strict single source —
  // entering any other state stops the sim daemon / cancels the tuner. Both are
  // idempotent; the re-entrant *RunningChanged they fire is suppressed by
  // runTransition (m_fsmBusy) and the state-guarded reconcile handlers.
#ifdef NAVIGATOR_HAS_SITL
  h.teardownSim = [this] {
    if (m_simulatorWidget) m_simulatorWidget->stopInAppSim();
  };
  h.teardownAutotune = [this] {
    if (m_simulatorWidget) m_simulatorWidget->stopAutotune();
  };
#endif
  h.applyFeed = [this](SourceState s) { applyFeed(s); };
  m_source.setHooks(std::move(h));
}

void MainWindow::onExportLogToggle() {
  // Already exporting → stop (the menu label is in the "Stop" state).
  if (m_exportActive) {
    QMetaObject::invokeMethod(m_engine, "stopExport", Qt::QueuedConnection);
    return;
  }

  // Stream picker: each row is a packet-type group; ticking it keeps those
  // packet types in the exported (replayable) .bin. Defaults to everything.
  struct Stream { const char *label; int mask; };
  static const Stream kStreams[] = {
      {"IMU (accel / gyro / mag)",   (1 << 0x1) | (1 << 0x2)},
      {"Attitude",                   (1 << 0x4)},
      {"RC channels",                (1 << 0x5)},
      {"Motors",                     (1 << 0x8)},
      {"System status / control",    (1 << 0x6)},
      {"Log messages",               (1 << 0x7)},
      {"Heartbeat",                  (1 << 0x0)},
      {"Perf / kernel stats",        (1 << 0x9) | (1 << 0xA)},
  };

  QDialog dlg(this);
  dlg.setWindowTitle(tr("Export telemetry"));
  auto *lay = new QVBoxLayout(&dlg);
  lay->addWidget(
      new QLabel(tr("Select the telemetry streams to export:"), &dlg));
  QList<QCheckBox *> boxes;
  for (const auto &s : kStreams) {
    auto *cb = new QCheckBox(tr(s.label), &dlg);
    cb->setChecked(true);
    lay->addWidget(cb);
    boxes.append(cb);
  }
  auto *bb = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  bb->button(QDialogButtonBox::Ok)->setText(tr("Start Export"));
  lay->addWidget(bb);
  connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted) return;

  int mask = 0;
  for (int i = 0; i < boxes.size(); ++i)
    if (boxes[i]->isChecked()) mask |= kStreams[i].mask;
  if (mask == 0) {
    Notify::warn(this, tr("Select at least one stream to export"));
    return;
  }

  // Destination file: a Save dialog pre-filled with the default name inside
  // ~/vayu-logs, so the user can keep the auto name or rename/relocate it.
  QDir logDir(QDir::home().filePath("vayu-logs"));
  if (!logDir.exists()) logDir.mkpath(".");
  const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
  const QString defaultPath =
      logDir.filePath(QStringLiteral("export-%1.bin").arg(stamp));
  QString path = QFileDialog::getSaveFileName(
      this, tr("Export telemetry to"), defaultPath,
      tr("Telemetry recordings (*.bin)"));
  if (path.isEmpty()) return;
  if (!path.endsWith(".bin", Qt::CaseInsensitive)) path += ".bin";

  QMetaObject::invokeMethod(m_engine, "startExport", Qt::QueuedConnection,
                            Q_ARG(QString, path), Q_ARG(int, mask));
  // The recent-list entry is added when the engine confirms the file is open
  // (exportStateChanged), avoiding a race with the menu's missing-file prune.
}

void MainWindow::onArmClicked() {
  if (!m_source.txAllowed()) {
    m_logPanel->appendLog("[GCS] ARM/DISARM ignored — not connected");
    return;
  }

  // Confirm before ARM (Settings ▸ Alerts & Audio). Gate arming only — disarm
  // is always safe and should never be blocked behind a dialog.
  if (m_confirmBeforeArm && !m_armed) {
    const auto btn = QMessageBox::question(
        this, tr("Confirm ARM"),
        tr("Arm the airframe? The motors will spin up once the throttle is "
           "raised."),
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (btn != QMessageBox::Yes) {
      m_logPanel->appendLog("[GCS] ARM cancelled");
      return;
    }
  }

  // Toggle on the last-known FC state (m_armed is synced from the engine
  // snapshot by the render clock): armed/failsafe -> CMD_DISARM, else CMD_ARM.
  // Both commands are idempotent on the firmware side, so a stale m_armed
  // is harmless. The firmware checks the arm preconditions (throttle low,
  // RC healthy, estimator OK) on its next RC frame. Framing lives in CommandCodec.
  sendToFc(m_armed ? CommandCodec::encodeDisarm() : CommandCodec::encodeArm());
  m_logPanel->appendLog(m_armed ? "[GCS] Sent CMD_DISARM"
                                : "[GCS] Sent CMD_ARM (lower throttle to arm)");
}

void MainWindow::onToggle3d(bool checked) {
  if (m_attStack) {
    m_attStack->setCurrentIndex(checked ? 1 : 0);
  }
}

// ---------------------------------------------------------------------------
// Data Slots
// ---------------------------------------------------------------------------

void MainWindow::onLogReceived(const QString &msg) {
  // Log lines are sparse and stay event-wired; the wire-packet counter lives in
  // the TelemetryEngine now (counted per packet, surfaced via snapshot()).
  m_logPanel->appendLog(msg);
}

// Firmware system-state pill (attitude page) + dashboard graph state band + sim
// HUD. Render-clock driven: called from onUiTimer only when the snapshot's
// vehicleState changes (the kStateNames filter already happened in the engine).
void MainWindow::applyVehicleStatePill(const QString &msg) {
#ifdef NAVIGATOR_HAS_SITL
  if (m_simulatorWidget) m_simulatorWidget->hudSetStatus(msg);
#endif

  // Drive the dashboard graph state band (mockup .g-status SCOL).
  if (m_imuPanel) {
    QColor sc(0x61, 0xAF, 0xEF);  // init blue
    if (msg == "FAILSAFE")
      sc = QColor(0xE0, 0x82, 0x2E);  // orange
    else if (msg == "ARMED" || msg == "IN_AIR")
      sc = QColor(0xE0, 0x6C, 0x75);  // red
    else if (msg == "STANDBY" || msg == "PREARM")
      sc = QColor(0x98, 0xC3, 0x79);  // green
    m_imuPanel->setVehicleState(sc);
  }

  if (m_statusLabel) {
    m_statusLabel->setText(msg.toUpper());
    QString style = "font-size: 18px; font-weight: bold; border-radius: 4px; "
                    "padding: 4px; margin-bottom: 8px;";

    if (msg.contains("FAILSAFE") || msg.contains("ERROR") ||
        msg.contains("FATAL")) {
      style +=
          " color: #E06C75; background: #4A2A2A; border: 1px solid #E06C75;";
    } else if (msg.contains("ARMED") || msg.contains("IN_AIR")) {
      style +=
          " color: #E06C75; background: #3A1A1A; border: 1px solid #E06C75;";
    } else if (msg.contains("INIT") || msg.contains("PREARM")) {
      style +=
          " color: #61AFEF; background: #1A2A3A; border: 1px solid #61AFEF;";
    } else if (msg.contains("STANDBY")) {
      style +=
          " color: #98C379; background: #1A2D23; border: 1px solid #98C379;";
    } else {
      style +=
          " color: #98C379; background: #1A1D27; border: 1px solid #2A3347;";
    }
    m_statusLabel->setStyleSheet(style);
  }
}

// Flight-mode pill (STABILISE/ACRO + RC/GCS). Render-clock driven, called only
// when the snapshot's mode/source changes.
void MainWindow::applyFlightModePill(quint8 mode, quint8 source) {
  if (!m_flightModeLabel) return;
  const bool acro = (mode == 1);
  const bool gcs = (source == 1);
  m_flightModeLabel->setText(
      QString("%1  ·  %2").arg(acro ? "ACRO" : "STABILISE", gcs ? "GCS" : "RC"));
  QString style = "font-size: 13px; font-weight: bold; border-radius: 4px; "
                  "padding: 3px; margin-bottom: 8px;";
  if (acro)  // rate mode: amber, no bank-angle limit
    style += " color: #E5C07B; background: #2D2616; border: 1px solid #E5C07B;";
  else       // stabilise: blue/calm
    style += " color: #61AFEF; background: #1A2A3A; border: 1px solid #61AFEF;";
  m_flightModeLabel->setStyleSheet(style);
}

// ---------------------------------------------------------------------------
// Serial State
// ---------------------------------------------------------------------------

void MainWindow::onConnectionStateChanged(bool connected) {
  setConnected(connected);
  const QString msg =
      connected ? QString("[GCS] Connected to %1").arg(m_currentPort)
                : "[GCS] Disconnected";
  m_logPanel->appendLog(msg);

  // Telemetry recording follows the link (Phase-1 1C).
  if (connected) {
    if (m_recordOnConnect) startRecording();
  } else {
    stopRecording();
    // Export-on-disconnect: dump the System Log buffer to a .log under the log
    // directory (Settings ▸ Logging & Recording).
    if (m_exportOnDisconnect && m_logPanel) {
      const QString stamp =
          QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss");
      const QString path =
          QDir(resolveLogDir()).filePath(QString("session_%1.log").arg(stamp));
      if (m_logPanel->exportToFile(path))
        m_logPanel->appendLog("[GCS] Session log exported → " + path);
      else
        m_logPanel->appendLog("[ERROR] Failed to export session log → " + path);
    }
  }
}

void MainWindow::applyAllSettings(bool persist) {
  const GcsSettings s = m_settingsWidget->getSettings();
  // Remember the steady-state period; only apply it live once locked, otherwise
  // keep the fast acquisition cadence (see onTimeSyncResponse).
  m_syncPeriodMs = s.syncPeriodMs;
  if (m_syncTimer && m_syncLocked) m_syncTimer->setInterval(m_syncPeriodMs);
  if (m_imuPanel) {
    m_imuPanel->setGraphWindow(s.graphWindowSec);
    m_imuPanel->setGraphDropout(s.graphDropoutRate);
    m_imuPanel->setSigmaTraces(s.sigmaTraces);
    m_imuPanel->setStateBand(s.stateBand);
    m_imuPanel->setTraceWidth(s.traceWidth);
    m_imuPanel->setAntialias(s.antialias);
  }
  QMetaObject::invokeMethod(m_engine, "setAutoReconnect", Qt::QueuedConnection,
                            Q_ARG(bool, s.autoReconnect));
  QMetaObject::invokeMethod(
      m_engine, "setReconnectInterval", Qt::QueuedConnection,
      Q_ARG(int, int(s.reconnectIntervalSec * 1000)));
  m_linkLossTimeoutMs = s.linkLossTimeoutMs;
  m_viewHistory.setDepth(s.recentViewsCount);

  // Units & Display: the live readouts (attitude labels, Sim HUD tapes) pull
  // these every render tick, so pushing them here takes effect next frame.
  Units::setAngleUnit(s.angleUnit == 1 ? Units::AngleUnit::Radians
                                       : Units::AngleUnit::Degrees);
  Units::setAltUnit(s.altitudeUnit == 1 ? Units::AltUnit::Feet
                                        : Units::AltUnit::Meters);
  Units::setSpeedUnit(s.speedUnit == 2   ? Units::SpeedUnit::Mph
                      : s.speedUnit == 1 ? Units::SpeedUnit::Kmh
                                         : Units::SpeedUnit::Mps);
  Units::setDecimals(s.decimals);
  // Launch behaviour — consumed by restoreUiState() (which runs after the
  // startup applyAllSettings(false), so these are set in time).
  m_startupPage = s.startupPage;
  m_restoreLayout = s.restoreLayout;

  // Logging & Recording.
  if (m_logPanel) {
    m_logPanel->setTimestampMode(s.timestampMode);  // Local/UTC/Elapsed
    m_logPanel->setMaxLines(s.maxLogLines);
  }
  m_logDir = s.logDirectory;
  m_exportOnDisconnect = s.exportOnDisconnect;

  // Alerts & Audio.
  Notify::setEnabled(s.toastNotifications);
  m_chime.setEnabled(s.audioAlerts);
  m_confirmBeforeArm = s.confirmBeforeArm;
#ifdef NAVIGATOR_HAS_SITL
  if (m_simulatorWidget) m_simulatorWidget->setPropAudioDefault(s.simPropAudio);
#endif

  // Advanced.
  if (m_analyzerWidget) m_analyzerWidget->setPacketCapacity(s.packetBufferRows);
  if (m_engine && m_engine->protocol())
    QMetaObject::invokeMethod(m_engine->protocol(), "setCrcCheck",
                              Qt::QueuedConnection, Q_ARG(bool, s.crcCheck));

  // Record-on-connect: start/stop now if the state actually changed and a feed
  // is live (no-op at startup, where nothing is connected yet).
  const bool wasRec = m_recordOnConnect;
  m_recordOnConnect = s.recordOnConnect;
  if (s.recordOnConnect && !wasRec && m_source.txAllowed())
    startRecording();
  else if (!s.recordOnConnect && wasRec)
    stopRecording();

  // Theme is persisted only (Dark is the only themed palette today). The
  // transport/port/baud launch defaults are applied to the toolbar at startup,
  // not here, since they describe next-launch behaviour.
  if (persist) {
    SettingsManager::save(s);
    if (m_logPanel) m_logPanel->appendLog("[GCS] Settings applied");
  }
}

void MainWindow::updateExportEnabled() {
  if (!m_exportAction) return;
  // Enabled when telemetry is arriving (live link or in-app sim), or while an
  // export is already running so the user can stop it.
  m_exportAction->setEnabled(m_source.txAllowed() || m_exportActive);
}

QString MainWindow::resolveLogDir() const {
  // Settings override wins; otherwise the ~/vayu-logs default. Created on demand.
  const QString path =
      m_logDir.isEmpty() ? QDir::home().filePath("vayu-logs") : m_logDir;
  QDir d(path);
  if (!d.exists()) d.mkpath(".");
  return path;
}

void MainWindow::startRecording() {
  if (!m_recordingPath.isEmpty()) return;  // already recording this session
  QDir logDir(resolveLogDir());
  const QString stamp =
      QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss");
  const QString path = logDir.filePath(QString("rec_%1.bin").arg(stamp));

  // The recorder lives on the worker thread; open it there. protocolVersion is a
  // forward-compat field; record 1 and let replay degrade gracefully.
  QMetaObject::invokeMethod(
      m_engine, "startRecording", Qt::QueuedConnection, Q_ARG(QString, path),
      Q_ARG(qulonglong, qulonglong(QDateTime::currentMSecsSinceEpoch())));
  m_recordingPath = path;
  m_logPanel->appendLog("[GCS] Recording telemetry → " + path);
  // Surface the recording in the recent list immediately — it counts even if
  // the user never opens it for replay, and survives a crash mid-recording.
  addRecentLog(path);
}

void MainWindow::stopRecording() {
  if (m_recordingPath.isEmpty()) return;
  QMetaObject::invokeMethod(m_engine, "stopRecording", Qt::QueuedConnection);
  m_logPanel->appendLog("[GCS] Stopped recording → " + m_recordingPath);
  m_recordingPath.clear();
}

void MainWindow::onSerialError(const QString &msg) {
  m_logPanel->appendLog("[ERROR] " + msg);
  if (m_statusBar) m_statusBar->setError(msg);
  setConnected(false);
}

void MainWindow::setConnected(bool on) {
  // `on` = the live transport is up/down (from connectionStateChanged). The FSM
  // (m_source) owns the mode; this drives the transport-tied UI side effects.
  m_linkLost = false;  // reset the watchdog on any connect/disconnect edge
  updateExportEnabled();

  // Page-level gating: any page whose actions only make sense with a
  // live serial link is told to disable its action surface here.
  // Read-only display pages (Motor / ControlLoop) are intentionally
  // left interactive so the last known data stays visible.
  if (m_calibrationWidget) m_calibrationWidget->setConnected(on);
  if (m_gyroNotchWidget) m_gyroNotchWidget->setConnected(on);

  if (m_toolbar)   m_toolbar->setConnected(on, m_currentPort);
  refreshConnectionPill();

  // ARM/DISARM is reachable whenever the link is up; the firmware enforces
  // the arm preconditions (throttle, RC, estimator). On disconnect reset the
  // cached arm state and the button label back to ARM.
  if (m_toolbar) m_toolbar->setArmEnabled(on);
  if (!on) {
    m_armed = false;
    if (m_toolbar) m_toolbar->setArmState(false);
  }

  // System-state pill on the attitude page. Stays here because it's
  // tied to firmware status packets, not to the connection per se.
  if (m_statusLabel) {
    if (on) {
      m_statusLabel->setText("WAITING");
      m_statusLabel->setStyleSheet(
          QString("font-size: 18px; font-weight: bold; color: %1; "
                  "background: %2; border: 1px solid %3; "
                  "border-radius: 4px; padding: 4px; margin-bottom: 8px;")
              .arg(Theme::hex(Theme::kAccent),
                   Theme::hex(Theme::kBg),
                   Theme::hex(Theme::kAccent)));
    } else {
      m_statusLabel->setText("—");
      m_statusLabel->setStyleSheet(
          QString("font-size: 18px; font-weight: bold; color: %1; "
                  "background: %2; border: 1px solid %3; "
                  "border-radius: 4px; padding: 4px; margin-bottom: 8px;")
              .arg(Theme::hex(Theme::kTextDim),
                   Theme::hex(Theme::kBg),
                   Theme::hex(Theme::kBorder)));
    }
  }

  if (on) {
    // Re-baseline the status-bar packet count to this connection and reset the
    // rate window + pill edge-guards so the next packets re-drive the pills.
    m_pktBase = m_engine->snapshot().packetCount;
    m_pktAtLastRate = m_pktBase;
    m_lastRateTime = 0;
    m_lastPushedState.clear();
    m_lastFlightMode = -1;
    m_lastFlightSrc = -1;
    // Start in fast acquisition mode; onTimeSyncResponse relaxes to the
    // configured period once the clock locks.
    m_syncLocked = false;
    m_syncWide = false;
    m_syncTimer->start(kSyncFastMs);
    onTimeSyncRequested();
  } else {
    m_armed = false;
    m_syncTimer->stop();
  }
}

void MainWindow::refreshConnectionPill() {
  if (!m_statusBar) return;
  // Driven by the FSM: a live Fc link (transport actually up) shows the port;
  // Sim shows SIM; everything else (Idle/Autotune/Replay) shows disconnected
  // (Replay is surfaced by the toolbar REPLAY pill instead).
  const SourceState st = m_source.state();
  if (st == SourceState::Fc && (m_serialOpen || m_udpOpen))
    m_statusBar->setConnectionStatus(true, m_currentPort);
  else if (st == SourceState::Sim)
    m_statusBar->setConnectionStatus(true, QStringLiteral("SIM"));
  else if (st == SourceState::Autotune)
    m_statusBar->setConnectionStatus(true, QStringLiteral("AUTOTUNE"));
  else if (st == SourceState::Replay)
    m_statusBar->setReplayStatus();
  else
    m_statusBar->setConnectionStatus(false, QString());
}

// ---------------------------------------------------------------------------
// Periodic UI update (10 Hz)
// ---------------------------------------------------------------------------

void MainWindow::onUiTimer() {
  static int tick = 0;
  tick++;

  // The render clock: pull one snapshot of the whole vehicle state and push it
  // to every high-rate widget. No widget is driven at the packet rate anymore —
  // per-packet updates happen inside the TelemetryEngine; the UI samples here at
  // ~30 Hz. See docs/telemetry-engine-architecture.md (Phase A).
  const VehicleState s = m_engine->snapshot();
  const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

  // Inbound packet rate (Hz) for the status bar — sampled once a second from the
  // engine's monotonic wire-packet counter (mockup "Rate: N Hz").
  if (m_lastRateTime == 0) m_lastRateTime = nowMs;
  if (nowMs - m_lastRateTime >= 1000) {
    const double hz =
        (s.packetCount - m_pktAtLastRate) * 1000.0 / (nowMs - m_lastRateTime);
    if (m_statusBar) m_statusBar->setPacketRate(hz);
    m_pktAtLastRate = s.packetCount;
    m_lastRateTime = nowMs;
  }
  const int pktShown = static_cast<int>(s.packetCount - m_pktBase);

  // A telemetry feed is present when connected / running the in-app sim / in
  // replay. The sparse pills (state, flight-mode, arm) only follow the snapshot
  // while a feed is active; otherwise setConnected() owns them.
  const bool feedActive = m_source.feedActive();

  // IMU panel — mark unavailable (numeric labels + temp gauge show "-") once the
  // feed goes stale / was never seen. Mirror the IMU onto the sim FPV HUD.
  const bool imuFresh =
      s.lastImuMs != 0 && (nowMs - s.lastImuMs) < kTelemetryStaleMs;
  m_imuPanel->updateImu(s.imu, imuFresh);
  // VERTICAL_STATE freshness — gates both the FC-authoritative AGL (D4) and the
  // fused-vs-raw chart below. Valid means the VERT filter has been seeded.
  const bool vertFresh =
      s.lastVerticalMs != 0 && (nowMs - s.lastVerticalMs) < kTelemetryStaleMs;
  const bool vertUsable = vertFresh && s.vertical.valid;

  // BARO altitude → the IMU panel's Baro Altitude graph (separate feed/stamp).
  // MSL is the absolute sea-level height from the barometer. AGL is height above
  // the ground reference: when the FC publishes its authoritative AGL (VERT,
  // captured while disarmed and frozen at arm — decision D4) we show that;
  // otherwise we fall back to the legacy GCS-side ground-ref (older firmware).
  const bool baroFresh =
      s.lastBaroMs != 0 && (nowMs - s.lastBaroMs) < kTelemetryStaleMs;
  if (baroFresh) {
    const float msl = s.baro.altitudeM;
    if (!m_haveBaroRef || !s.armed) {
      m_baroGroundRefM = msl;
      m_haveBaroRef = true;
    }
    const float agl =
        vertUsable ? s.vertical.aglM : (msl - m_baroGroundRefM);
    m_imuPanel->setBaroAltitude(msl, agl, true);
  } else {
    m_imuPanel->setBaroAltitude(0.0f, 0.0f, false);
  }
  // VERTICAL_STATE → the fused-vs-raw "Vertical Estimate" graph (own feed/stamp).
  // Before the first baro fix the fused outputs are meaningless, so the trace
  // stays NA until the filter is seeded.
  if (vertUsable) {
    m_imuPanel->setVerticalState(s.vertical.altitudeM, s.vertical.baroAltitudeM,
                                 s.vertical.climbRateMs,
                                 s.vertical.accelBiasMs2,
                                 s.vertical.accelUnhealthy, true);
  } else {
    m_imuPanel->setVerticalState(0.0f, 0.0f, 0.0f, 0.0f, false, false);
  }
#ifdef NAVIGATOR_HAS_SITL
  if (m_simulatorWidget) m_simulatorWidget->hudSetImu(s.imu.acc, s.imu.gyr);
#endif

  // Attitude instruments (2D ADI + 3D airframe) render here at the timer rate
  // from the snapshot, decoupled from the packet rate.
  const bool attFresh =
      s.lastAttMs != 0 && (nowMs - s.lastAttMs) < kTelemetryStaleMs;
  if (attFresh) {
    m_attitude->setAttitude(s.attitude);
    m_drone3d->setAttitude(s.attitude);
  }

  // RC + motors: push the latest snapshot while the feed is live (the panels'
  // history graphs buffer + repaint internally). Sampled at the render rate, not
  // per packet.
  const bool rcFresh =
      s.lastRcMs != 0 && (nowMs - s.lastRcMs) < kTelemetryStaleMs;
  if (rcFresh) m_rcWidget->updateChannels(s.rc);
  const bool motorFresh =
      s.lastMotorMs != 0 && (nowMs - s.lastMotorMs) < kTelemetryStaleMs;
  if (motorFresh) {
    QVector<float> speeds;
    for (int i = 0; i < 4; ++i) speeds.append(s.motors.speeds[i]);
    m_motorWidget->setMotorSpeeds(speeds);
  }

  // Sparse pills + arm state — edge-guarded so we only restyle on change.
  if (feedActive) {
    if (!s.vehicleState.isEmpty() && s.vehicleState != m_lastPushedState) {
      const bool enteringFailsafe = s.vehicleState.contains("FAILSAFE") &&
                                    !m_lastPushedState.contains("FAILSAFE");
      m_lastPushedState = s.vehicleState;
      applyVehicleStatePill(s.vehicleState);
      if (enteringFailsafe) {
        m_chime.play(ChimeAudio::Kind::Failsafe);
        Notify::error(this, tr("FAILSAFE"));
      }
    }
    if (s.armed != m_armed) {
      m_armed = s.armed;
      if (m_toolbar) m_toolbar->setArmState(m_armed);
      m_chime.play(m_armed ? ChimeAudio::Kind::Arm : ChimeAudio::Kind::Disarm);
      if (m_armed)
        Notify::ok(this, tr("Armed"));
      else
        Notify::info(this, tr("Disarmed"));
    }
    if (s.flightMode != m_lastFlightMode ||
        s.flightModeSource != m_lastFlightSrc) {
      m_lastFlightMode = s.flightMode;
      m_lastFlightSrc = s.flightModeSource;
      applyFlightModePill(s.flightMode, s.flightModeSource);
    }
  }

  // Throttled updates for numeric labels (update every 4 ticks)
  if (tick % 4 != 0) {
    // Still update packet count and live blinker every tick for smoothness
    if (m_statusBar) m_statusBar->setPacketCount(pktShown);
    updateLiveBlinker();
    return;
  }

  // Update attitude numeric labels (stable ~7.5 Hz update). Angle unit +
  // precision come from Settings ▸ Units & Display via the Units:: helper.
  auto fmtVal = [](float v) {
    return Units::angle(static_cast<double>(v), 7);
  };
  auto fmtStd = [](float v) {
    return QString("± σ %1").arg(Units::toAngle(static_cast<double>(v)), 6, 'f',
                                 Units::decimals() + 1);
  };

  // Attitude readouts: "-" when no fresh attitude telemetry (never seen / stale
  // / link down) so an absent feed is distinct from a real 0.00°. (attFresh is
  // computed once per tick above, shared with the instrument repaint.)
  if (attFresh) {
    m_rollLabel->setText(fmtVal(s.attitude.roll));
    m_pitchLabel->setText(fmtVal(s.attitude.pitch));
    m_yawLabel->setText(fmtVal(s.attitude.yaw));
    m_rollStd->setText(fmtStd(s.rollStd));
    m_pitchStd->setText(fmtStd(s.pitchStd));
    m_yawStd->setText(fmtStd(s.yawStd));
  } else {
    for (QLabel *l : {m_rollLabel, m_pitchLabel, m_yawLabel}) l->setText("-");
    for (QLabel *l : {m_rollStd, m_pitchStd, m_yawStd}) l->setText("± σ -");
  }

  if (m_statusBar) m_statusBar->setPacketCount(pktShown);
  updateLiveBlinker();
}

void MainWindow::updateLiveBlinker() {
  if (!m_toolbar) return;
  // In replay the pill shows a static REPLAY; don't let the heartbeat fade
  // fight it (Phase-1 1D).
  if (m_source.isReplay()) return;
  QLabel *live = m_toolbar->liveLabel();
  if (!live) return;

  if (m_lastHbTime == 0) {
    // Never received a heartbeat — leave the idle qss styling in place.
    live->setStyleSheet(QString());
    return;
  }
  const qint64 elapsed =
      QDateTime::currentMSecsSinceEpoch() - m_lastHbTime;
  MainStatusBar::fadeLive(live, elapsed);

  // Link-loss watchdog (live links only; the sim/replay don't heartbeat the
  // same way). On timeout, flag the connection pill disconnected — the
  // transport stays open, so auto-reconnect still handles genuine drops.
  const bool lost = m_source.state() == SourceState::Fc &&
                    elapsed > m_linkLossTimeoutMs;
  if (lost != m_linkLost) {
    m_linkLost = lost;
    if (m_statusBar) {
      if (lost)
        m_statusBar->setError(
            tr("Link lost — no heartbeat for %1 ms").arg(elapsed));
      else
        refreshConnectionPill();  // heartbeat resumed → restore the pill
    }
  }
}

void MainWindow::onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId) {
  Q_UNUSED(timestamp);
  Q_UNUSED(deviceId);
  // Heartbeat is liveness-only now; clock alignment moved to the time-sync
  // handshake (onTimeSyncResponse). The wire-packet counter lives in the engine.
  m_lastHbTime = QDateTime::currentMSecsSinceEpoch();
  // Flash the LIVE label bright; the UI tick will fade it back.
  if (m_toolbar) MainStatusBar::flashLive(m_toolbar->liveLabel());
}

void MainWindow::onTimeSyncRequested() {
  if (m_source.state() != SourceState::Fc)
    return;

  const quint64 t1 = static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
  // A deviation beyond int32 ms (e.g. the FC's uptime clock vs GCS epoch on cold
  // start) goes via the REQUEST_WIDE two-word path. It is jammed on the FIRST
  // valid sample (no synced() gate): a multi-decade offset is unambiguous, so
  // waiting for the filter just delays the first correction by several
  // round-trips. Small residuals still wait for synced() to filter jitter.
  if (m_syncWide) {
    sendToFc(CommandCodec::encodeTimeSyncRequestWide(m_syncSeq++, t1,
                                                     m_syncWideOffset));
  } else {
    const qint32 cmd = m_tsEst.synced() ? m_syncCorrection : (-2147483647 - 1);
    sendToFc(CommandCodec::encodeTimeSyncRequest(m_syncSeq++, t1, cmd));
  }
}

void MainWindow::onTimeSyncResponse(quint8 seq, quint64 t1, quint64 t2,
                                    quint64 t3, quint64 t4) {
  Q_UNUSED(seq);
  m_lastHbTime = QDateTime::currentMSecsSinceEpoch();  // counts as liveness
  if (m_toolbar) MainStatusBar::flashLive(m_toolbar->liveLabel());

  TimeSyncEstimator::Sample s;
  s.t1 = static_cast<qint64>(t1);
  s.t2 = static_cast<qint64>(t2);
  s.t3 = static_cast<qint64>(t3);
  s.t4 = static_cast<qint64>(t4);
  if (!m_tsEst.addSample(s))
    return;  // implausible round-trip — keep the last good reading

  // The FC now keeps a full 64-bit ms clock, so we discipline against the full
  // offset (no modular-u32 games). The correction to apply is -offset; if it
  // fits int32 we use the usual field, otherwise the REQUEST_WIDE two-word path
  // (cold start: FC uptime clock vs GCS epoch is ~decades, far beyond int32).
  const qint64 off = ((s.t2 - s.t1) + (s.t3 - s.t4)) / 2;  // per-sample FC - GCS
  const qint64 corr = -off;                                // apply to the FC
  constexpr qint64 kSyncEpsilonMs = 50;  // in-sync deadband / lock threshold
  const bool locked = qAbs(corr) < kSyncEpsilonMs;
  constexpr qint64 kI32Min = -2147483647 - 1, kI32Max = 2147483647;
  if (locked) {
    m_syncWide = false;
    m_syncCorrection = static_cast<qint32>(kI32Min);  // no command
  } else if (corr > kI32Min && corr <= kI32Max) {      // fits int32 (avoid sentinel)
    m_syncWide = false;
    m_syncCorrection = static_cast<qint32>(corr);
  } else {
    m_syncWide = true;
    m_syncWideOffset = corr;
  }

  // Poll fast while acquiring; relax to the configured period only once the
  // clock is both locked (inside the deadband) AND synced (enough samples that
  // the drift display is live). Until then keep the fast cadence so the first
  // correction and the first drift readout land within a second or two, not
  // after several steady-period round-trips. Touch the timer only on change.
  const bool relax = locked && m_tsEst.synced();
  if (m_syncLocked != relax) {
    m_syncLocked = relax;
    if (m_syncTimer)
      m_syncTimer->setInterval(relax ? m_syncPeriodMs : kSyncFastMs);
  }

  if (m_statusBar && m_tsEst.synced())
    m_statusBar->showSyncDrift(
        static_cast<qint32>(qBound<qint64>(kI32Min, off, kI32Max)));
}

// ---------------------------------------------------------------------------
// Shortcuts (Phase-0 0l) and UI state persistence (0c + 0m)
// ---------------------------------------------------------------------------
//
// QSettings is namespaced by the org/app name set in main.cpp, so we
// just use grouped string keys. Splitter state goes through Qt's
// built-in saveState()/restoreState() — opaque blob, version-tagged
// by Qt internally, robust to widget add/remove as long as the
// splitter shape doesn't change.

namespace {
constexpr const char *kGeomKey       = "ui/geometry";
constexpr const char *kWinStateKey   = "ui/windowState";
constexpr const char *kTopSplitKey   = "ui/topSplitter";
constexpr const char *kVSplitKey     = "ui/vSplitter";
constexpr const char *kPageKey       = "ui/lastPage";
constexpr const char *kPortKey       = "comm/lastPort";
constexpr const char *kBaudKey       = "comm/lastBaud";
constexpr const char *kRecentLogsKey = "replay/recentLogs";
constexpr int kMaxRecentLogs = 10;
}  // namespace

void MainWindow::installShortcuts() {
  // Page navigation (Ctrl+1‑8), Connect (Ctrl+K), Arm (Ctrl+A), Calibration
  // (Ctrl+7), Fullscreen (F11) and Documentation (F1) are hosted on the menu
  // bar (buildMenuBar), which activates their shortcuts. Only menuless
  // shortcut-commands are registered here and bound to the window directly.

  // Ctrl+L — clear the log panel.
  addAction(m_cmds->add("log.clear", "Clear Log", "Log",
                        QKeySequence("Ctrl+L"), CmdContext::Always, [this] {
                          if (m_logPanel) m_logPanel->clearLog();
                        }));

  // Ctrl+Shift+P — fuzzy command palette over the registry (FR-UX-20).
  addAction(m_cmds->add(
      "command.palette", "Command Palette…", "View",
      QKeySequence("Ctrl+Shift+P"), CmdContext::Always, [this] {
        CommandPalette pal(m_cmds, this);
        pal.move(geometry().center() -
                 QPoint(pal.width() / 2, pal.height() / 2));
        pal.exec();
      }));

  // Ctrl+Tab — recent-views (MRU) switcher (FR-UX-21).
  addAction(m_cmds->add("view.recent", "Recent Views", "View",
                        QKeySequence("Ctrl+Tab"), CmdContext::Always,
                        [this] { showRecentViews(); }));
}

void MainWindow::buildViewTitles() {
  m_viewTitles.clear();
  auto put = [this](QWidget *w, const QString &name) {
    if (!w) return;
    const int i = m_stackedWidget->indexOf(w);
    if (i >= 0) m_viewTitles[i] = name;
  };
  put(m_homeWidget, tr("Home"));
  put(m_analyzerWidget, tr("Packet Analyzer"));
  put(m_rcWidget, tr("RC Channels"));
  put(m_settingsWidget, tr("Settings"));
  put(m_calibrationWidget, tr("Calibration"));
  put(m_motorWidget, tr("Motor Status"));
  put(m_controlLoopWidget, tr("Control Loop"));
  put(m_gyroNotchWidget, tr("Gyro Notch"));
  put(m_perfWidget, tr("Kernel Perf"));
#ifdef NAVIGATOR_HAS_SITL
  put(m_simulatorWidget, tr("Simulator"));
#endif
}

void MainWindow::showRecentViews() {
  const QList<int> mru = m_viewHistory.mru();
  if (mru.size() < 2) return;  // nothing to switch between yet
  QList<QPair<int, QString>> items;
  for (int idx : mru)
    items.append({idx, m_viewTitles.value(idx, tr("View %1").arg(idx + 1))});
  m_recentOverlay->setItems(items);
  m_recentOverlay->startCycle(1);
}

void MainWindow::saveUiState() {
  QSettings s;
  s.setValue(kGeomKey,     saveGeometry());
  s.setValue(kWinStateKey, saveState());
  if (m_topSplitter) s.setValue(kTopSplitKey, m_topSplitter->saveState());
  if (m_vSplitter)   s.setValue(kVSplitKey,   m_vSplitter->saveState());
  if (m_stackedWidget) s.setValue(kPageKey, m_stackedWidget->currentIndex());
  persistPortBaud();
}

void MainWindow::restoreUiState() {
  QSettings s;
  // Window geometry + splitter sizes are gated by "Restore layout" (Settings ▸
  // Units & Display). Off ⇒ open at the default size with default splitters.
  if (m_restoreLayout) {
    if (s.contains(kGeomKey))     restoreGeometry(s.value(kGeomKey).toByteArray());
    if (s.contains(kWinStateKey)) restoreState(s.value(kWinStateKey).toByteArray());
    if (m_topSplitter && s.contains(kTopSplitKey))
      m_topSplitter->restoreState(s.value(kTopSplitKey).toByteArray());
    if (m_vSplitter && s.contains(kVSplitKey))
      m_vSplitter->restoreState(s.value(kVSplitKey).toByteArray());
  }

  // Restore last-used port / baud through the toolbar's typed accessors (this is
  // connection state, not layout, so it's kept regardless of "Restore layout").
  // setPort handles both "matches a known item" and "custom path" cases.
  if (m_toolbar) {
    m_toolbar->setPort(s.value(kPortKey).toString());
    m_toolbar->setBaud(s.value(kBaudKey).toInt());
  }

  // Startup page (Settings ▸ Units & Display): the dashboard/simulator choices
  // win outright; "Last viewed" restores the saved page index.
  if (m_startupPage == 1) {
    showHome();
#ifdef NAVIGATOR_HAS_SITL
  } else if (m_startupPage == 2) {
    showSimulator();
#endif
  } else if (m_stackedWidget && s.contains(kPageKey)) {
    const int idx = s.value(kPageKey).toInt();
    if (idx >= 0 && idx < m_stackedWidget->count())
      m_stackedWidget->setCurrentIndex(idx);
  }
}

void MainWindow::persistPortBaud() {
  if (!m_toolbar) return;
  QSettings s;
  s.setValue(kPortKey, m_toolbar->currentPort());
  s.setValue(kBaudKey, m_toolbar->currentBaud());
}

void MainWindow::addRecentLog(const QString &path) {
  if (path.isEmpty()) return;
  const QString abs = QFileInfo(path).absoluteFilePath();
  QSettings s;
  QStringList logs = s.value(kRecentLogsKey).toStringList();
  logs.removeAll(abs);              // dedup: move to front if already present
  logs.prepend(abs);
  while (logs.size() > kMaxRecentLogs) logs.removeLast();
  s.setValue(kRecentLogsKey, logs);
  rebuildRecentLogsMenu();
}

void MainWindow::rebuildRecentLogsMenu() {
  if (!m_recentLogsMenu) return;
  m_recentLogsMenu->clear();

  QSettings s;
  QStringList logs = s.value(kRecentLogsKey).toStringList();

  // Prune entries whose files have since been deleted, but keep the currently
  // recording file even though it may not exist on disk yet.
  QStringList live;
  for (const QString &p : logs)
    if (p == m_recordingPath || QFileInfo::exists(p)) live.append(p);
  if (live != logs) s.setValue(kRecentLogsKey, live);

  if (live.isEmpty()) {
    QAction *none = m_recentLogsMenu->addAction(tr("(no recent logs)"));
    none->setEnabled(false);
    return;
  }

  int n = 1;
  for (const QString &path : live) {
    // &1..&9, then plain — the leading digit is an Alt-accelerator.
    const QString prefix = n <= 9 ? QString("&%1  ").arg(n) : QString();
    QAction *a =
        m_recentLogsMenu->addAction(prefix + QFileInfo(path).fileName());
    a->setData(path);
    a->setToolTip(path);
    connect(a, &QAction::triggered, this, [this, path] { enterReplay(path); });
    ++n;
  }
  m_recentLogsMenu->addSeparator();
  QAction *clear = m_recentLogsMenu->addAction(tr("Clear Recent Logs"));
  connect(clear, &QAction::triggered, this, [this] {
    QSettings cs;
    cs.remove(kRecentLogsKey);
    rebuildRecentLogsMenu();
  });
}

void MainWindow::openLastRecentLog() {
  QSettings s;
  const QStringList logs = s.value(kRecentLogsKey).toStringList();
  for (const QString &path : logs) {
    if (QFileInfo::exists(path)) {
      enterReplay(path);
      return;
    }
  }
  Notify::warn(this, tr("No recent log to open"));
}
