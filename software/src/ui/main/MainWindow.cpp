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
#include "../core/crc.h"
#include "comm/PortArbiter.h"

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QStringList>
#include <QSettings>
#include <QSplitter>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  setWindowTitle("Vayu GCS — Ground Control Station");
  setMinimumSize(1100, 700);
  resize(1300, 820);

  // Palette + stylesheet are applied in main() via Theme::apply() so
  // every dialog/secondary window picks them up; nothing to do here.

  // Back-end objects
  m_serial = new SerialManager(this);
  m_udp = new UdpManager(this);
  m_protocol = new DroneProtocol(this);

  // If another subsystem (the in-sim RC bridge) claims our serial port, drop
  // the board connection so we never contend for the same tty.
  connect(&PortArbiter::instance(), &PortArbiter::revoked, this,
          [this](const QString &, QObject *owner) {
            if (owner == this && m_serial && m_serial->isOpen()) {
              m_logPanel->appendLog(
                  "[GCS] Port taken by the simulator RC — disconnected.");
              m_serial->close();
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

  // Session mode (Phase-1 1D): in replay the toolbar goes read-only and the
  // LIVE pill flips to REPLAY. Dormant until a ReplaySource is attached (2E).
  connect(&m_session, &SessionState::changed, this, [this](SessionMode m) {
    const bool replay = (m == SessionMode::Replay);
    if (m_toolbar) m_toolbar->setReplayMode(replay);
    Notify::info(this, replay ? tr("Replay — read-only") : tr("Live"));
  });

  // Wire inbound bytes → protocol → UI through the telemetry-source seam
  // (Phase-1 1B). The active source forwards serial + UDP (and, in replay,
  // a recorded .bin) so the decode path never sees the transport.
  m_liveSource = new LiveSource(m_serial, m_udp, this);
  m_source = m_liveSource;
  connect(m_source, &ITelemetrySource::bytesReceived, m_protocol,
          &DroneProtocol::processData);
  connect(m_udp, &UdpManager::connectionStateChanged, this,
          &MainWindow::onConnectionStateChanged);
  connect(m_udp, &UdpManager::errorOccurred, this, [this](const QString &m) {
    m_logPanel->appendLog("[UDP] " + m);
  });

  connect(m_protocol, &DroneProtocol::imuReceived, this,
          &MainWindow::onImuReceived);
  connect(m_protocol, &DroneProtocol::attitudeReceived, this,
          &MainWindow::onAttitudeReceived);
  connect(m_protocol, &DroneProtocol::logReceived, this,
          &MainWindow::onLogReceived);
  connect(m_protocol, &DroneProtocol::statusReceived, this,
          &MainWindow::onStatusReceived);
  connect(m_protocol, &DroneProtocol::unknownPacket, this,
          [this](const QByteArray &raw) {
            onLogReceived(QString("[raw] ") + QString::fromLatin1(raw));
          });

  connect(m_serial, &SerialManager::connectionStateChanged, this,
          &MainWindow::onConnectionStateChanged);
  connect(m_serial, &SerialManager::errorOccurred, this,
          &MainWindow::onSerialError);
  connect(m_protocol, &DroneProtocol::heartbeatReceived, this,
          &MainWindow::onHeartbeatReceived);
  connect(m_protocol, &DroneProtocol::timeSyncRequested, this,
          &MainWindow::onTimeSyncRequested);
  connect(m_protocol, &DroneProtocol::rcReceived, this,
          &MainWindow::onRcReceived);
  connect(m_protocol, &DroneProtocol::motorReceived, this,
          &MainWindow::onMotorReceived);

  m_rcWidget = new RcChannelsWidget(this);
  m_stackedWidget->addWidget(m_rcWidget);

  connect(m_rcWidget, &RcChannelsWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Periodic status refresh at 20 Hz for smoother fade
  connect(m_uiTimer, &QTimer::timeout, this, &MainWindow::onUiTimer);
  m_uiTimer->start(50);

  connect(m_syncTimer, &QTimer::timeout, this,
          &MainWindow::onTimeSyncRequested);

  buildUi(); // Built as m_homeWidget

  // Build Packet Analyzer
  m_analyzerWidget = new PacketAnalyzerWidget(this);
  m_analyzerWidget->setProtocol(m_protocol);
  m_stackedWidget->addWidget(m_analyzerWidget);

  // Wire Protocol and Serial to Analyzer
  connect(m_protocol, &DroneProtocol::packetReceived, m_analyzerWidget,
          &PacketAnalyzerWidget::logRxPacket);
  connect(m_protocol, &DroneProtocol::unknownPacket, m_analyzerWidget,
          &PacketAnalyzerWidget::logRxPacket);
  connect(m_serial, &SerialManager::dataSent, m_analyzerWidget,
          &PacketAnalyzerWidget::logTxPacket);
  connect(m_udp, &UdpManager::dataSent, m_analyzerWidget,
          &PacketAnalyzerWidget::logTxPacket);
  connect(m_analyzerWidget, &PacketAnalyzerWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Settings
  m_settingsWidget = new SettingsWidget(this);
  m_stackedWidget->addWidget(m_settingsWidget);
  connect(m_settingsWidget, &SettingsWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_settingsWidget, &SettingsWidget::syncPeriodChanged, this,
          [this](int ms) {
            if (m_syncTimer) {
              m_syncTimer->setInterval(ms);
              m_logPanel->appendLog(
                  QString("[GCS] Sync period updated to %1 ms").arg(ms));
            }
            SettingsManager::save(m_settingsWidget->getSettings());
          });
  connect(m_settingsWidget, &SettingsWidget::graphWindowChanged, this,
          [this](int seconds) {
            m_imuPanel->setGraphWindow(seconds);
            SettingsManager::save(m_settingsWidget->getSettings());
          });
  connect(m_settingsWidget, &SettingsWidget::graphDropoutChanged, this,
          [this](double rate) {
            m_imuPanel->setGraphDropout(rate);
            SettingsManager::save(m_settingsWidget->getSettings());
          });
  connect(m_settingsWidget, &SettingsWidget::autoReconnectChanged, this,
          [this](bool on) {
            if (m_serial) m_serial->setAutoReconnect(on);
            SettingsManager::save(m_settingsWidget->getSettings());
          });
  connect(m_settingsWidget, &SettingsWidget::recordOnConnectChanged, this,
          [this](bool on) {
            m_recordOnConnect = on;
            // Apply immediately if a link is already up.
            if (on && (m_connected || m_simRunning)) startRecording();
            else if (!on) stopRecording();
            SettingsManager::save(m_settingsWidget->getSettings());
          });
  connect(m_settingsWidget, &SettingsWidget::recentViewsCountChanged, this,
          [this](int n) {
            m_viewHistory.setDepth(n);
            SettingsManager::save(m_settingsWidget->getSettings());
          });

  // Tee inbound bytes to the recorder (Phase-1 1C). Same source the parser
  // reads, so the recording is exactly the live stream.
  connect(m_source, &ITelemetrySource::bytesReceived, this,
          [this](const QByteArray &b) {
            if (m_recorder.isOpen())
              m_recorder.writeFrame(quint64(m_elapsed.nsecsElapsed() / 1000),
                                    b);
          });

  // Load and apply persistent settings
  GcsSettings savedSettings;
  if (SettingsManager::load(savedSettings)) {
    m_settingsWidget->setSettings(savedSettings);
    // Trigger the actual logic changes
    m_syncTimer->setInterval(savedSettings.syncPeriodMs);
    m_imuPanel->setGraphWindow(savedSettings.graphWindowSec);
    m_imuPanel->setGraphDropout(savedSettings.graphDropoutRate);
    m_serial->setAutoReconnect(savedSettings.autoReconnect);
    m_recordOnConnect = savedSettings.recordOnConnect;
    m_viewHistory.setDepth(savedSettings.recentViewsCount);
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
  connect(m_toolbar, &MainToolbar::showRcRequested, this,
          &MainWindow::showRcMonitor);
  connect(m_toolbar, &MainToolbar::showCalibRequested, this,
          &MainWindow::showCalibration);

  m_toolbar->refreshPorts();  // populate port list on startup
  setConnected(false);
  // Build Calibration
  m_calibrationWidget = new CalibrationWidget(this);
  m_calibrationWidget->setProtocol(m_protocol);
  m_stackedWidget->addWidget(m_calibrationWidget);
  connect(m_calibrationWidget, &CalibrationWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_calibrationWidget, &CalibrationWidget::commandRequested,
          [this](const QByteArray &data) {
            if (m_connected)
              sendToFc(data);
          });

  // Build Motor Status
  m_motorWidget = new MotorStatusWidget(this);
  m_stackedWidget->addWidget(m_motorWidget);
  connect(m_motorWidget, &MotorStatusWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Control Loop Plot
  m_controlLoopWidget = new ControlLoopPlot(this);
  m_controlLoopWidget->setProtocol(m_protocol);
  m_stackedWidget->addWidget(m_controlLoopWidget);
  connect(m_controlLoopWidget, &ControlLoopPlot::backToHomeRequested, this,
          &MainWindow::showHome);

  // Build Kernel Observability (vaios perf telemetry)
  m_perfWidget = new PerfWidget(this);
  m_stackedWidget->addWidget(m_perfWidget);
  connect(m_perfWidget, &PerfWidget::backToHomeRequested, this,
          &MainWindow::showHome);
  connect(m_protocol, &DroneProtocol::perfReceived, m_perfWidget,
          &PerfWidget::updateReport);
  // On-demand task-name resolution: widget asks, FC replies, widget caches.
  connect(m_protocol, &DroneProtocol::taskNameReceived, m_perfWidget,
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
  connect(m_simulatorWidget, &SimulatorWidget::dataReceived,
          m_protocol, &DroneProtocol::processData);
  // AT-1: the autotune tab proposes gains; applying to firmware is an explicit
  // operator action. Turn the proposal into CMD_SET_PID frames and send them
  // over the live link (sendToFc refuses in replay; we also require a link).
  connect(m_simulatorWidget, &SimulatorWidget::applyPidGainsRequested, this,
          [this](const QVector<PidSetCmd> &cmds) {
            const bool linkUp =
                m_connected || (m_udp && m_udp->isOpen());
            if (!linkUp) {
              m_logPanel->appendLog(
                  "[GCS] Apply gains ignored — no flight controller link");
              Notify::warn(this, tr("Not connected — can't apply gains"));
              return;
            }
            const uint32_t now =
                static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
            for (const PidSetCmd &c : cmds)
              sendToFc(CommandCodec::encodeSetPid(c.controller, c.axis, c.kp,
                                                  c.ki, c.kd, c.kff, 42, now));
            m_logPanel->appendLog(
                QString("[GCS] Applied %1 PID slot(s) to firmware (CMD_SET_PID)")
                    .arg(cmds.size()));
            Notify::ok(this, tr("Applied gains to firmware"));
          });
  // Reflect the firmware's reported flight mode (stabilise/acro + RC/GCS source)
  // back onto the simulator's Acro toggle.
  connect(m_protocol, &DroneProtocol::flightModeReceived,
          m_simulatorWidget, &SimulatorWidget::setFlightModeStatus);
  connect(m_protocol, &DroneProtocol::flightModeReceived,
          this, &MainWindow::onFlightModeReceived);
  // Reflect the in-app sim as "Connected: SIM" in the bottom status bar.
  connect(m_simulatorWidget, &SimulatorWidget::simRunningChanged, this,
          [this](bool running) {
            m_simRunning = running;
            refreshConnectionPill();
            // While the in-app sim feeds telemetry, lock out the serial
            // connection controls (port / baud / refresh / Connect) so the
            // user can't open a conflicting real link.
            if (m_toolbar) m_toolbar->setSerialControlsEnabled(!running);
          });
#endif

  // Recent-views (MRU) switcher (Phase-2 2C / FR-UX-21). Record every page
  // change centrally and build the index->label map the overlay displays.
  buildViewTitles();
  connect(m_stackedWidget, &QStackedWidget::currentChanged, this,
          [this](int idx) {
            if (idx >= 0) m_viewHistory.visit(idx);
          });
  m_recentOverlay = new RecentViewsOverlay(this);
  connect(m_recentOverlay, &RecentViewsOverlay::activated, this,
          [this](int idx) { m_stackedWidget->setCurrentIndex(idx); });

  // Replay transport bar (Phase-2 2E), docked at the bottom, shown only in
  // replay. Exit returns to the live session.
  m_replayBar = new ReplayBar(this);
  connect(m_replayBar, &ReplayBar::exitRequested, this,
          &MainWindow::exitReplay);
  m_replayToolbar = new QToolBar(tr("Replay"), this);
  m_replayToolbar->setMovable(false);
  m_replayToolbar->addWidget(m_replayBar);
  addToolBar(Qt::BottomToolBarArea, m_replayToolbar);
  m_replayToolbar->setVisible(false);

  installShortcuts();

  // Editable shortcut overrides on top of the registry defaults (Phase-2 2A).
  // Created after every command is registered, then load() applies any saved
  // overrides to the shared QActions.
  m_shortcuts = new ShortcutsManager(m_cmds, this);
  m_shortcuts->load();

  // Restore window geometry, splitter sizes, last page, port/baud.
  // Must run after every widget the state references has been built.
  restoreUiState();

  // If restoreUiState didn't put us on a page (first run), default to home.
  if (m_stackedWidget->currentWidget() == nullptr) showHome();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  saveUiState();
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

void MainWindow::showControlLoopPlot() {
  m_stackedWidget->setCurrentWidget(m_controlLoopWidget);
}

void MainWindow::showPerf() {
  m_stackedWidget->setCurrentWidget(m_perfWidget);
}

void MainWindow::sendTaskNameRequest(int taskId) {
  if (!m_serial || taskId < 0 || taskId > 255)
    return;
  // PERF_TASKNAME (0xA) request frame: payload = [task_id] (1 byte).
  const uint32_t now =
      static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  const uint8_t dev_id = 42;
  QByteArray pkt;
  pkt.append(static_cast<char>(0x56));            // sync
  pkt.append(static_cast<char>((0xA << 4) | 0x1)); // type 0xA, proto v1
  pkt.append(static_cast<char>(1));               // payload length
  pkt.append(static_cast<char>(dev_id));
  pkt.append(reinterpret_cast<const char *>(&now), 4);
  pkt.append(static_cast<char>(taskId & 0xFF));
  const uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()),
      static_cast<uint32_t>(pkt.size()));
  pkt.append(reinterpret_cast<const char *>(&crc), 4);
  sendToFc(pkt);
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

  // Toggle button at the top right of the group
  auto *headerLayout = new QHBoxLayout();
  headerLayout->addStretch();
  auto *toggleBtn = new QPushButton("3D VIEW", attGroup);
  toggleBtn->setObjectName("ToggleButton");
  toggleBtn->setCheckable(true);
  toggleBtn->setFixedSize(70, 22);
  toggleBtn->setToolTip(tr("Toggle between the 2D HUD and 3D airframe view"));
  headerLayout->addWidget(toggleBtn);
  attLayout->addLayout(headerLayout);

  m_attStack = new QStackedWidget(attGroup);
  m_attitude = new AttitudeWidget(m_attStack);
  m_drone3d = new Drone3DWidget(m_attStack);
  m_attStack->addWidget(m_attitude);
  m_attStack->addWidget(m_drone3d);
  attLayout->addWidget(m_attStack, 1);

  connect(toggleBtn, &QPushButton::toggled, this, &MainWindow::onToggle3d);

  // Firmware system-state pill. Shows ARMED / STANDBY / FAILSAFE /…
  // as system-status packets arrive. Connection state lives in the
  // status-bar pill; this label deliberately starts as a muted dash so
  // it doesn't compete with that (FR-UX-05).
  m_statusLabel = new QLabel("—", attGroup);
  m_statusLabel->setAlignment(Qt::AlignCenter);
  m_statusLabel->setStyleSheet(
      QString("font-size: 18px; font-weight: bold; color: %1; "
              "background: %2; border: 1px solid %3; "
              "border-radius: 4px; padding: 4px; margin-bottom: 8px;")
          .arg(Theme::hex(Theme::kTextDim),
               Theme::hex(Theme::kBg),
               Theme::hex(Theme::kBorder)));
  attLayout->addWidget(m_statusLabel);

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
    (*lblPtrs[i])->setFixedWidth(100);

    QLabel **stdPtrs[] = {&m_rollStd, &m_pitchStd, &m_yawStd};
    *stdPtrs[i] = new QLabel("± σ 0.000", container);
    (*stdPtrs[i])
        ->setStyleSheet(QString(
            "color: #888888; font-size: 10px; font-family: Monospace;"));
    (*stdPtrs[i])->setAlignment(Qt::AlignCenter);
    (*stdPtrs[i])->setFixedWidth(100);

    vbox->addWidget(title);
    vbox->addWidget(*lblPtrs[i]);
    vbox->addWidget(*stdPtrs[i]);
    numHBox->addWidget(container);
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

  // Every menu item is a registered command (FR-UX-19): the registry owns
  // the QAction, the menu just hosts it. Adding the action to the menu also
  // activates its shortcut, so these must NOT be added to the window again.
  QMenu *fileMenu = menu->addMenu("&File");
  fileMenu->addAction(m_cmds->add("view.home", "&Home Screen", "View",
                                  QKeySequence("Ctrl+H"), CmdContext::Always,
                                  [this] { showHome(); }));
  fileMenu->addAction(m_cmds->add(
      "view.packetAnalyzer", "&Packet Analyzer", "View",
      QKeySequence("Ctrl+P"), CmdContext::Always,
      [this] { showPacketAnalyzer(); }));

  fileMenu->addAction(m_cmds->add(
      "replay.open", "&Open Log…", "Replay", QKeySequence("Ctrl+O"),
      CmdContext::Always, [this] {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Open Telemetry Log"),
            QDir::home().filePath("vayu-logs"),
            tr("Telemetry recordings (*.bin)"));
        if (!path.isEmpty()) enterReplay(path);
      }));

  QMenu *settingsMenu = menu->addMenu("&Settings");
  settingsMenu->addAction(m_cmds->add("view.settings", "&Configuration",
                                      "View", QKeySequence(),
                                      CmdContext::Always,
                                      [this] { showSettings(); }));

  QMenu *windowMenu = menu->addMenu("&Window");
  windowMenu->addAction(m_cmds->add("view.channels", "&Channels", "View",
                                    QKeySequence(), CmdContext::Always,
                                    [this] { showRcMonitor(); }));
  windowMenu->addAction(m_cmds->add("view.calibration", "&Calibration", "View",
                                    QKeySequence(), CmdContext::Always,
                                    [this] { showCalibration(); }));
  windowMenu->addAction(m_cmds->add("view.motorStatus", "&Motor Status",
                                    "View", QKeySequence(), CmdContext::Always,
                                    [this] { showMotorStatus(); }));
  windowMenu->addAction(m_cmds->add("view.controlLoop", "&Control Loop",
                                    "View", QKeySequence(), CmdContext::Always,
                                    [this] { showControlLoopPlot(); }));
  windowMenu->addAction(m_cmds->add("view.kernelPerf", "&Kernel Perf", "View",
                                    QKeySequence(), CmdContext::Always,
                                    [this] { showPerf(); }));
#ifdef NAVIGATOR_HAS_SITL
  windowMenu->addAction(m_cmds->add("view.simulator", "Si&mulator", "View",
                                    QKeySequence(), CmdContext::Always,
                                    [this] { showSimulator(); }));
#endif

  QMenu *helpMenu = menu->addMenu("&Help");
  helpMenu->addAction(m_cmds->add(
      "help.shortcuts", "&Keyboard Shortcuts…", "Help",
      QKeySequence("Ctrl+Alt+K"), CmdContext::Always, [this] {
        ShortcutsEditorDialog dlg(m_cmds, m_shortcuts, this);
        dlg.exec();
      }));
  helpMenu->addAction(m_cmds->add(
      "help.docs", "&Documentation", "Help", QKeySequence(),
      CmdContext::Always, [this] {
        const QString docs = AboutDialog::docsPath();
        if (docs.isEmpty())
          Notify::warn(this, tr("Bundled documentation not found"));
        else
          QDesktopServices::openUrl(QUrl::fromLocalFile(docs));
      }));
  helpMenu->addSeparator();
  helpMenu->addAction(m_cmds->add(
      "help.about", "&About Navigator", "Help", QKeySequence(),
      CmdContext::Always, [this] {
        AboutDialog dlg(this);
        dlg.exec();
      }));

  fileMenu->addSeparator();

  fileMenu->addAction(m_cmds->add("app.exit", "E&xit", "Application",
                                  QKeySequence(QKeySequence::Quit),
                                  CmdContext::Always, [this] { close(); }));
}

// ---------------------------------------------------------------------------
// Toolbar intent slots — toolbar emits, MainWindow drives serial.
// ---------------------------------------------------------------------------

void MainWindow::onConnectRequested(const QString &port, int baud) {
  if (m_session.isReplay()) {
    Notify::warn(this, tr("Read-only in replay — exit replay to connect"));
    return;
  }
  if (port.isEmpty()) {
    m_logPanel->appendLog("[GCS] No port specified");
    return;
  }
  // UDP transport: type "udp" or "udp:<port>" (default 14555) in the Port field
  // to receive over WiFi from the ESP8266 telemetry bridge. Feeds the very same
  // parser as serial, so every page works unchanged. Baud is ignored.
  if (port.startsWith("udp", Qt::CaseInsensitive)) {
    quint16 udpPort = 14555;
    const int colon = port.indexOf(':');
    if (colon >= 0) {
      const quint16 p = port.mid(colon + 1).toUShort();
      if (p)
        udpPort = p;
    }
    if (m_udp->bind(udpPort)) {
      m_logPanel->appendLog(QString("[UDP] listening on :%1").arg(udpPort));
      persistPortBaud();
    }
    return;
  }
  // Claim the port: if the in-sim RC bridge holds it, this revokes it from the
  // sim (which disconnects), so the two never fight over the same tty.
  PortArbiter::instance().acquire(port, this);
  if (m_serial->open(port, baud)) {
    persistPortBaud();
  } else {
    PortArbiter::instance().release(port, this);
  }
}

void MainWindow::onDisconnectRequested() {
  if (m_udp && m_udp->isOpen()) {
    m_udp->close();
  }
  if (m_serial) {
    PortArbiter::instance().release(m_serial->currentPort(), this);
    m_serial->close();
  }
}

void MainWindow::setSessionMode(SessionMode mode) {
  // Drives the read-only authority and the toolbar/pill via
  // SessionState::changed. enterReplay/exitReplay handle the source swap.
  m_session.setMode(mode);
}

void MainWindow::enterReplay(const QString &path) {
  if (m_session.isReplay()) exitReplay();  // re-open: drop the previous log

  auto *rs = new ReplaySource(this);
  if (!rs->open(path)) {
    Notify::error(this, tr("Could not open recording: %1").arg(path));
    rs->deleteLater();
    return;
  }

  // A live recording must not capture replayed frames; stop it first.
  stopRecording();

  // Swap the parser's source: detach live, attach replay. The decode path and
  // every widget are untouched (Phase-1 1B seam).
  disconnect(m_liveSource, &ITelemetrySource::bytesReceived, m_protocol,
             &DroneProtocol::processData);
  connect(rs, &ITelemetrySource::bytesReceived, m_protocol,
          &DroneProtocol::processData);
  m_replaySource = rs;
  m_source = rs;

  m_replayBar->bind(rs);
  m_replayToolbar->setVisible(true);
  setSessionMode(SessionMode::Replay);  // read-only authority + REPLAY pill
  m_logPanel->appendLog("[GCS] Replay: " + path);
}

void MainWindow::exitReplay() {
  if (!m_replaySource) return;

  m_replaySource->pause();
  disconnect(m_replaySource, &ITelemetrySource::bytesReceived, m_protocol,
             &DroneProtocol::processData);
  connect(m_liveSource, &ITelemetrySource::bytesReceived, m_protocol,
          &DroneProtocol::processData);
  m_source = m_liveSource;

  m_replayBar->bind(nullptr);
  m_replayToolbar->setVisible(false);
  m_replaySource->deleteLater();
  m_replaySource = nullptr;

  setSessionMode(SessionMode::Live);
  m_logPanel->appendLog("[GCS] Exited replay — live");
}

void MainWindow::sendToFc(const QByteArray &pkt) {
  // Read-only authority (Phase-1 1D): replay never transmits. This is the
  // single tx choke point — ARM / PID / calibrate / time-sync all route here,
  // so one guard makes the whole session read-only.
  if (!m_session.txAllowed()) {
    Notify::warn(this, tr("Read-only in replay"));
    return;
  }

  // Route to the active transport: over UDP it goes to the ESP bridge, which
  // writes it out to the FC's UART; over serial it's the wired path.
  if (m_udp && m_udp->isOpen())
    m_udp->write(pkt);
  else if (m_serial && m_serial->isOpen())
    m_serial->write(pkt);  // was a recursive sendToFc() call — infinite loop
}

void MainWindow::onArmClicked() {
  if (!m_serial || !m_connected) {
    m_logPanel->appendLog("[GCS] ARM/DISARM ignored — not connected");
    return;
  }

  // Toggle on the last-known FC state (driven by telemetry in
  // onStatusReceived): armed/failsafe -> CMD_DISARM, otherwise CMD_ARM.
  // Both commands are idempotent on the firmware side, so a stale m_armed
  // is harmless. The firmware checks the arm preconditions (throttle low,
  // RC healthy, estimator OK) on its next RC frame.
  const uint16_t cmd_id = m_armed ? 0x0003 /*CMD_DISARM*/ : 0x0002 /*CMD_ARM*/;

  // NavLink command frame (matches CalibrationWidget): no args, so
  // length = 2 (just the cmd_id). sync, type|ver, length, dev_id, ts,
  // payload, crc32 over everything preceding the crc.
  const uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  const uint8_t dev_id = 42;
  QByteArray pkt;
  pkt.append(static_cast<char>(0x56));   // sync
  pkt.append(static_cast<char>(0x31));   // packet type 3 (command), proto v1
  pkt.append(static_cast<char>(2));      // payload length = 2 (cmd_id only)
  pkt.append(static_cast<char>(dev_id));
  pkt.append(reinterpret_cast<const char *>(&now), 4);
  pkt.append(reinterpret_cast<const char *>(&cmd_id), 2);
  const uint32_t crc = CRC32::calculate(
      reinterpret_cast<const uint8_t *>(pkt.constData()),
      static_cast<uint32_t>(pkt.size()));
  pkt.append(reinterpret_cast<const char *>(&crc), 4);

  sendToFc(pkt);
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

void MainWindow::onImuReceived(const ImuData &data) {
  m_latestImu = data;
  if (m_simulatorWidget) m_simulatorWidget->hudSetImu(data.acc, data.gyr);
  ++m_pktCount;
}

void MainWindow::onAttitudeReceived(const AttitudeData &data) {
  m_latestAtt = data;
  m_attStats[0].push(data.roll);
  m_attStats[1].push(data.pitch);
  m_attStats[2].push(data.yaw);

  m_attitude->setAttitude(data);
  m_drone3d->setAttitude(data);
  ++m_pktCount;
}

void MainWindow::onLogReceived(const QString &msg) {
  m_logPanel->appendLog(msg);
  ++m_pktCount;
}

void MainWindow::onStatusReceived(const QString &msg) {
  // Track the FC's armed state from genuine state-name status strings only
  // (statusReceived also carries log / other strings). This drives the ARM
  // button label so it reflects the vehicle, not the last click. ARMED /
  // IN_AIR / FAILSAFE all show DISARM (so the operator can always recover).
  static const QStringList kStateNames = {
      "UNINITIALIZED", "INIT",     "STANDBY",    "PREARM",     "ARMED",
      "IN_AIR",        "FAILSAFE", "TERMINATED", "CALIBRATING"};
  // PACKET_TYPE_SYSTEM_STATUS is multiplexed across origins (SYS_STATE,
  // HEALTH counters, control data, ...). Only the SYS_STATE origin decodes to
  // a real state name; the binary HEALTH counters get stringified to
  // non-printable bytes upstream, which would render in the pill as a tofu
  // box. Ignore anything that isn't a recognised state name.
  if (!kStateNames.contains(msg)) {
    ++m_pktCount;
    return;
  }
  if (m_simulatorWidget) m_simulatorWidget->hudSetStatus(msg);
  {
    const bool armedish =
        (msg == "ARMED" || msg == "IN_AIR" || msg == "FAILSAFE");
    if (armedish != m_armed) {
      m_armed = armedish;
      if (m_toolbar) m_toolbar->setArmState(m_armed);
    }
  }

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
  ++m_pktCount;
}

void MainWindow::onFlightModeReceived(quint8 mode, quint8 source) {
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
  ++m_pktCount;
}

void MainWindow::onRcReceived(const RcData &data) {
  m_rcWidget->updateChannels(data);
  ++m_pktCount;
}

void MainWindow::onMotorReceived(const MotorData &data) {
  QVector<float> speeds;
  for (int i = 0; i < 4; ++i)
    speeds.append(data.speeds[i]);
  m_motorWidget->setMotorSpeeds(speeds);
  ++m_pktCount;
}

// ---------------------------------------------------------------------------
// Serial State
// ---------------------------------------------------------------------------

void MainWindow::onConnectionStateChanged(bool connected) {
  setConnected(connected);
  const QString msg =
      connected ? QString("[GCS] Connected to %1").arg(m_serial->currentPort())
                : "[GCS] Disconnected";
  m_logPanel->appendLog(msg);

  // Telemetry recording follows the link (Phase-1 1C).
  if (connected) {
    if (m_recordOnConnect) startRecording();
  } else {
    stopRecording();
  }
}

void MainWindow::startRecording() {
  if (m_recorder.isOpen()) return;  // already recording this session
  QDir logDir(QDir::home().filePath("vayu-logs"));
  if (!logDir.exists()) logDir.mkpath(".");
  const QString stamp =
      QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss");
  const QString path = logDir.filePath(QString("rec_%1.bin").arg(stamp));

  // protocolVersion is a forward-compat field; no wire-version constant
  // exists yet, so record 1 and let replay degrade gracefully.
  if (m_recorder.open(path, /*protocolVersion=*/1,
                      quint64(QDateTime::currentMSecsSinceEpoch()))) {
    m_logPanel->appendLog("[GCS] Recording telemetry → " + path);
  } else {
    m_logPanel->appendLog("[GCS] Could not open recording file: " + path);
  }
}

void MainWindow::stopRecording() {
  if (!m_recorder.isOpen()) return;
  const QString path = m_recorder.path();
  m_recorder.close();
  m_logPanel->appendLog("[GCS] Stopped recording → " + path);
}

void MainWindow::onSerialError(const QString &msg) {
  m_logPanel->appendLog("[ERROR] " + msg);
  if (m_statusBar) m_statusBar->setError(msg);
  setConnected(false);
}

void MainWindow::setConnected(bool on) {
  m_connected = on;

  // Page-level gating: any page whose actions only make sense with a
  // live serial link is told to disable its action surface here.
  // Read-only display pages (Motor / ControlLoop) are intentionally
  // left interactive so the last known data stays visible.
  if (m_calibrationWidget) m_calibrationWidget->setConnected(on);

  if (m_toolbar)   m_toolbar->setConnected(on, m_serial->currentPort());
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
    m_pktCount = 0;
    m_syncTimer->start(5000);
    onTimeSyncRequested();
  } else {
    m_armed = false;
    m_syncTimer->stop();
  }
}

void MainWindow::refreshConnectionPill() {
  if (!m_statusBar) return;
  // A real serial link wins; otherwise surface the in-app sim as SIM.
  if (m_connected) {
    m_statusBar->setConnectionStatus(true, m_serial->currentPort());
  } else if (m_simRunning) {
    m_statusBar->setConnectionStatus(true, QStringLiteral("SIM"));
  } else {
    m_statusBar->setConnectionStatus(false, QString());
  }
}

// ---------------------------------------------------------------------------
// Periodic UI update (10 Hz)
// ---------------------------------------------------------------------------

void MainWindow::onUiTimer() {
  static int tick = 0;
  tick++;

  // Update IMU panel with latest cached data
  m_imuPanel->updateImu(m_latestImu);

  // Throttled updates for numeric labels (update every 4 ticks = 5Hz)
  if (tick % 4 != 0) {
    // Still update packet count and live blinker every tick for smoothness
    if (m_statusBar) m_statusBar->setPacketCount(m_pktCount);
    updateLiveBlinker();
    return;
  }

  // Update attitude numeric labels (stable 5Hz update)
  auto fmtVal = [](float v) {
    return QString("%1°").arg(static_cast<double>(v), 7, 'f', 2);
  };
  auto fmtStd = [](float v) {
    return QString("± σ %1").arg(static_cast<double>(v), 6, 'f', 3);
  };

  m_rollLabel->setText(fmtVal(m_latestAtt.roll));
  m_pitchLabel->setText(fmtVal(m_latestAtt.pitch));
  m_yawLabel->setText(fmtVal(m_latestAtt.yaw));

  m_rollStd->setText(fmtStd(m_attStats[0].stdDev()));
  m_pitchStd->setText(fmtStd(m_attStats[1].stdDev()));
  m_yawStd->setText(fmtStd(m_attStats[2].stdDev()));

  if (m_statusBar) m_statusBar->setPacketCount(m_pktCount);
  updateLiveBlinker();
}

void MainWindow::updateLiveBlinker() {
  if (!m_toolbar) return;
  // In replay the pill shows a static REPLAY; don't let the heartbeat fade
  // fight it (Phase-1 1D).
  if (m_session.isReplay()) return;
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
}

void MainWindow::onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId) {
  ++m_pktCount;
  m_lastHbTime = QDateTime::currentMSecsSinceEpoch();

  // Flash the LIVE label bright; the UI tick will fade it back.
  if (m_toolbar) MainStatusBar::flashLive(m_toolbar->liveLabel());

  // Drone timestamp is 32-bit ms; signed subtraction yields the
  // shortest modular distance (handles wrap-around).
  const uint32_t gcs_now_32  = static_cast<uint32_t>(m_lastHbTime);
  const uint32_t drone_ts_32 = static_cast<uint32_t>(timestamp);
  const qint32 diff_ms = static_cast<qint32>(gcs_now_32 - drone_ts_32);

  if (m_statusBar) m_statusBar->showSyncDrift(diff_ms);
  statusBar()->showMessage(QString("Heartbeat from Device %1").arg(deviceId),
                           1000);
}

void MainWindow::onTimeSyncRequested() {
  if (!m_connected)
    return;

  uint32_t now = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch());
  uint8_t id = 42;

  // Binary PACKET_TYPE_HEARTBEAT
  // Header (8b) + Payload (0b) + CRC (4b) = 12 bytes
  QByteArray pkt;
  pkt.append(static_cast<char>(0x56)); // sync
  pkt.append(static_cast<char>(0x01)); // type 0 (heartbeat), ver 1
  pkt.append(static_cast<char>(0x00)); // length 0
  pkt.append(static_cast<char>(id));   // dev_id

  pkt.append(reinterpret_cast<const char *>(&now), 4);

  uint32_t crc =
      CRC32::calculate(reinterpret_cast<const uint8_t *>(pkt.constData()), 8);
  pkt.append(reinterpret_cast<const char *>(&crc), 4);

  sendToFc(pkt);
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
}  // namespace

void MainWindow::installShortcuts() {
  // Page navigation — Ctrl+1..8 map to QStackedWidget indices 0..7 in the
  // order pages were added in the ctor. Registered as positional "Go to
  // view N" commands so they show up in the (Phase-2) editor/palette like
  // any other command. These have no menu host, so the QAction is added to
  // the window to make its shortcut live. Bind sequentially; the count
  // stays <= the number of pages.
  const QStringList navKeys = {"Ctrl+1", "Ctrl+2", "Ctrl+3", "Ctrl+4",
                               "Ctrl+5", "Ctrl+6", "Ctrl+7", "Ctrl+8"};
  for (int i = 0; i < navKeys.size() && i < m_stackedWidget->count(); ++i) {
    const int idx = i;
    addAction(m_cmds->add(
        QString("view.go%1").arg(i + 1), QString("Go to View %1").arg(i + 1),
        "View", QKeySequence(navKeys[i]), CmdContext::Always,
        [this, idx] { m_stackedWidget->setCurrentIndex(idx); }));
  }

  // Ctrl+K — toggle connection (same path as a click on the toolbar's
  // Connect/Disconnect button).
  addAction(m_cmds->add("link.toggle", "Connect / Disconnect", "Link",
                        QKeySequence("Ctrl+K"), CmdContext::Always, [this] {
                          if (m_toolbar) m_toolbar->onConnectClicked();
                        }));

  // Ctrl+L — clear the log panel.
  addAction(m_cmds->add("log.clear", "Clear Log", "Log",
                        QKeySequence("Ctrl+L"), CmdContext::Always, [this] {
                          if (m_logPanel) m_logPanel->clearLog();
                        }));

  // F11 — toggle fullscreen.
  addAction(m_cmds->add("window.fullscreen", "Toggle Fullscreen", "Window",
                        QKeySequence(Qt::Key_F11), CmdContext::Always, [this] {
                          if (isFullScreen()) showNormal();
                          else showFullScreen();
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
  if (s.contains(kGeomKey))     restoreGeometry(s.value(kGeomKey).toByteArray());
  if (s.contains(kWinStateKey)) restoreState(s.value(kWinStateKey).toByteArray());
  if (m_topSplitter && s.contains(kTopSplitKey))
    m_topSplitter->restoreState(s.value(kTopSplitKey).toByteArray());
  if (m_vSplitter && s.contains(kVSplitKey))
    m_vSplitter->restoreState(s.value(kVSplitKey).toByteArray());

  // Restore last-used port / baud through the toolbar's typed accessors.
  // setPort handles both "matches a known item" and "custom path" cases.
  if (m_toolbar) {
    m_toolbar->setPort(s.value(kPortKey).toString());
    m_toolbar->setBaud(s.value(kBaudKey).toInt());
  }

  // Last page index. Default to home (0) on first run.
  if (m_stackedWidget && s.contains(kPageKey)) {
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
