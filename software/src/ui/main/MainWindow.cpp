#include "MainWindow.h"
#include "../core/SettingsManager.h"
#include "../core/Theme.h"
#include "../core/crc.h"

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QSettings>
#include <QShortcut>
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
  m_protocol = new DroneProtocol(this);
  m_uiTimer = new QTimer(this);
  m_syncTimer = new QTimer(this);
  m_elapsed.start();

  m_stackedWidget = new QStackedWidget(this);
  setCentralWidget(m_stackedWidget);

  // Wire serial → protocol → UI
  connect(m_serial, &SerialManager::dataReceived, m_protocol,
          &DroneProtocol::processData);

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

  // Load and apply persistent settings
  GcsSettings savedSettings;
  if (SettingsManager::load(savedSettings)) {
    m_settingsWidget->setSettings(savedSettings);
    // Trigger the actual logic changes
    m_syncTimer->setInterval(savedSettings.syncPeriodMs);
    m_imuPanel->setGraphWindow(savedSettings.graphWindowSec);
    m_imuPanel->setGraphDropout(savedSettings.graphDropoutRate);
    m_serial->setAutoReconnect(savedSettings.autoReconnect);
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
            if (m_serial && m_connected) {
              m_serial->write(data);
            }
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
#endif

  installShortcuts();
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

  QMenu *fileMenu = menu->addMenu("&File");

  m_homeAction =
      fileMenu->addAction("&Home Screen", this, &MainWindow::showHome);
  m_homeAction->setShortcut(QKeySequence("Ctrl+H"));

  m_analyzerAction = fileMenu->addAction("&Packet Analyzer", this,
                                         &MainWindow::showPacketAnalyzer);
  m_analyzerAction->setShortcut(QKeySequence("Ctrl+P"));

  QMenu *settingsMenu = menu->addMenu("&Settings");
  settingsMenu->addAction("&Configuration", this, &MainWindow::showSettings);

  QMenu *windowMenu = menu->addMenu("&Window");
  windowMenu->addAction("&Channels", this, &MainWindow::showRcMonitor);
  windowMenu->addAction("&Calibration", this, &MainWindow::showCalibration);
  windowMenu->addAction("&Motor Status", this, &MainWindow::showMotorStatus);
  windowMenu->addAction("&Control Loop", this,
                        &MainWindow::showControlLoopPlot);
#ifdef NAVIGATOR_HAS_SITL
  windowMenu->addAction("Si&mulator", this, &MainWindow::showSimulator);
#endif

  fileMenu->addSeparator();

  QAction *exitAction = fileMenu->addAction("E&xit");
  exitAction->setShortcut(QKeySequence::Quit);
  connect(exitAction, &QAction::triggered, this, &MainWindow::close);
}

// ---------------------------------------------------------------------------
// Toolbar intent slots — toolbar emits, MainWindow drives serial.
// ---------------------------------------------------------------------------

void MainWindow::onConnectRequested(const QString &port, int baud) {
  if (port.isEmpty()) {
    m_logPanel->appendLog("[GCS] No port specified");
    return;
  }
  if (m_serial->open(port, baud)) {
    persistPortBaud();
  }
}

void MainWindow::onDisconnectRequested() {
  if (m_serial) m_serial->close();
}

void MainWindow::onArmClicked() {
  // Intentional no-op until FR-TX-02 (Phase-0 item 0a) defines the
  // ARM packet with the firmware. The button stays disabled in the
  // toolbar, so this slot is only reachable via keyboard shortcut or
  // a future state change. Log so the action is observable.
  m_logPanel->appendLog(
      "[GCS] ARM/DISARM not implemented — pending firmware packet "
      "definition (FR-TX-02)");
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
  if (m_statusBar) m_statusBar->setConnectionStatus(on, m_serial->currentPort());

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

  m_serial->write(pkt);
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
  // Page navigation — Ctrl+1..7 maps to QStackedWidget indices in the
  // order they were added in the ctor (home, packet analyzer, rc,
  // settings, calibration, motor status, control loop, simulator).
  // Bind sequentially; the count must stay <= the number of pages.
  const QList<QKeyCombination> keys = {
      Qt::CTRL | Qt::Key_1, Qt::CTRL | Qt::Key_2, Qt::CTRL | Qt::Key_3,
      Qt::CTRL | Qt::Key_4, Qt::CTRL | Qt::Key_5, Qt::CTRL | Qt::Key_6,
      Qt::CTRL | Qt::Key_7, Qt::CTRL | Qt::Key_8,
  };
  for (int i = 0; i < keys.size() && i < m_stackedWidget->count(); ++i) {
    auto *sc = new QShortcut(QKeySequence(keys[i]), this);
    const int idx = i;
    connect(sc, &QShortcut::activated, this,
            [this, idx] { m_stackedWidget->setCurrentIndex(idx); });
  }

  // Ctrl+K — toggle connection (same path as a click on the toolbar's
  // Connect/Disconnect button).
  auto *scConn = new QShortcut(
      QKeySequence(QKeyCombination(Qt::CTRL, Qt::Key_K)), this);
  connect(scConn, &QShortcut::activated, this,
          [this] { if (m_toolbar) m_toolbar->onConnectClicked(); });

  // Ctrl+L — clear the log panel.
  auto *scClr = new QShortcut(
      QKeySequence(QKeyCombination(Qt::CTRL, Qt::Key_L)), this);
  connect(scClr, &QShortcut::activated, this, [this] {
    if (m_logPanel) m_logPanel->clearLog();
  });

  // F11 — toggle fullscreen.
  auto *scFs = new QShortcut(QKeySequence(Qt::Key_F11), this);
  connect(scFs, &QShortcut::activated, this, [this] {
    if (isFullScreen()) showNormal();
    else showFullScreen();
  });
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
