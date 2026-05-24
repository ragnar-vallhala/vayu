#include "MainWindow.h"
#include "../core/SettingsManager.h"
#include "../core/crc.h"

#include <QAction>
#include <QApplication>
#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyleFactory>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  setWindowTitle("Vayu GCS — Ground Control Station");
  setMinimumSize(1100, 700);
  resize(1300, 820);

  applyDarkTheme();

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

  // Load and apply persistent settings
  GcsSettings savedSettings;
  if (SettingsManager::load(savedSettings)) {
    m_settingsWidget->setSettings(savedSettings);
    // Trigger the actual logic changes
    m_syncTimer->setInterval(savedSettings.syncPeriodMs);
    m_imuPanel->setGraphWindow(savedSettings.graphWindowSec);
    m_imuPanel->setGraphDropout(savedSettings.graphDropoutRate);
  }

  buildMenuBar();
  buildToolBar();
  onRefreshPorts(); // populate port list on startup
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

  // Build Simulator (host SITL launcher + monitor)
  m_simulatorWidget = new SimulatorWidget(this);
  m_stackedWidget->addWidget(m_simulatorWidget);
  connect(m_simulatorWidget, &SimulatorWidget::backToHomeRequested, this,
          &MainWindow::showHome);

  showHome();
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
  m_stackedWidget->setCurrentWidget(m_simulatorWidget);
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
  auto *topSplitter = new QSplitter(Qt::Horizontal, m_homeWidget);

  // ---- Attitude group ----
  auto *attGroup = new QGroupBox("Attitude", m_homeWidget);
  auto *attLayout = new QVBoxLayout(attGroup);

  // Toggle button at the top right of the group
  auto *headerLayout = new QHBoxLayout();
  headerLayout->addStretch();
  auto *toggleBtn = new QPushButton("3D VIEW", attGroup);
  toggleBtn->setCheckable(true);
  toggleBtn->setFixedSize(70, 22);
  toggleBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; "
      "border-radius: 4px; font-weight: bold; font-size: 10px; }"
      "QPushButton:checked { background: #61AFEF; color: #21252B; }");
  headerLayout->addWidget(toggleBtn);
  attLayout->addLayout(headerLayout);

  m_attStack = new QStackedWidget(attGroup);
  m_attitude = new AttitudeWidget(m_attStack);
  m_drone3d = new Drone3DWidget(m_attStack);
  m_attStack->addWidget(m_attitude);
  m_attStack->addWidget(m_drone3d);
  attLayout->addWidget(m_attStack, 1);

  connect(toggleBtn, &QPushButton::toggled, this, &MainWindow::onToggle3d);

  m_statusLabel = new QLabel("DISCONNECTED", attGroup);
  m_statusLabel->setAlignment(Qt::AlignCenter);
  m_statusLabel->setStyleSheet(
      "font-size: 18px; font-weight: bold; color: #E06C75; "
      "background: #1A1D27; border: 1px solid #2A3347; "
      "border-radius: 4px; padding: 4px; margin-bottom: 8px;");
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
  auto *vSplitter = new QSplitter(Qt::Vertical, m_homeWidget);
  vSplitter->addWidget(topSplitter);
  vSplitter->addWidget(m_logPanel);
  vSplitter->setStretchFactor(0, 3);
  vSplitter->setStretchFactor(1, 1);

  rootVBox->addWidget(vSplitter);
  m_stackedWidget->addWidget(m_homeWidget);

  // ---- Status bar ----
  m_connStatus = new QLabel("  ● Disconnected  ", this);
  m_connStatus->setStyleSheet("color: #E06C75; font-weight: bold;");
  m_syncStatus = new QLabel("  Diff: ---  ", this);
  m_syncStatus->setStyleSheet("color: #ABB2BF; font-family: Monospace;");
  m_pktStatus = new QLabel("  Packets: 0  ", this);

  statusBar()->addPermanentWidget(m_connStatus);
  statusBar()->addPermanentWidget(m_syncStatus);
  statusBar()->addPermanentWidget(m_pktStatus);
  statusBar()->setStyleSheet("background: #1A1D27; color: #ABB2BF;");
}

void MainWindow::buildMenuBar() {
  QMenuBar *menu = menuBar();
  menu->setStyleSheet(
      "QMenuBar { background: #1A1D27; color: #ABB2BF; "
      "border-bottom: 1px solid #2A3347; }"
      "QMenuBar::item { background: transparent; padding: 4px 10px; }"
      "QMenuBar::item:selected { background: #2A3347; color: #FFFFFF; }"
      "QMenu { background: #21252B; color: #ABB2BF; border: 1px solid #3E4452; "
      "}"
      "QMenu::item { padding: 4px 24px 4px 20px; }"
      "QMenu::item:selected { background: #3E4452; color: #FFFFFF; }");

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
  windowMenu->addAction("Si&mulator", this, &MainWindow::showSimulator);

  fileMenu->addSeparator();

  QAction *exitAction = fileMenu->addAction("E&xit");
  exitAction->setShortcut(QKeySequence::Quit);
  connect(exitAction, &QAction::triggered, this, &MainWindow::close);
}

void MainWindow::buildToolBar() {
  auto *tb = addToolBar("Main");
  tb->setMovable(false);
  tb->setStyleSheet(
      "QToolBar { background: #1A1D27; border-bottom: 1px solid #2A3347; "
      "spacing: 6px; }"
      "QToolBar QLabel { color: #ABB2BF; }"
      "QComboBox { background: #21252B; color: #ABB2BF; border: 1px solid "
      "#3E4452; "
      "            padding: 2px 6px; border-radius: 4px; min-width: 110px; }"
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; "
      "              padding: 4px 14px; border-radius: 4px; font-weight: bold; "
      "}"
      "QPushButton:hover { background: #3E4452; }"
      "QPushButton:pressed { background: #4B5263; }");

  // Logo / title
  auto *title = new QLabel(" ✈  <b>Vayu GCS</b> ", this);
  title->setStyleSheet("color: #61AFEF; font-size: 16px; padding: 0 8px;");
  tb->addWidget(title);

  tb->addSeparator();

  // Port
  tb->addWidget(new QLabel(" Port: ", this));
  m_portCombo = new QComboBox(this);
  tb->addWidget(m_portCombo);

  // Refresh ports
  auto *refreshBtn = new QPushButton("⟳", this);
  refreshBtn->setToolTip("Refresh port list");
  refreshBtn->setFixedWidth(32);
  connect(refreshBtn, &QPushButton::clicked, this, &MainWindow::onRefreshPorts);
  tb->addWidget(refreshBtn);

  tb->addSeparator();

  // Baud
  tb->addWidget(new QLabel(" Baud: ", this));
  m_baudCombo = new QComboBox(this);
  const QList<int> bauds = {9600,   19200,  38400,  57600,
                            115200, 230400, 460800, 921600};
  for (int b : bauds)
    m_baudCombo->addItem(QString::number(b), b);
  m_baudCombo->setCurrentIndex(4); // 115200 default
  tb->addWidget(m_baudCombo);

  tb->addSeparator();

  // Connect / Disconnect
  m_connectBtn = new QPushButton("Connect", this);
  m_connectBtn->setStyleSheet("QPushButton { background: #3A5F3A; color: "
                              "#98C379; border: 1px solid #5A8F5A; "
                              "              padding: 4px 18px; border-radius: "
                              "4px; font-weight: bold; }"
                              "QPushButton:hover { background: #4A7A4A; }");
  connect(m_connectBtn, &QPushButton::clicked, this,
          &MainWindow::onConnectClicked);
  tb->addWidget(m_connectBtn);

  tb->addSeparator();

  // Arm / Disarm
  m_armBtn = new QPushButton("ARM", this);
  m_armBtn->setEnabled(false);
  m_armBtn->setStyleSheet("QPushButton { background: #5A3A3A; color: #E06C75; "
                          "border: 1px solid #8F5A5A; "
                          "              padding: 4px 18px; border-radius: "
                          "4px; font-weight: bold; }"
                          "QPushButton:hover { background: #7A4A4A; }"
                          "QPushButton:disabled { color: #4A4A4A; "
                          "border-color: #3A3A3A; background: #252525; }");
  connect(m_armBtn, &QPushButton::clicked, this, &MainWindow::onArmClicked);
  tb->addWidget(m_armBtn);

  tb->addSeparator();

  // RC Monitor
  auto *rcBtn = new QPushButton("RC", this);
  rcBtn->setToolTip("Open RC Channels Monitor");
  rcBtn->setFixedWidth(40);
  rcBtn->setStyleSheet("QPushButton { font-weight: bold; }");
  connect(rcBtn, &QPushButton::clicked, this, &MainWindow::showRcMonitor);
  tb->addWidget(rcBtn);

  tb->addSeparator();

  // Calibration
  auto *calBtn = new QPushButton("CALIB", this);
  calBtn->setToolTip("Open Calibration IMU");
  calBtn->setFixedWidth(60);
  calBtn->setStyleSheet("QPushButton { font-weight: bold; }");
  connect(calBtn, &QPushButton::clicked, this, &MainWindow::showCalibration);
  tb->addWidget(calBtn);

  tb->addSeparator();

  // LIVE blinker
  m_liveLabel = new QLabel(" LIVE ", this);
  m_liveLabel->setAlignment(Qt::AlignCenter);
  m_liveLabel->setStyleSheet(
      "background: #1A1D27; color: #ABB2BF; border-radius: 4px; font-weight: "
      "bold; padding: 2px 8px; border: 1px solid #2A3347;");
  tb->addWidget(m_liveLabel);
}

void MainWindow::applyDarkTheme() {
  qApp->setStyle(QStyleFactory::create("Fusion"));

  QPalette dark;
  dark.setColor(QPalette::Window, QColor(26, 29, 39));
  dark.setColor(QPalette::WindowText, QColor(171, 178, 191));
  dark.setColor(QPalette::Base, QColor(19, 20, 27));
  dark.setColor(QPalette::AlternateBase, QColor(30, 33, 43));
  dark.setColor(QPalette::ToolTipBase, QColor(40, 44, 58));
  dark.setColor(QPalette::ToolTipText, QColor(171, 178, 191));
  dark.setColor(QPalette::Text, QColor(171, 178, 191));
  dark.setColor(QPalette::Button, QColor(40, 44, 58));
  dark.setColor(QPalette::ButtonText, QColor(171, 178, 191));
  dark.setColor(QPalette::BrightText, Qt::red);
  dark.setColor(QPalette::Link, QColor(97, 175, 239));
  dark.setColor(QPalette::Highlight, QColor(97, 175, 239));
  dark.setColor(QPalette::HighlightedText, Qt::black);
  qApp->setPalette(dark);

  qApp->setStyleSheet(
      "QGroupBox { border: 1px solid #2A3347; border-radius: 6px; "
      "            margin-top: 8px; padding-top: 6px; color: #7090C8; "
      "font-weight: bold; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 "
      "4px; }"
      "QSplitter::handle { background: #2A3347; }");
}

// ---------------------------------------------------------------------------
// Toolbar Slots
// ---------------------------------------------------------------------------

void MainWindow::onRefreshPorts() {
  m_portCombo->clear();
  const QStringList ports = SerialManager::availablePorts();
  if (ports.isEmpty()) {
    m_portCombo->addItem("(no ports found)");
  } else {
    m_portCombo->addItems(ports);
  }
}

void MainWindow::onConnectClicked() {
  if (m_connected) {
    m_serial->close();
  } else {
    const QString port = m_portCombo->currentText();
    const int baud = m_baudCombo->currentData().toInt();
    m_serial->open(port, baud);
  }
}

void MainWindow::onArmClicked() {
  m_armed = !m_armed;
  if (m_armed) {
    m_armBtn->setText("DISARM");
    m_armBtn->setStyleSheet("QPushButton { background: #8F5A5A; color: "
                            "#FF8888; border: 1px solid #BF6060; "
                            "              padding: 4px 18px; border-radius: "
                            "4px; font-weight: bold; }");
    m_logPanel->appendLog("[GCS] ARM command sent");
    // TODO: send ARM command over serial when protocol is defined
  } else {
    m_armBtn->setText("ARM");
    m_armBtn->setStyleSheet("QPushButton { background: #5A3A3A; color: "
                            "#E06C75; border: 1px solid #8F5A5A; "
                            "              padding: 4px 18px; border-radius: "
                            "4px; font-weight: bold; }");
    m_logPanel->appendLog("[GCS] DISARM command sent");
  }
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
  m_connStatus->setText(QString("  ● Error: %1  ").arg(msg));
  m_connStatus->setStyleSheet("color: #E06C75; font-weight: bold;");
  setConnected(false);
}

void MainWindow::setConnected(bool on) {
  m_connected = on;

  if (on) {
    m_connectBtn->setText("Disconnect");
    m_connectBtn->setStyleSheet("QPushButton { background: #5A3A3A; color: "
                                "#E06C75; border: 1px solid #8F5A5A; "
                                "              padding: 4px 18px; "
                                "border-radius: 4px; font-weight: bold; }"
                                "QPushButton:hover { background: #7A4A4A; }");
    m_connStatus->setText(
        QString("  ● Connected: %1  ").arg(m_serial->currentPort()));
    m_connStatus->setStyleSheet("color: #98C379; font-weight: bold;");
    m_armBtn->setEnabled(true);
    m_portCombo->setEnabled(false);
    m_baudCombo->setEnabled(false);
    m_pktCount = 0;
    m_syncTimer->start(5000);
    onTimeSyncRequested();
  } else {
    m_connected = false;
    m_armed = false;
    m_syncTimer->stop();
    m_connectBtn->setText("Connect");
    m_connectBtn->setStyleSheet("QPushButton { background: #3A5F3A; color: "
                                "#98C379; border: 1px solid #5A8F5A; "
                                "              padding: 4px 18px; "
                                "border-radius: 4px; font-weight: bold; }"
                                "QPushButton:hover { background: #4A7A4A; }");
    m_armBtn->setText("ARM");
    m_armBtn->setEnabled(false);
    m_connStatus->setText("  ● Disconnected  ");
    m_connStatus->setStyleSheet("color: #E06C75; font-weight: bold;");
    m_portCombo->setEnabled(true);
    m_baudCombo->setEnabled(true);
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
    m_pktStatus->setText(QString("  Packets: %1  ").arg(m_pktCount));
    updateLiveBlinker();
    return;
  }

  // Update attitude numeric labels
  // Update attitude numeric labels (stable 20Hz update)
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

  // Update packet counter in status bar
  m_pktStatus->setText(QString("  Packets: %1  ").arg(m_pktCount));

  updateLiveBlinker();
}

void MainWindow::updateLiveBlinker() {
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 elapsed = now - m_lastHbTime;

  if (m_lastHbTime == 0) {
    // Never received a heartbeat yet — show dark
    m_liveLabel->setStyleSheet(
        "background: #1A1D27; color: #4B5263; border-radius: 4px; "
        "font-weight: bold; padding: 2px 8px; border: 1px solid #2A3347;");
  } else if (elapsed < 150) {
    // Hold bright for 150ms before fading
    // stylesheet already set in onHeartbeatReceived, leave it
  } else if (elapsed < 2000) {
    double factor = std::exp(-4.0 * (elapsed - 150) / 1850.0);
    int alpha = static_cast<int>(255 * factor);
    int green = static_cast<int>(106 * factor + 29); // 29 minimum
    m_liveLabel->setStyleSheet(
        QString("background: rgba(30, %1, 30, 200); "
                "color: rgba(255, 255, 255, %2); "
                "border-radius: 4px; font-weight: bold; padding: 2px 8px; "
                "border: 1px solid rgba(152, 195, 121, %2);")
            .arg(green)
            .arg(alpha));
  } else {
    m_liveLabel->setStyleSheet(
        "background: #1A1D27; color: #4B5263; border-radius: 4px; "
        "font-weight: bold; padding: 2px 8px; border: 1px solid #2A3347;");
  }
}

void MainWindow::onHeartbeatReceived(uint64_t timestamp, uint8_t deviceId) {
  ++m_pktCount;
  m_lastHbTime = QDateTime::currentMSecsSinceEpoch();
  // Flash immediately bright on receipt
  m_liveLabel->setStyleSheet(
      "background: #2D6A2D; color: #FFFFFF; border-radius: 4px; "
      "font-weight: bold; padding: 2px 8px; "
      "border: 1px solid #98C379;");
  // Calculate time difference (drone timestamp is 32-bit ms)
  uint32_t gcs_now_32 = static_cast<uint32_t>(m_lastHbTime);
  uint32_t drone_ts_32 = static_cast<uint32_t>(timestamp);

  // Use signed 32-bit to get the shortest modular distance (handles
  // wrap-around)
  int32_t diff_ms = static_cast<int32_t>(gcs_now_32 - drone_ts_32);

  QString diffStr =
      QString("  Diff: %1%2ms  ").arg(diff_ms >= 0 ? "+" : "").arg(diff_ms);

  m_syncStatus->setText(diffStr);

  // Color grade based on error
  int32_t absDiff = std::abs(diff_ms);
  if (absDiff < 20) {
    m_syncStatus->setStyleSheet(
        "color: #98C379; font-weight: bold; font-family: Monospace;"); // Green
  } else if (absDiff < 100) {
    m_syncStatus->setStyleSheet("color: #D19A66; font-weight: bold; "
                                "font-family: Monospace;"); // Yellow/Orange
  } else {
    m_syncStatus->setStyleSheet(
        "color: #E06C75; font-weight: bold; font-family: Monospace;"); // Red
  }

  // Optional: Update UI with heartbeat info
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
