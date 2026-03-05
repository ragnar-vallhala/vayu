#include "MainWindow.h"

#include <QAction>
#include <QApplication>
#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
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
  m_elapsed.start();

  // Wire serial → protocol → UI
  connect(m_serial, &SerialManager::packetReceived, m_protocol,
          &DroneProtocol::parseLine);

  connect(m_protocol, &DroneProtocol::imuReceived, this,
          &MainWindow::onImuReceived);
  connect(m_protocol, &DroneProtocol::attitudeReceived, this,
          &MainWindow::onAttitudeReceived);
  connect(m_protocol, &DroneProtocol::logReceived, this,
          &MainWindow::onLogReceived);
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

  // Periodic status refresh at 20 Hz for smoother fade
  connect(m_uiTimer, &QTimer::timeout, this, &MainWindow::onUiTimer);
  m_uiTimer->start(50);

  buildUi();
  buildToolBar();
  onRefreshPorts(); // populate port list on startup
  setConnected(false);
}

// ---------------------------------------------------------------------------
// UI Construction
// ---------------------------------------------------------------------------

void MainWindow::buildUi() {
  auto *central = new QWidget(this);
  auto *rootVBox = new QVBoxLayout(central);
  rootVBox->setSpacing(8);
  rootVBox->setContentsMargins(8, 8, 8, 8);

  // ---- Top row: attitude + attitude labels + IMU panel ----
  auto *topSplitter = new QSplitter(Qt::Horizontal, central);

  // ---- Attitude group ----
  auto *attGroup = new QGroupBox("Attitude", central);
  auto *attLayout = new QVBoxLayout(attGroup);

  m_attitude = new AttitudeWidget(attGroup);
  attLayout->addWidget(m_attitude, 1);

  // Numeric roll/pitch/yaw labels
  auto *numGrid = new QGridLayout;
  const char *lblNames[] = {"Roll", "Pitch", "Yaw"};
  QLabel **lblPtrs[] = {&m_rollLabel, &m_pitchLabel, &m_yawLabel};
  const char *colors[] = {"#FF6B6B", "#4ECDC4", "#FFE66D"};

  for (int i = 0; i < 3; ++i) {
    auto *title = new QLabel(QString("<b>%1</b>").arg(lblNames[i]), attGroup);
    title->setStyleSheet(QString("color: %1; font-size: 12px;").arg(colors[i]));
    title->setAlignment(Qt::AlignRight);

    *lblPtrs[i] = new QLabel("  0.00°", attGroup);
    (*lblPtrs[i])
        ->setStyleSheet(QString("color: %1; font-size: 18px; font-weight: "
                                "bold; font-family: Monospace;")
                            .arg(colors[i]));
    (*lblPtrs[i])->setAlignment(Qt::AlignLeft);

    numGrid->addWidget(title, i, 0);
    numGrid->addWidget(*lblPtrs[i], i, 1);
  }
  attLayout->addLayout(numGrid);
  topSplitter->addWidget(attGroup);

  // ---- IMU Panel ----
  m_imuPanel = new ImuPanel(central);
  topSplitter->addWidget(m_imuPanel);

  topSplitter->setStretchFactor(0, 1);
  topSplitter->setStretchFactor(1, 2);

  // ---- Bottom: Log panel ----
  m_logPanel = new LogPanel(central);
  m_logPanel->setMinimumHeight(180);

  // ---- Vertical splitter: attitude+imu / log ----
  auto *vSplitter = new QSplitter(Qt::Vertical, central);
  vSplitter->addWidget(topSplitter);
  vSplitter->addWidget(m_logPanel);
  vSplitter->setStretchFactor(0, 3);
  vSplitter->setStretchFactor(1, 1);

  rootVBox->addWidget(vSplitter);
  setCentralWidget(central);

  // ---- Status bar ----
  m_connStatus = new QLabel("  ● Disconnected  ", this);
  m_connStatus->setStyleSheet("color: #E06C75; font-weight: bold;");
  m_pktStatus = new QLabel("  Packets: 0  ", this);

  statusBar()->addPermanentWidget(m_connStatus);
  statusBar()->addPermanentWidget(m_pktStatus);
  statusBar()->setStyleSheet("background: #1A1D27; color: #ABB2BF;");
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

// ---------------------------------------------------------------------------
// Data Slots
// ---------------------------------------------------------------------------

void MainWindow::onImuReceived(const ImuData &data) {
  m_latestImu = data;
  ++m_pktCount;
}

void MainWindow::onAttitudeReceived(const AttitudeData &data) {
  m_latestAtt = data;
  m_attitude->setAttitude(data);
  ++m_pktCount;
}

void MainWindow::onLogReceived(const QString &msg) {
  m_logPanel->appendLog(msg);
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
  } else {
    m_connected = false;
    m_armed = false;
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
  // Update IMU panel with latest cached data
  m_imuPanel->updateImu(m_latestImu);

  // Update attitude numeric labels
  auto fmt = [](float v) { return QString("%1°").arg(v, 7, 'f', 2); };
  m_rollLabel->setText(fmt(m_latestAtt.roll));
  m_pitchLabel->setText(fmt(m_latestAtt.pitch));
  m_yawLabel->setText(fmt(m_latestAtt.yaw));

  // Update packet counter in status bar
  m_pktStatus->setText(QString("  Packets: %1  ").arg(m_pktCount));

  // Update LIVE blinker decay
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 elapsed = now - m_lastHbTime;

  if (elapsed < 2000) {
    // Exponential decay: e^(-5 * t / 2000)
    double factor = std::exp(-5.0 * elapsed / 2000.0);
    int alpha = static_cast<int>(255 * factor);
    int green = static_cast<int>(150 * factor); // Darker green base

    m_liveLabel->setStyleSheet(
        QString(
            "background: rgba(40, %1, 40, %2); color: rgba(255, 255, 255, %2); "
            "border-radius: 4px; font-weight: bold; padding: 2px 8px; "
            "border: 1px solid rgba(152, 195, 121, %2);")
            .arg(green + 50)
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
  // Optional: Update UI with heartbeat info
  statusBar()->showMessage(
      QString("Heartbeat from Device %1 (TS: %2)").arg(deviceId).arg(timestamp),
      2000);
}

void MainWindow::onTimeSyncRequested() {
  if (!m_connected)
    return;

  uint64_t now = QDateTime::currentMSecsSinceEpoch();
  uint8_t id = 42; // Example fixed Device ID or could be configurable
  QByteArray resp = QString("$TIME_SET,%1,%2\n").arg(now).arg(id).toLatin1();
  m_serial->write(resp);
  m_logPanel->appendLog(
      QString("[GCS] Sent Time Sync: %1, ID: %2").arg(now).arg(id));
}
