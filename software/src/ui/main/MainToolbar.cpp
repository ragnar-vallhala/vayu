#include "MainToolbar.h"

#include "SerialManager.h"
#include "core/ui/Buttons.h"

#include <QFile>
#include <QLineEdit>
#include <QList>
#include <QStringList>
#include <QStyle>

namespace {

// Flip the connect button's object-name role and re-polish so dark.qss
// recolours it instantly. Same trick MainWindow used before the split.
void repolish(QWidget *w, const QString &name) {
  w->setObjectName(name);
  w->style()->unpolish(w);
  w->style()->polish(w);
  w->update();
}

}  // namespace

MainToolbar::MainToolbar(QWidget *parent) : QToolBar(parent) {
  setObjectName("MainToolbar");
  setMovable(false);
  buildContent();
}

void MainToolbar::buildContent() {
  // Brand label
  auto *title = new QLabel(" ✈  <b>Vayu GCS</b> ", this);
  title->setObjectName("BrandLabel");
  addWidget(title);

  addSeparator();

  // Transport selector — explicit Serial vs UDP (WiFi bridge) so the right
  // inputs are shown for each, instead of overloading the Port field.
  addWidget(new QLabel(" Link: ", this));
  m_transportCombo = new QComboBox(this);
  m_transportCombo->addItem("Serial");
  m_transportCombo->addItem("UDP");
  m_transportCombo->setToolTip(
      tr("Telemetry transport: Serial (USB/UART) or UDP (the WiFi bridge)."));
  connect(m_transportCombo, &QComboBox::currentIndexChanged, this,
          &MainToolbar::onTransportChanged);
  addWidget(m_transportCombo);

  addSeparator();

  // ---- Serial inputs (shown when Link = Serial) ----
  // Port — editable so custom paths (e.g. /dev/pts/3 for SITL) work. Each
  // item's userData carries the bare device path.
  m_serialActions << addWidget(new QLabel(" Port: ", this));
  m_portCombo = new QComboBox(this);
  m_portCombo->setEditable(true);
  m_portCombo->setInsertPolicy(QComboBox::NoInsert);
  m_portCombo->lineEdit()->setPlaceholderText("(custom path… e.g. /dev/pts/3)");
  m_portCombo->setToolTip(
      tr("Serial device path. Editable — type a custom path "
         "(e.g. /dev/pts/3 for a SITL pty)."));
  m_serialActions << addWidget(m_portCombo);

  m_refreshBtn = new ui::GhostButton("⟳", this);
  m_refreshBtn->setToolTip(tr("Refresh port list"));
  m_refreshBtn->setFixedWidth(32);
  connect(m_refreshBtn, &QPushButton::clicked, this, &MainToolbar::refreshPorts);
  m_serialActions << addWidget(m_refreshBtn);

  m_serialActions << addSeparator();

  m_serialActions << addWidget(new QLabel(" Baud: ", this));
  m_baudCombo = new QComboBox(this);
  const QList<int> bauds = {9600,   19200,  38400,  57600,
                            115200, 230400, 460800, 921600};
  for (int b : bauds) m_baudCombo->addItem(QString::number(b), b);
  m_baudCombo->setCurrentIndex(4);  // 115200 default
  m_baudCombo->setToolTip(tr("Baud rate. Must match firmware UART config."));
  m_serialActions << addWidget(m_baudCombo);

  // ---- UDP inputs (shown when Link = UDP) ----
  m_udpActions << addWidget(new QLabel(" UDP port: ", this));
  m_udpPortEdit = new QLineEdit("14555", this);
  m_udpPortEdit->setFixedWidth(70);
  m_udpPortEdit->setToolTip(
      tr("UDP port to listen on — matches the ESP8266 bridge (default 14555)."));
  m_udpActions << addWidget(m_udpPortEdit);

  addSeparator();

  onTransportChanged();  // set initial input visibility (Serial)

  // Connect / Disconnect — Success colour flips to Danger on connect
  // via repolish() in setConnected().
  m_connectBtn = new ui::SuccessButton(tr("Connect"), this);
  m_connectBtn->setToolTip(
      tr("Connect over the selected link (Serial port or UDP) — Ctrl+K."));
  connect(m_connectBtn, &QPushButton::clicked, this,
          &MainToolbar::onConnectClicked);
  addWidget(m_connectBtn);

  addSeparator();

  // ARM / DISARM — enabled by MainWindow once connected (FR-TX-02).
  m_armBtn = new ui::DangerButton(tr("ARM"), this);
  m_armBtn->setEnabled(false);
  m_armBtn->setToolTip(tr("ARM / DISARM the airframe (CMD_ARM / CMD_DISARM). "
                          "Lower the throttle before arming."));
  connect(m_armBtn, &QPushButton::clicked, this, &MainToolbar::armClicked);
  addWidget(m_armBtn);

  addSeparator();

  auto *rcBtn = new ui::GhostButton(tr("RC"), this);
  rcBtn->setToolTip(tr("Open RC Channels Monitor"));
  rcBtn->setFixedWidth(40);
  connect(rcBtn, &QPushButton::clicked, this, &MainToolbar::showRcRequested);
  addWidget(rcBtn);

  addSeparator();

  auto *calBtn = new ui::GhostButton(tr("CALIB"), this);
  calBtn->setToolTip(tr("Open IMU Calibration"));
  calBtn->setFixedWidth(60);
  connect(calBtn, &QPushButton::clicked, this,
          &MainToolbar::showCalibRequested);
  addWidget(calBtn);

  addSeparator();

  // LIVE heartbeat blinker. Styling-by-objectName lives in dark.qss;
  // the heartbeat handler repaints the active-state stylesheet inline.
  m_liveLabel = new QLabel(" LIVE ", this);
  m_liveLabel->setObjectName("LiveBlinker");
  m_liveLabel->setAlignment(Qt::AlignCenter);
  m_liveLabel->setToolTip(tr("Heartbeat indicator — bright on each rx, "
                             "fades when no packet seen."));
  addWidget(m_liveLabel);
}

// ---------------------------------------------------------------------------

void MainToolbar::refreshPorts() {
  // Preserve whatever the user has typed (e.g. a custom /dev/pts/N) so
  // refreshing the auto-detected list doesn't clobber a SITL pty path.
  const QString currentText = m_portCombo->currentText();

  m_portCombo->clear();
  const QStringList ports = SerialManager::availablePorts();
  for (const QString &p : ports) m_portCombo->addItem(p, p);

  // Auto-offer the SITL pty if it's been advertised by sim_host.
  QFile ptyAdv("/tmp/vayu_uart2_pty");
  if (ptyAdv.exists() && ptyAdv.open(QIODevice::ReadOnly | QIODevice::Text)) {
    const QString ptyPath = QString::fromUtf8(ptyAdv.readAll()).trimmed();
    if (!ptyPath.isEmpty() && !ports.contains(ptyPath))
      m_portCombo->addItem(ptyPath + "  (SITL UART2)", ptyPath);
  }

  if (m_portCombo->count() == 0)
    m_portCombo->addItem("(no ports found)", QString());

  if (!currentText.isEmpty()) m_portCombo->setEditText(currentText);
}

void MainToolbar::onConnectClicked() {
  if (m_connected) {
    emit disconnectRequested();
    return;
  }
  // UDP transport: hand MainWindow a "udp:<port>" string (it routes those to
  // the UdpManager); baud is irrelevant.
  if (m_transportCombo && m_transportCombo->currentIndex() == 1) {
    QString p = m_udpPortEdit ? m_udpPortEdit->text().trimmed() : QString();
    if (p.isEmpty()) p = "14555";
    emit connectRequested("udp:" + p, 0);
    return;
  }
  QString port = currentPort();
  if (port.isEmpty() || port.startsWith('(')) return;
  emit connectRequested(port, currentBaud());
}

void MainToolbar::onTransportChanged() {
  const bool udp = m_transportCombo && m_transportCombo->currentIndex() == 1;
  for (QAction *a : m_serialActions) a->setVisible(!udp);
  for (QAction *a : m_udpActions) a->setVisible(udp);
}

// ---------------------------------------------------------------------------

void MainToolbar::setConnected(bool on, const QString &portLabel) {
  Q_UNUSED(portLabel);
  m_connected = on;
  if (on) {
    m_connectBtn->setText(tr("Disconnect"));
    repolish(m_connectBtn, "DangerButton");
    m_portCombo->setEnabled(false);
    m_baudCombo->setEnabled(false);
    if (m_transportCombo) m_transportCombo->setEnabled(false);
    if (m_udpPortEdit) m_udpPortEdit->setEnabled(false);
  } else {
    m_connectBtn->setText(tr("Connect"));
    repolish(m_connectBtn, "SuccessButton");
    m_armBtn->setText(tr("ARM"));
    m_armBtn->setEnabled(false);
    m_portCombo->setEnabled(true);
    m_baudCombo->setEnabled(true);
    if (m_transportCombo) m_transportCombo->setEnabled(true);
    if (m_udpPortEdit) m_udpPortEdit->setEnabled(true);
  }
}

void MainToolbar::setSerialControlsEnabled(bool enabled) {
  if (m_refreshBtn) m_refreshBtn->setEnabled(enabled);
  if (m_connectBtn) m_connectBtn->setEnabled(enabled);
  // Port/baud stay locked whenever a serial link is up (setConnected), so
  // only re-enable them here if we're enabling AND not connected.
  const bool combos = enabled && !m_connected;
  if (m_portCombo) m_portCombo->setEnabled(combos);
  if (m_baudCombo) m_baudCombo->setEnabled(combos);
  if (m_transportCombo) m_transportCombo->setEnabled(combos);
  if (m_udpPortEdit) m_udpPortEdit->setEnabled(combos);
}

void MainToolbar::setArmEnabled(bool on) {
  if (m_armBtn) m_armBtn->setEnabled(on);
}

void MainToolbar::setArmState(bool armed) {
  if (m_armBtn) m_armBtn->setText(armed ? tr("DISARM") : tr("ARM"));
}

QString MainToolbar::currentPort() const {
  if (!m_portCombo) return {};
  QString port = m_portCombo->currentData().toString();
  if (port.isEmpty()) port = m_portCombo->currentText().trimmed();
  return port;
}

int MainToolbar::currentBaud() const {
  if (!m_baudCombo) return 0;
  return m_baudCombo->currentData().toInt();
}

void MainToolbar::setPort(const QString &path) {
  if (!m_portCombo || path.isEmpty()) return;
  const int idx = m_portCombo->findData(path);
  if (idx >= 0) m_portCombo->setCurrentIndex(idx);
  else          m_portCombo->setEditText(path);
}

void MainToolbar::setBaud(int baud) {
  if (!m_baudCombo || baud <= 0) return;
  const int idx = m_baudCombo->findData(baud);
  if (idx >= 0) m_baudCombo->setCurrentIndex(idx);
}
