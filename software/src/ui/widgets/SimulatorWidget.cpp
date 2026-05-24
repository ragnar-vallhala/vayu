#include "SimulatorWidget.h"

#include <QApplication>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QProcessEnvironment>
#include <QScrollBar>
#include <QSettings>
#include <QSplitter>
#include <QTextCursor>
#include <QVBoxLayout>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>      // posix_openpt, grantpt, unlockpt, ptsname
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr const char* kPwmFifoPath = "/tmp/vayu_pwm.fifo";
constexpr const char* kRepoRootSettingKey = "simulator/repoRoot";
constexpr const char* kPassthroughSettingKey = "simulator/passthrough";
constexpr const char* kVirtualRcSettingKey = "simulator/virtualRc";
constexpr const char* kCommandsGroupPrefix = "simulator/cmds/";

// Default commands, relative to the repo root.
// Gazebo GUI is forced onto the AMD Vega 6 via DRI_PRIME=0 and Mesa, so it
// never touches the NVIDIA driver. The hybrid Optimus setup on this host has
// crashed Gazebo's GUI before when it tried to use the NVIDIA path; rendering
// on the integrated GPU is slower but stable. If you want to go back to
// headless, edit the field to: gz sim -s -r --headless-rendering ...
constexpr const char* kDefaultGzCmd =
    "env DRI_PRIME=0 __GLX_VENDOR_LIBRARY_NAME=mesa "
    "gz sim -r tools/sim_gazebo/worlds/vayu_quad.sdf";
constexpr const char* kDefaultImuBridgeCmd =
    "python3 tools/sim_gazebo/gz_imu_to_vayu.py";
constexpr const char* kDefaultPwmBridgeCmd =
    "python3 tools/sim_gazebo/vayu_pwm_to_gz.py";
constexpr const char* kDefaultSitlCmd = "./build_sitl/vayu_sitl";

QString defaultRepoRoot() {
  // Honor VAYU_REPO if set; otherwise default to ~/Documents/Drone/vayu
  // which matches the user's actual layout. The path is editable + saved
  // via QSettings, so this default only matters on first launch.
  QByteArray env = qgetenv("VAYU_REPO");
  if (!env.isEmpty()) return QString::fromUtf8(env);
  return QDir::homePath() + "/Documents/Drone/vayu";
}

}  // namespace

// ============================================================================

SimulatorWidget::SimulatorWidget(QWidget* parent) : QWidget(parent) {
  QSettings settings;
  m_repoRoot =
      settings.value(kRepoRootSettingKey, defaultRepoRoot()).toString();

  m_gz.name = "Gazebo (headless)";
  m_gz.tag = "gz";
  m_gz.defaultCmd = kDefaultGzCmd;

  m_imuBridge.name = "IMU bridge (gz → vayu)";
  m_imuBridge.tag = "imu";
  m_imuBridge.defaultCmd = kDefaultImuBridgeCmd;

  m_pwmBridge.name = "PWM bridge (vayu → gz)";
  m_pwmBridge.tag = "pwm";
  m_pwmBridge.defaultCmd = kDefaultPwmBridgeCmd;

  m_sitl.name = "vayu_sitl";
  m_sitl.tag = "sitl";
  m_sitl.defaultCmd = kDefaultSitlCmd;

  buildUi();
  openPwmFifo();
}

SimulatorWidget::~SimulatorWidget() {
  for (Proc* p : {&m_gz, &m_imuBridge, &m_pwmBridge, &m_sitl}) {
    if (p->process &&
        p->process->state() != QProcess::NotRunning) {
      p->process->terminate();
      if (!p->process->waitForFinished(1500)) p->process->kill();
    }
  }
  closePwmFifo();
  closeVirtualRcPty();
}

// ----------------------------------------------------------------------------
// UI construction
// ----------------------------------------------------------------------------

void SimulatorWidget::buildUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(16, 16, 16, 16);
  root->setSpacing(12);

  // ---- Header: title + back button ----
  auto* headerRow = new QHBoxLayout();
  auto* title = new QLabel("<h2>Simulator</h2>", this);
  title->setStyleSheet("color: #61AFEF;");
  headerRow->addWidget(title);
  headerRow->addStretch();

  auto* backBtn = new QPushButton("← Home", this);
  backBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; padding: 4px 14px; border-radius: 4px; font-weight: bold; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(backBtn, &QPushButton::clicked, this,
          &SimulatorWidget::backToHomeRequested);
  headerRow->addWidget(backBtn);
  root->addLayout(headerRow);

  // ---- Repo root row ----
  auto* repoRow = new QHBoxLayout();
  repoRow->addWidget(new QLabel("Repo root:", this));
  m_repoRootEdit = new QLineEdit(m_repoRoot, this);
  m_repoRootEdit->setStyleSheet(
      "background: #21252B; color: #ABB2BF; border: 1px solid #3E4452; "
      "padding: 4px; font-family: monospace;");
  connect(m_repoRootEdit, &QLineEdit::editingFinished, this, [this]() {
    m_repoRoot = m_repoRootEdit->text();
    QSettings().setValue(kRepoRootSettingKey, m_repoRoot);
  });
  repoRow->addWidget(m_repoRootEdit, 1);
  root->addLayout(repoRow);

  // ---- Passthrough toggle ----
  // Prepends `env VAYU_SITL_PASSTHROUGH=1` to the vayu_sitl command at
  // launch, which makes the SITL binary skip the cascaded PID and write
  // throttle directly to all four motors. Useful when the rate-PID
  // integrator winds up against the drone's ground-friction-locked
  // attitude and starves one motor (a classic "PID windup on the ground"
  // pattern that prevents liftoff).
  auto* ptRow = new QHBoxLayout();
  m_passthroughCheck = new QCheckBox(
      "Passthrough mode (bypass cascaded PID — throttle → all 4 motors)",
      this);
  m_passthroughCheck->setChecked(
      QSettings().value(kPassthroughSettingKey, false).toBool());
  m_passthroughCheck->setStyleSheet("color: #ABB2BF; padding: 2px;");
  connect(m_passthroughCheck, &QCheckBox::toggled, this, [](bool on) {
    QSettings().setValue(kPassthroughSettingKey, on);
  });
  ptRow->addWidget(m_passthroughCheck);
  ptRow->addStretch();
  root->addLayout(ptRow);

  // ---- Virtual RC group ----
  // Creates a pty pair on toggle; the GCS writes CSV channel widths
  // to the master fd at 50 Hz from the slider state, and exports the
  // slave path (/dev/pts/N) as VAYU_UART_RC_PATH on the SITL command
  // at launch. Same firmware path the real sim_bridge MCU drives.
  auto* rcGroup = new QGroupBox("Virtual RC (pty-backed Arduino stand-in)", this);
  auto* rcV = new QVBoxLayout(rcGroup);

  auto* rcEnableRow = new QHBoxLayout();
  m_virtualRcCheck = new QCheckBox(
      "Enable virtual RC (sticks below drive vayu_sitl over a pty)", this);
  m_virtualRcCheck->setStyleSheet("color: #ABB2BF; padding: 2px;");
  m_virtualRcCheck->setChecked(
      QSettings().value(kVirtualRcSettingKey, false).toBool());
  connect(m_virtualRcCheck, &QCheckBox::toggled, this,
          &SimulatorWidget::onVirtualRcToggled);
  rcEnableRow->addWidget(m_virtualRcCheck);
  m_virtualRcPathLabel = new QLabel("(pty not open)", this);
  m_virtualRcPathLabel->setStyleSheet(
      "color: #61AFEF; font-family: monospace; font-size: 11px;");
  rcEnableRow->addWidget(m_virtualRcPathLabel);
  rcEnableRow->addStretch();
  rcV->addLayout(rcEnableRow);

  auto* rcGrid = new QGridLayout();
  rcGrid->setHorizontalSpacing(8);

  struct SliderSpec {
    const char* label;
    QSlider** target;
    QLabel** valueLabel;
    int defaultVal;
  };
  SliderSpec specs[] = {
      {"Roll",     &m_rcRollSlider,     &m_rcRollLabel,     1500},
      {"Pitch",    &m_rcPitchSlider,    &m_rcPitchLabel,    1500},
      {"Throttle", &m_rcThrottleSlider, &m_rcThrottleLabel, 1000},
      {"Yaw",      &m_rcYawSlider,      &m_rcYawLabel,      1500},
  };
  for (int i = 0; i < 4; ++i) {
    auto* nameLbl = new QLabel(specs[i].label, this);
    nameLbl->setMinimumWidth(60);
    nameLbl->setStyleSheet("color: #ABB2BF;");
    rcGrid->addWidget(nameLbl, i, 0);

    auto* s = new QSlider(Qt::Horizontal, this);
    s->setRange(1000, 2000);
    s->setValue(specs[i].defaultVal);
    s->setStyleSheet(
        "QSlider::groove:horizontal { background: #21252B; height: 6px; "
        "border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #61AFEF; width: 14px; "
        "margin: -6px 0; border-radius: 3px; }");
    rcGrid->addWidget(s, i, 1);
    *specs[i].target = s;

    auto* val = new QLabel(QString::number(specs[i].defaultVal), this);
    val->setMinimumWidth(50);
    val->setStyleSheet("color: #61AFEF; font-family: monospace;");
    rcGrid->addWidget(val, i, 2);
    *specs[i].valueLabel = val;

    connect(s, &QSlider::valueChanged, val,
            [val](int v) { val->setText(QString::number(v)); });
  }
  rcV->addLayout(rcGrid);

  auto* rcBtnRow = new QHBoxLayout();
  m_rcArmSwitch = new QCheckBox("SwA armed (ch5 = 2000)", this);
  m_rcArmSwitch->setStyleSheet("color: #ABB2BF;");
  rcBtnRow->addWidget(m_rcArmSwitch);
  m_rcRecenterBtn = new QPushButton("Re-center sticks", this);
  m_rcRecenterBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; padding: 4px 12px; border-radius: 3px; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(m_rcRecenterBtn, &QPushButton::clicked, this, [this]() {
    m_rcRollSlider->setValue(1500);
    m_rcPitchSlider->setValue(1500);
    m_rcThrottleSlider->setValue(1000);
    m_rcYawSlider->setValue(1500);
  });
  rcBtnRow->addWidget(m_rcRecenterBtn);
  rcBtnRow->addStretch();
  rcV->addLayout(rcBtnRow);

  root->addWidget(rcGroup);

  // 50 Hz timer for CSV frame emission - matches the real sim_bridge cadence.
  m_rcTimer = new QTimer(this);
  m_rcTimer->setInterval(20);
  connect(m_rcTimer, &QTimer::timeout, this, &SimulatorWidget::onRcTimerTick);

  // Apply persisted enabled state without retriggering writes to QSettings.
  if (m_virtualRcCheck->isChecked()) onVirtualRcToggled(true);

  // ---- Processes group ----
  auto* procGroup = new QGroupBox("Processes", this);
  auto* procV = new QVBoxLayout(procGroup);

  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(6);
  // Header row
  grid->addWidget(new QLabel("<b>Process</b>", this), 0, 0);
  grid->addWidget(new QLabel("<b>Status</b>", this), 0, 1);
  grid->addWidget(new QLabel("<b>Command (cwd = repo root)</b>", this), 0, 2);
  grid->addWidget(new QLabel("", this), 0, 3);
  grid->addWidget(new QLabel("", this), 0, 4);
  grid->setColumnStretch(2, 1);

  buildProcessRow(grid, 1, &m_gz);
  buildProcessRow(grid, 2, &m_imuBridge);
  buildProcessRow(grid, 3, &m_pwmBridge);
  buildProcessRow(grid, 4, &m_sitl);
  procV->addLayout(grid);

  auto* allRow = new QHBoxLayout();
  auto* launchAllBtn = new QPushButton("⏵ Launch All", this);
  launchAllBtn->setStyleSheet(
      "QPushButton { background: #2BBC8A; color: #21252B; border: none; "
      "padding: 6px 18px; border-radius: 4px; font-weight: bold; }"
      "QPushButton:hover { background: #3FD49F; }");
  connect(launchAllBtn, &QPushButton::clicked, this,
          &SimulatorWidget::onLaunchAll);

  auto* stopAllBtn = new QPushButton("⏹ Stop All", this);
  stopAllBtn->setStyleSheet(
      "QPushButton { background: #E06C75; color: #21252B; border: none; "
      "padding: 6px 18px; border-radius: 4px; font-weight: bold; }"
      "QPushButton:hover { background: #EF8088; }");
  connect(stopAllBtn, &QPushButton::clicked, this,
          &SimulatorWidget::onStopAll);

  allRow->addWidget(launchAllBtn);
  allRow->addWidget(stopAllBtn);
  allRow->addStretch();
  procV->addLayout(allRow);

  root->addWidget(procGroup);

  // ---- PWM monitor + log split ----
  auto* splitter = new QSplitter(Qt::Horizontal, this);

  // PWM monitor group
  auto* pwmGroup = new QGroupBox("PWM monitor (/tmp/vayu_pwm.fifo)", splitter);
  auto* pwmV = new QVBoxLayout(pwmGroup);
  m_pwmStatusLabel = new QLabel("(waiting for SITL output…)", pwmGroup);
  m_pwmStatusLabel->setStyleSheet("color: #ABB2BF; font-style: italic;");
  pwmV->addWidget(m_pwmStatusLabel);

  for (int i = 0; i < 4; ++i) {
    auto* row = new QHBoxLayout();
    auto* nameLabel = new QLabel(QString("M%1").arg(i), pwmGroup);
    nameLabel->setMinimumWidth(28);
    nameLabel->setStyleSheet("color: #ABB2BF; font-weight: bold;");
    row->addWidget(nameLabel);

    m_motorBars[i] = new QProgressBar(pwmGroup);
    m_motorBars[i]->setRange(0, 1000);
    m_motorBars[i]->setValue(0);
    m_motorBars[i]->setTextVisible(false);
    m_motorBars[i]->setStyleSheet(
        "QProgressBar { background: #21252B; border: 1px solid #3E4452; "
        "border-radius: 3px; height: 14px; }"
        "QProgressBar::chunk { background: #61AFEF; border-radius: 2px; }");
    row->addWidget(m_motorBars[i], 1);

    m_motorLabels[i] = new QLabel("—", pwmGroup);
    m_motorLabels[i]->setMinimumWidth(70);
    m_motorLabels[i]->setStyleSheet(
        "color: #61AFEF; font-family: monospace; font-weight: bold;");
    row->addWidget(m_motorLabels[i]);

    pwmV->addLayout(row);
  }
  pwmV->addStretch();
  splitter->addWidget(pwmGroup);

  // Log group
  auto* logGroup = new QGroupBox("Process log", splitter);
  auto* logV = new QVBoxLayout(logGroup);
  m_log = new QPlainTextEdit(logGroup);
  m_log->setReadOnly(true);
  m_log->setMaximumBlockCount(2000);
  m_log->setStyleSheet(
      "background: #1A1D27; color: #ABB2BF; border: 1px solid #2A3347; "
      "font-family: monospace; font-size: 11px;");
  logV->addWidget(m_log);
  splitter->addWidget(logGroup);
  splitter->setSizes({240, 600});

  root->addWidget(splitter, 1);
}

void SimulatorWidget::buildProcessRow(QGridLayout* grid, int row, Proc* p) {
  auto* nameLbl = new QLabel(p->name, this);
  nameLbl->setStyleSheet("color: #ABB2BF;");
  grid->addWidget(nameLbl, row, 0);

  p->statusLabel = new QLabel("● Stopped", this);
  p->statusLabel->setStyleSheet("color: #E06C75; font-weight: bold;");
  grid->addWidget(p->statusLabel, row, 1);

  // Restore command from settings (per-tag), else default.
  QSettings settings;
  QString savedCmd =
      settings.value(QString(kCommandsGroupPrefix) + p->tag, p->defaultCmd)
          .toString();
  p->commandEdit = new QLineEdit(savedCmd, this);
  p->commandEdit->setStyleSheet(
      "background: #21252B; color: #ABB2BF; border: 1px solid #3E4452; "
      "padding: 4px; font-family: monospace; font-size: 11px;");
  connect(p->commandEdit, &QLineEdit::editingFinished, this, [this, p]() {
    QSettings().setValue(QString(kCommandsGroupPrefix) + p->tag,
                         p->commandEdit->text());
  });
  grid->addWidget(p->commandEdit, row, 2);

  p->startBtn = new QPushButton("Start", this);
  p->startBtn->setStyleSheet(
      "QPushButton { background: #2C313A; color: #ABB2BF; border: 1px solid "
      "#3E4452; padding: 4px 12px; border-radius: 3px; }"
      "QPushButton:hover { background: #3E4452; }");
  connect(p->startBtn, &QPushButton::clicked, this,
          [this, p]() { startProcess(p); });
  grid->addWidget(p->startBtn, row, 3);

  p->stopBtn = new QPushButton("Stop", this);
  p->stopBtn->setStyleSheet(p->startBtn->styleSheet());
  p->stopBtn->setEnabled(false);
  connect(p->stopBtn, &QPushButton::clicked, this,
          [this, p]() { stopProcess(p); });
  grid->addWidget(p->stopBtn, row, 4);
}

// ----------------------------------------------------------------------------
// Process management
// ----------------------------------------------------------------------------

void SimulatorWidget::connectProcessSignals(Proc* p) {
  connect(p->process, &QProcess::readyReadStandardOutput, this, [this, p]() {
    QString text = QString::fromUtf8(p->process->readAllStandardOutput());
    appendLog(p->tag, text);
  });
  connect(p->process, &QProcess::readyReadStandardError, this, [this, p]() {
    QString text = QString::fromUtf8(p->process->readAllStandardError());
    appendLog(p->tag, text);
  });
  connect(p->process, &QProcess::stateChanged, this,
          [this, p](QProcess::ProcessState) { updateStatusLabel(p); });
  connect(p->process, &QProcess::errorOccurred, this,
          [this, p](QProcess::ProcessError err) {
            appendLog(p->tag, QString("[error %1] %2\n").arg(err).arg(
                                    p->process->errorString()));
          });
}

void SimulatorWidget::startProcess(Proc* p) {
  if (p->process && p->process->state() != QProcess::NotRunning) {
    appendLog(p->tag, "(already running)\n");
    return;
  }

  if (!p->process) {
    p->process = new QProcess(this);
    p->process->setProcessChannelMode(QProcess::SeparateChannels);
    connectProcessSignals(p);
  }

  QString cmd = p->commandEdit->text().trimmed();
  if (cmd.isEmpty()) {
    appendLog(p->tag, "(empty command, skipped)\n");
    return;
  }

  // Passthrough toggle: only affects the SITL row. We prepend
  // `env VAYU_SITL_PASSTHROUGH=1` rather than touching the command field
  // itself so the user's edited command stays clean.
  if (p == &m_sitl && m_passthroughCheck && m_passthroughCheck->isChecked()) {
    if (!cmd.contains("VAYU_SITL_PASSTHROUGH")) {
      cmd = "env VAYU_SITL_PASSTHROUGH=1 " + cmd;
    }
  }

  // Virtual RC: when the pty is open, route the SITL's UART RC reader
  // at the slave end. The firmware sees a normal serial source streaming
  // sim_bridge-style CSV at 50 Hz.
  if (p == &m_sitl && m_virtualRcCheck && m_virtualRcCheck->isChecked() &&
      !m_ptySlavePath.isEmpty()) {
    if (!cmd.contains("VAYU_UART_RC_PATH")) {
      cmd = QString("env VAYU_UART_RC_PATH=%1 %2").arg(m_ptySlavePath, cmd);
    }
  }

  p->process->setWorkingDirectory(workingDir());

  // QProcess::startCommand handles shell-style splitting reasonably.
  appendLog(p->tag, QString("$ %1\n").arg(cmd));
  p->process->startCommand(cmd);
}

void SimulatorWidget::stopProcess(Proc* p) {
  if (!p->process || p->process->state() == QProcess::NotRunning) {
    return;
  }
  appendLog(p->tag, "(SIGTERM)\n");
  p->process->terminate();
  // Don't block the UI; let the process land. If it doesn't, the destructor
  // will kill it on widget shutdown.
}

void SimulatorWidget::updateStatusLabel(Proc* p) {
  if (!p->process) return;
  switch (p->process->state()) {
    case QProcess::NotRunning:
      p->statusLabel->setText("● Stopped");
      p->statusLabel->setStyleSheet("color: #E06C75; font-weight: bold;");
      p->startBtn->setEnabled(true);
      p->stopBtn->setEnabled(false);
      break;
    case QProcess::Starting:
      p->statusLabel->setText("● Starting");
      p->statusLabel->setStyleSheet("color: #E5C07B; font-weight: bold;");
      p->startBtn->setEnabled(false);
      p->stopBtn->setEnabled(true);
      break;
    case QProcess::Running:
      p->statusLabel->setText(
          QString("● Running (pid %1)").arg(p->process->processId()));
      p->statusLabel->setStyleSheet("color: #2BBC8A; font-weight: bold;");
      p->startBtn->setEnabled(false);
      p->stopBtn->setEnabled(true);
      break;
  }
}

// "Launch All" walks the dependency order: gazebo, then the bridges, then SITL.
// We don't wait synchronously — the bridges/SITL retry their FIFO opens so a
// staggered come-up resolves itself within a couple of seconds.
void SimulatorWidget::onLaunchAll() {
  startProcess(&m_gz);
  startProcess(&m_imuBridge);
  startProcess(&m_pwmBridge);
  startProcess(&m_sitl);
}

void SimulatorWidget::onStopAll() {
  // Reverse order: stop SITL first so it doesn't keep producing PWM into the
  // FIFO with no consumer.
  stopProcess(&m_sitl);
  stopProcess(&m_pwmBridge);
  stopProcess(&m_imuBridge);
  stopProcess(&m_gz);
}

// ----------------------------------------------------------------------------
// Log
// ----------------------------------------------------------------------------

void SimulatorWidget::appendLog(const QString& tag, const QString& text) {
  if (!m_log) return;
  // Split on newlines so we can prefix each line with the tag.
  QString prefixed;
  prefixed.reserve(text.size() + tag.size() + 8);
  for (const QString& rawLine : text.split('\n')) {
    if (rawLine.isEmpty()) continue;
    prefixed.append(QString("[%1] %2\n").arg(tag, rawLine));
  }
  if (prefixed.isEmpty()) return;
  // Append without inserting an extra newline; QPlainTextEdit's appendPlainText
  // adds one of its own, which would double-space our log.
  QTextCursor cur(m_log->document());
  cur.movePosition(QTextCursor::End);
  cur.insertText(prefixed);
  m_log->verticalScrollBar()->setValue(m_log->verticalScrollBar()->maximum());
}

// ----------------------------------------------------------------------------
// PWM FIFO monitor
// ----------------------------------------------------------------------------

void SimulatorWidget::openPwmFifo() {
  // mkfifo if missing, then open non-blocking O_RDWR. O_RDWR is the
  // same trick the SITL uses on its producer side: the kernel doesn't
  // block waiting for the "other end" because we are the other end.
  struct stat st;
  if (::stat(kPwmFifoPath, &st) != 0) {
    if (::mkfifo(kPwmFifoPath, 0666) != 0 && errno != EEXIST) {
      m_pwmStatusLabel->setText(
          QString("FIFO error: mkfifo %1 failed (%2)").arg(kPwmFifoPath,
              QString::fromUtf8(strerror(errno))));
      return;
    }
  }
  m_pwmFd = ::open(kPwmFifoPath, O_RDWR | O_NONBLOCK);
  if (m_pwmFd < 0) {
    m_pwmStatusLabel->setText(
        QString("FIFO error: open %1 failed (%2)").arg(kPwmFifoPath,
            QString::fromUtf8(strerror(errno))));
    return;
  }
  m_pwmNotifier = new QSocketNotifier(m_pwmFd, QSocketNotifier::Read, this);
  connect(m_pwmNotifier, &QSocketNotifier::activated, this,
          &SimulatorWidget::onPwmReadable);
  m_pwmStatusLabel->setText(
      QString("FIFO: %1 (open, waiting for data)").arg(kPwmFifoPath));
}

void SimulatorWidget::closePwmFifo() {
  if (m_pwmNotifier) {
    m_pwmNotifier->setEnabled(false);
    m_pwmNotifier->deleteLater();
    m_pwmNotifier = nullptr;
  }
  if (m_pwmFd >= 0) {
    ::close(m_pwmFd);
    m_pwmFd = -1;
  }
}

void SimulatorWidget::onPwmReadable() {
  if (m_pwmFd < 0) return;
  char chunk[1024];
  ssize_t r = ::read(m_pwmFd, chunk, sizeof(chunk));
  if (r > 0) {
    m_pwmBuf.append(chunk, static_cast<int>(r));
    parsePwmBuffer();
  } else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
    // Producer closed (rare with O_RDWR, but handle anyway) or real error.
    // Don't tear down — just note it; next writer reconnect will resume.
  }
}

void SimulatorWidget::parsePwmBuffer() {
  // Each frame is "<idx> <duty>\n", e.g. "0 0.520000\n". Pull whole lines,
  // leave a partial trailing line in the buffer for next time.
  int newlineIdx;
  while ((newlineIdx = m_pwmBuf.indexOf('\n')) >= 0) {
    QByteArray line = m_pwmBuf.left(newlineIdx);
    m_pwmBuf.remove(0, newlineIdx + 1);
    if (line.isEmpty()) continue;

    bool okIdx = false, okDuty = false;
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx <= 0) continue;
    int idx = line.left(spaceIdx).toInt(&okIdx);
    double duty = line.mid(spaceIdx + 1).toDouble(&okDuty);
    if (!okIdx || !okDuty) continue;
    if (idx < 0 || idx >= 4) continue;

    if (duty < 0.0) duty = 0.0;
    if (duty > 1.0) duty = 1.0;
    m_motorBars[idx]->setValue(static_cast<int>(duty * 1000.0));
    m_motorLabels[idx]->setText(QString::number(duty, 'f', 3));
  }
  m_pwmStatusLabel->setText(
      QString("FIFO: %1 (streaming)").arg(kPwmFifoPath));
}

QString SimulatorWidget::workingDir() const {
  QString root = m_repoRoot.trimmed();
  if (root.isEmpty()) root = defaultRepoRoot();
  // QProcess does the right thing if root doesn't exist (start will fail
  // and surface via errorOccurred). We don't fight it here.
  return root;
}

// ----------------------------------------------------------------------------
// Virtual RC: pty pair + 50 Hz CSV emitter
// ----------------------------------------------------------------------------

bool SimulatorWidget::openVirtualRcPty() {
  if (m_ptyMasterFd >= 0) return true;

  // POSIX pty allocation. The master fd is what we write to; the slave's
  // path (/dev/pts/N) is what we hand to vayu_sitl as VAYU_UART_RC_PATH.
  int fd = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (fd < 0 || ::grantpt(fd) < 0 || ::unlockpt(fd) < 0) {
    appendLog("rc", QString("pty open failed: %1\n")
                          .arg(QString::fromUtf8(strerror(errno))));
    if (fd >= 0) ::close(fd);
    return false;
  }
  const char* slave = ::ptsname(fd);
  if (!slave) {
    appendLog("rc", "ptsname failed\n");
    ::close(fd);
    return false;
  }
  m_ptyMasterFd = fd;
  m_ptySlavePath = QString::fromUtf8(slave);
  appendLog("rc",
            QString("virtual RC pty open: slave=%1\n").arg(m_ptySlavePath));
  if (m_virtualRcPathLabel)
    m_virtualRcPathLabel->setText(QString("slave: %1").arg(m_ptySlavePath));
  return true;
}

void SimulatorWidget::closeVirtualRcPty() {
  if (m_rcTimer && m_rcTimer->isActive()) m_rcTimer->stop();
  if (m_ptyMasterFd >= 0) {
    ::close(m_ptyMasterFd);
    m_ptyMasterFd = -1;
  }
  m_ptySlavePath.clear();
  if (m_virtualRcPathLabel) m_virtualRcPathLabel->setText("(pty not open)");
}

void SimulatorWidget::onVirtualRcToggled(bool on) {
  QSettings().setValue(kVirtualRcSettingKey, on);
  if (on) {
    if (openVirtualRcPty() && m_rcTimer) m_rcTimer->start();
  } else {
    closeVirtualRcPty();
  }
}

void SimulatorWidget::onRcTimerTick() {
  if (m_ptyMasterFd < 0) return;
  int ch1 = m_rcRollSlider     ? m_rcRollSlider->value()     : 1500;
  int ch2 = m_rcPitchSlider    ? m_rcPitchSlider->value()    : 1500;
  int ch3 = m_rcThrottleSlider ? m_rcThrottleSlider->value() : 1000;
  int ch4 = m_rcYawSlider      ? m_rcYawSlider->value()      : 1500;
  int ch5 = (m_rcArmSwitch && m_rcArmSwitch->isChecked()) ? 2000 : 1000;
  int ch6 = 1000;
  QByteArray line = QString("%1,%2,%3,%4,%5,%6\n")
                        .arg(ch1).arg(ch2).arg(ch3).arg(ch4)
                        .arg(ch5).arg(ch6)
                        .toUtf8();
  ssize_t n = ::write(m_ptyMasterFd, line.constData(), line.size());
  (void)n;  // best-effort - pty consumer may not be connected yet
}
