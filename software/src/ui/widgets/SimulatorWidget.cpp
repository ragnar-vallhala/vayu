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
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr const char* kPwmFifoPath = "/tmp/vayu_pwm.fifo";
constexpr const char* kRepoRootSettingKey = "simulator/repoRoot";
constexpr const char* kCommandsGroupPrefix = "simulator/cmds/";

// Default commands, relative to the repo root.
constexpr const char* kDefaultGzCmd =
    "gz sim -s -r --headless-rendering tools/sim_gazebo/worlds/vayu_quad.sdf";
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
