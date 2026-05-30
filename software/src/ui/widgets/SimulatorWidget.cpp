#include "SimulatorWidget.h"

#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QSplitter>
#include <QUrl>
#include <QVBoxLayout>

namespace {

constexpr const char* kRepoRootSettingKey = "simulator/repoRoot";
constexpr const char* kLogDirSettingKey   = "simulator/logDir";
constexpr const char* kGeomGroup          = "simulator/geometry";

QString defaultRepoRoot() {
  QByteArray env = qgetenv("VAYU_REPO");
  if (!env.isEmpty()) return QString::fromUtf8(env);
  return QDir::homePath() + "/Documents/Drone/vayu";
}

QString defaultLogDir() {
  return defaultRepoRoot() + "/logs";
}

// C trampoline registered with vsim_iface_set_uart2_callback. Runs
// on whichever firmware thread is producing the bytes (telemetry
// task, flush task, ...). Marshals onto the widget's thread via
// queued invocation -- Qt does the heavy lifting; we don't share any
// Qt object across threads here, the QByteArray is copied across the
// connection.
void uart2_to_widget_trampoline(void* user, const uint8_t* data, size_t n) {
  auto* w = static_cast<SimulatorWidget*>(user);
  if (!w || n == 0) return;
  QByteArray ba(reinterpret_cast<const char*>(data), static_cast<int>(n));
  QMetaObject::invokeMethod(w, "onUartBytes", Qt::QueuedConnection,
                            Q_ARG(QByteArray, ba));
}

}  // namespace

// ============================================================================

SimulatorWidget::SimulatorWidget(QWidget* parent) : QWidget(parent) {
  QSettings settings;
  m_repoRoot =
      settings.value(kRepoRootSettingKey, defaultRepoRoot()).toString();
  m_logDir =
      settings.value(kLogDirSettingKey, defaultLogDir()).toString();

  // Initialize the shared iface up front. The pthread mutex/cond is
  // safe to leave constructed for the lifetime of Navigator; the
  // SimWorker + firmware threads both reference it.
  vsim_iface_init(&m_iface);
  m_ifaceInit = true;

  // Route firmware UART2 bytes through our queued slot. Once
  // vayu_sitl_start spins up the telemetry chain, packets begin
  // flowing into onUartBytes() on the GUI thread, which re-emits
  // them as dataReceived(). MainWindow connects that to its existing
  // DroneProtocol parser so the regular telemetry / log panels light
  // up automatically.
  vsim_iface_set_uart2_callback(&m_iface, &uart2_to_widget_trampoline, this);

  buildUi();
}

SimulatorWidget::~SimulatorWidget() {
  stopInAppSim();
  closeLogFile();    // belt-and-suspenders: stopInAppSim already does this
  // We do NOT call vayu_sitl_stop()'s teardown completely; the firmware
  // threads keep running until the process exits. See host_lifecycle.c.
  if (m_ifaceInit) {
    vsim_iface_destroy(&m_iface);
  }
}

// ----------------------------------------------------------------------------
// UI
// ----------------------------------------------------------------------------

void SimulatorWidget::buildUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(16, 16, 16, 16);
  root->setSpacing(12);

  // ---- Header ----
  {
    auto* header = new QHBoxLayout();
    auto* title = new QLabel(tr("Simulator (in-app)"), this);
    title->setStyleSheet(
        QString("color: %1; font-size: 18px; font-weight: bold;")
            .arg(Theme::hex(Theme::kAccent)));
    header->addWidget(title);
    header->addStretch();
    auto* backBtn = new ui::BackButton(this);
    backBtn->setToolTip(tr("Return to home"));
    connect(backBtn, &QPushButton::clicked, this,
            [this] { emit backToHomeRequested(); });
    header->addWidget(backBtn);
    root->addLayout(header);
  }

  // ---- Repo-root row + Launch / Stop ----
  {
    auto* row = new QHBoxLayout();
    row->addWidget(new QLabel(tr("Repo root:"), this));
    m_repoRootEdit = new QLineEdit(m_repoRoot, this);
    m_repoRootEdit->setToolTip(tr("Path to the vayu repo. Used for log paths."));
    connect(m_repoRootEdit, &QLineEdit::editingFinished, this, [this] {
      m_repoRoot = m_repoRootEdit->text();
      QSettings().setValue(kRepoRootSettingKey, m_repoRoot);
    });
    row->addWidget(m_repoRootEdit, 1);

    auto* launchBtn = new ui::SuccessButton(tr("Launch"), this);
    launchBtn->setToolTip(tr("Boot the in-process firmware + start the sim worker"));
    connect(launchBtn, &QPushButton::clicked, this,
            &SimulatorWidget::startInAppSim);
    row->addWidget(launchBtn);

    auto* stopBtn = new ui::DangerButton(tr("Stop"), this);
    stopBtn->setToolTip(tr("Pause the sim worker (firmware threads keep running)"));
    connect(stopBtn, &QPushButton::clicked, this,
            &SimulatorWidget::stopInAppSim);
    row->addWidget(stopBtn);
    root->addLayout(row);
  }

  // ---- Log dir row + current-file display ----
  {
    auto* row = new QHBoxLayout();
    row->addWidget(new QLabel(tr("Log dir:"), this));
    m_logDirEdit = new QLineEdit(m_logDir, this);
    m_logDirEdit->setToolTip(tr("Directory the per-run UART2 byte log is written into"));
    connect(m_logDirEdit, &QLineEdit::editingFinished, this, [this] {
      m_logDir = m_logDirEdit->text();
      QSettings().setValue(kLogDirSettingKey, m_logDir);
    });
    row->addWidget(m_logDirEdit, 1);

    auto* openBtn = new ui::GhostButton(tr("Open dir"), this);
    openBtn->setToolTip(tr("Open the log directory in your file manager"));
    connect(openBtn, &QPushButton::clicked, this, [this] {
      QDesktopServices::openUrl(QUrl::fromLocalFile(m_logDir));
    });
    row->addWidget(openBtn);
    root->addLayout(row);

    m_logPathLabel = new QLabel(tr("Current log: (none)"), this);
    m_logPathLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kTextMuted)));
    m_logPathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(m_logPathLabel);
  }

  // ---- Main split: renderer | right column ----
  auto* splitter = new QSplitter(Qt::Horizontal, this);
  root->addWidget(splitter, 1);

  m_renderer = new vsim::SimRendererWidget(splitter);
  splitter->addWidget(m_renderer);

  auto* right = new QWidget(splitter);
  splitter->addWidget(right);
  auto* rcol = new QVBoxLayout(right);
  rcol->setContentsMargins(0, 0, 0, 0);
  rcol->setSpacing(10);

  // -- Sim status group --
  {
    auto* g = new QGroupBox(tr("In-app sim"), right);
    auto* gv = new QVBoxLayout(g);
    m_simStatusLabel = new QLabel(tr("● Stopped"), g);
    m_simStatusLabel->setStyleSheet(
        QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
    gv->addWidget(m_simStatusLabel);

    auto* hb = new QHBoxLayout();
    m_simStartBtn = new ui::PrimaryButton(tr("Start"), g);
    m_simStartBtn->setToolTip(tr("Resume the sim worker (boots firmware on first call)"));
    connect(m_simStartBtn, &QPushButton::clicked, this,
            &SimulatorWidget::startInAppSim);
    hb->addWidget(m_simStartBtn);
    m_simStopBtn = new ui::GhostButton(tr("Stop"), g);
    m_simStopBtn->setEnabled(false);
    m_simStopBtn->setToolTip(tr("Stop the sim worker (firmware stays alive)"));
    connect(m_simStopBtn, &QPushButton::clicked, this,
            &SimulatorWidget::stopInAppSim);
    hb->addWidget(m_simStopBtn);
    gv->addLayout(hb);

    m_simPoseLabel = new QLabel(tr("pose: -"), g);
    m_simPoseLabel->setStyleSheet(
        QString("color: %1; font-family: monospace;")
            .arg(Theme::hex(Theme::kAccent)));
    gv->addWidget(m_simPoseLabel);
    rcol->addWidget(g);
  }

  // -- Geometry & motor editor --
  {
    m_geomEditor = new GeometryEditorWidget(right);
    connect(m_geomEditor, &GeometryEditorWidget::previewUpdated, this,
            [this] { applyGeometryToRenderer(); });
    connect(m_geomEditor, &GeometryEditorWidget::geometryApplied, this, [this] {
      applyGeometryToRenderer();
      if (m_sim) m_sim->sendGeometry(m_geomEditor->config());
      persistGeometry(m_geomEditor->config());
      appendLog("geom", tr("geometry applied (m=%1 kg)")
                            .arg(m_geomEditor->config().mass));
    });

    // The motor grid is wider than the right column; scroll rather than
    // squeeze it.
    auto* scroll = new QScrollArea(right);
    scroll->setWidgetResizable(true);
    scroll->setWidget(m_geomEditor);
    scroll->setMinimumHeight(240);
    rcol->addWidget(scroll, 1);
  }

  // -- Log --
  {
    auto* g = new QGroupBox(tr("Log"), right);
    auto* gv = new QVBoxLayout(g);
    m_log = new QPlainTextEdit(g);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    // Editor chrome handled by global QSS (QPlainTextEdit rule).
    gv->addWidget(m_log);
    rcol->addWidget(g, 1);
  }

  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);

  // Restore the persisted airframe; setConfig loads the mesh + recomputes
  // and fires previewUpdated -> applyGeometryToRenderer for the 3D view.
  m_geomEditor->setConfig(restoreGeometry());
}

// ----------------------------------------------------------------------------
// Sim lifecycle
// ----------------------------------------------------------------------------

void SimulatorWidget::startInAppSim() {
  if (m_sim) return;

  // Open a fresh per-run log file before the firmware starts emitting.
  openNewLogFile();

  // First Start: also boot the firmware in-process. host_lifecycle.c
  // spawns the vaios task pthreads (PID controllers, motor task,
  // telemetry, IMU feeder, RC feeder). Subsequent Starts only
  // resume the physics worker.
  if (!m_sitlStarted) {
    if (vayu_sitl_start(&m_iface) == 0) {
      m_sitlStarted = true;
      appendLog("sitl", "[vayu_sitl_start ok]");
    } else {
      appendLog("sitl", "[vayu_sitl_start failed - already running?]");
    }
  }

  // SimWorker spawns the vsim_d daemon and reads pose frames off
  // /tmp/vsim_pose; the iface is no longer used for PWM/IMU transport
  // (those went FIFO-only when we split the daemon out). The iface is
  // still alive for the UART2 telemetry callback above.
  m_sim = new vsim::SimWorker(this);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          m_renderer, &vsim::SimRendererWidget::setSnapshot);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          this, [this](vsim::SimSnapshot snap) {
            float pitch, yaw, roll;
            snap.att.getEulerAngles(&pitch, &yaw, &roll);
            m_simPoseLabel->setText(
                QString("pos=(%1, %2, %3) m   rpy=(%4, %5, %6) deg")
                    .arg(snap.pos_w.x(), 0, 'f', 2)
                    .arg(snap.pos_w.y(), 0, 'f', 2)
                    .arg(snap.pos_w.z(), 0, 'f', 2)
                    .arg(roll, 0, 'f', 1)
                    .arg(pitch, 0, 'f', 1)
                    .arg(yaw, 0, 'f', 1));
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString& s) { appendLog("vsim", s); });
  // Once the daemon's FIFOs are up, push the configured airframe so the
  // sim flies the edited mass properties + motor layout from frame one.
  connect(m_sim, &vsim::SimWorker::online, this, [this] {
    if (m_sim) m_sim->sendGeometry(m_geomEditor->config());
  });
  m_sim->start(QThread::TimeCriticalPriority);

  m_simStartBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  m_simStatusLabel->setText(tr("● Running"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
}

void SimulatorWidget::stopInAppSim() {
  if (!m_sim) return;
  m_sim->requestStop();
  if (!m_sim->wait(2000)) {
    m_sim->terminate();
    m_sim->wait(1000);
  }
  m_sim->deleteLater();
  m_sim = nullptr;
  m_simStartBtn->setEnabled(true);
  m_simStopBtn->setEnabled(false);
  m_simStatusLabel->setText(tr("● Stopped (firmware idle)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  // Close the per-run log file so its trailing bytes flush to disk.
  closeLogFile();
  // Note: we don't call vayu_sitl_stop() here on the Stop button.
  // The firmware threads stay alive but receive no fresh IMU samples
  // (SimWorker isn't pushing). Restarting the sim resumes the pipe.
}

// ----------------------------------------------------------------------------
// Geometry editor <-> renderer / daemon / persistence
// ----------------------------------------------------------------------------

void SimulatorWidget::applyGeometryToRenderer() {
  if (!m_renderer || !m_geomEditor) return;
  const auto& cfg = m_geomEditor->config();
  if (m_geomEditor->hasMesh()) {
    m_renderer->setDroneMesh(m_geomEditor->meshPositions(),
                             m_geomEditor->meshNormals());
  }
  m_renderer->setComMarker(cfg.com);
  std::array<QVector3D, 4> pos, axis;
  std::array<int, 4> spin;
  for (int i = 0; i < 4; ++i) {
    pos[i]  = cfg.motors[i].pos;
    axis[i] = cfg.motors[i].axis;
    spin[i] = cfg.motors[i].spin;
  }
  m_renderer->setMotorLayout(pos, axis, spin);
}

void SimulatorWidget::persistGeometry(const vsim::GeometryConfig& g) {
  QSettings s;
  s.beginGroup(kGeomGroup);
  s.setValue("meshPath", g.meshPath);
  s.setValue("scale", g.scale);
  s.setValue("mass", g.mass);
  s.setValue("comX", g.com.x());
  s.setValue("comY", g.com.y());
  s.setValue("comZ", g.com.z());
  for (int i = 0; i < 9; ++i) s.setValue(QString("I%1").arg(i), g.inertia[i]);
  for (int i = 0; i < 4; ++i) {
    const auto& m = g.motors[i];
    const QString p = QString("m%1_").arg(i);
    s.setValue(p + "px", m.pos.x());
    s.setValue(p + "py", m.pos.y());
    s.setValue(p + "pz", m.pos.z());
    s.setValue(p + "ax", m.axis.x());
    s.setValue(p + "ay", m.axis.y());
    s.setValue(p + "az", m.axis.z());
    s.setValue(p + "spin", m.spin);
    s.setValue(p + "kt", m.k_thrust);
    s.setValue(p + "km", m.k_moment);
    s.setValue(p + "wmax", m.max_omega);
  }
  s.endGroup();
}

vsim::GeometryConfig SimulatorWidget::restoreGeometry() {
  vsim::GeometryConfig g;  // sensible defaults if nothing was saved
  QSettings s;
  s.beginGroup(kGeomGroup);
  if (!s.contains("mass")) {
    s.endGroup();
    return g;
  }
  g.meshPath = s.value("meshPath").toString();
  g.scale = s.value("scale", g.scale).toFloat();
  g.mass = s.value("mass", g.mass).toFloat();
  g.com = QVector3D(s.value("comX").toFloat(), s.value("comY").toFloat(),
                    s.value("comZ").toFloat());
  for (int i = 0; i < 9; ++i)
    g.inertia[i] = s.value(QString("I%1").arg(i), g.inertia[i]).toFloat();
  for (int i = 0; i < 4; ++i) {
    auto& m = g.motors[i];
    const QString p = QString("m%1_").arg(i);
    m.pos = QVector3D(s.value(p + "px", m.pos.x()).toFloat(),
                      s.value(p + "py", m.pos.y()).toFloat(),
                      s.value(p + "pz", m.pos.z()).toFloat());
    m.axis = QVector3D(s.value(p + "ax", m.axis.x()).toFloat(),
                       s.value(p + "ay", m.axis.y()).toFloat(),
                       s.value(p + "az", m.axis.z()).toFloat());
    m.spin = s.value(p + "spin", m.spin).toInt();
    m.k_thrust = s.value(p + "kt", m.k_thrust).toFloat();
    m.k_moment = s.value(p + "km", m.k_moment).toFloat();
    m.max_omega = s.value(p + "wmax", m.max_omega).toFloat();
  }
  s.endGroup();
  return g;
}

void SimulatorWidget::onUartBytes(QByteArray bytes) {
  // 1) Persist to the per-run raw byte log so post-run analysis has
  //    the exact same byte stream the GCS saw.
  if (m_runLog && m_runLog->isOpen()) {
    m_runLog->write(bytes);
    m_runLogBytes += bytes.size();
  }
  // 2) Forward to anything connected to dataReceived (MainWindow's
  //    DroneProtocol parser typically).
  emit dataReceived(bytes);
  // 3) Periodic one-line size readout in the panel log.
  static int chatter_div = 0;
  if (++chatter_div % 200 == 0 && m_runLog) {
    appendLog("uart2", QString("logged %1 KB").arg(m_runLogBytes / 1024));
  }
}

void SimulatorWidget::openNewLogFile() {
  closeLogFile();
  QDir dir(m_logDir);
  if (!dir.exists() && !dir.mkpath(".")) {
    appendLog("log", QString("[mkdir failed: %1]").arg(m_logDir));
    return;
  }
  QString stamp =
      QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
  QString path = dir.absoluteFilePath(QString("sim-%1.bin").arg(stamp));
  m_runLog = std::make_unique<QFile>(path);
  if (!m_runLog->open(QIODevice::WriteOnly)) {
    appendLog("log", QString("[open failed: %1]").arg(path));
    m_runLog.reset();
    return;
  }
  m_runLogBytes = 0;
  if (m_logPathLabel) {
    m_logPathLabel->setText(QString("Current log: %1").arg(path));
    m_logPathLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kOk)));
  }
  appendLog("log", QString("[opened %1]").arg(path));
}

void SimulatorWidget::closeLogFile() {
  if (!m_runLog) return;
  if (m_runLog->isOpen()) {
    m_runLog->flush();
    appendLog("log", QString("[closed: %1 bytes total]").arg(m_runLogBytes));
    m_runLog->close();
  }
  m_runLog.reset();
}

void SimulatorWidget::appendLog(const QString& tag, const QString& text) {
  QString trimmed = text;
  while (trimmed.endsWith('\n') || trimmed.endsWith('\r')) {
    trimmed.chop(1);
  }
  if (trimmed.isEmpty()) return;
  for (const QString& line : trimmed.split('\n', Qt::SkipEmptyParts)) {
    m_log->appendPlainText(QString("[%1] %2").arg(tag, line));
  }
  m_log->verticalScrollBar()->setValue(
      m_log->verticalScrollBar()->maximum());
}
