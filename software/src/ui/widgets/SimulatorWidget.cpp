#include "SimulatorWidget.h"

#include "../../vsim/MeshLoader.h"
#include "../../vsim/WorldMeshBuilder.h"
#include "CollapsibleSection.h"
#include "comm/PortArbiter.h"
#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDesktopServices>
#include <QCoreApplication>
#include <QDir>
#include <QTimer>
#include <QEvent>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QSlider>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QScrollBar>
#include <QSerialPortInfo>
#include <QLineEdit>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QUrl>
#include <QByteArray>
#include <QFile>
#include <QVBoxLayout>
#include <QProcess>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QLabel>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

#include <cmath>
#include <cstdio>   // ::rename (atomic blob publish)

#include <unistd.h>

// In-process firmware (libvayu_sitl_core) mixer setter — keeps the firmware
// roll/pitch/yaw->motor mix consistent with the vehicle geometry the sim uses.
extern "C" void angle_rate_controller_set_motor_geometry(
    const float pos_x[4], const float pos_y[4], const int spin[4]);

// In-process firmware flight-mode control (mirrors the geometry forward-decl
// above; the firmware include path isn't on the GCS). mode arg: 0=stabilise/
// angle, 1=acro — matching flight_mode_t. Telemetry uses the same values, with
// source 1 == GCS override.
extern "C" void flight_mode_set_override(int mode);
extern "C" void flight_mode_release(void);

namespace {
constexpr int kFmAngle = 0, kFmAcro = 1, kFmSrcGcs = 1;

// Push the vehicle's motor layout into the in-process firmware mixer so the
// control mix matches the physics (stable for any quad layout, not just the
// firmware's default numbering).
void pushFirmwareMotorGeometry(const vsim::GeometryConfig& g) {
  float x[4], y[4];
  int spin[4];
  for (int i = 0; i < 4; ++i) {
    x[i] = g.motors[i].pos.x();
    y[i] = g.motors[i].pos.y();
    spin[i] = g.motors[i].spin;
  }
  angle_rate_controller_set_motor_geometry(x, y, spin);
}

// The roll-mix signs this geometry produces (for logging): -sign(y) per motor.
// The firmware DEFAULT is {-1,-1,+1,+1}; a Y-mirrored airframe (e.g. an S500
// with M1/M2 on -y) needs {+1,+1,-1,-1} — the exact inverse — so flying the
// default mix on it inverts roll and topples. Surfacing this catches a mix that
// didn't get pushed to the firmware before arming.
QString rollMixString(const vsim::GeometryConfig& g) {
  QString s;
  for (int i = 0; i < 4; ++i)
    s += (g.motors[i].pos.y() >= 0.0f ? "-" : "+");
  return s;
}


constexpr const char* kRepoRootSettingKey = "simulator/repoRoot";
constexpr const char* kLogDirSettingKey   = "simulator/logDir";
constexpr const char* kGeomGroup          = "simulator/geometry";
constexpr const char* kWorldGroup         = "simulator/world";
constexpr const char* kAudioKey           = "simulator/propAudio";
constexpr const char* kImuHzKey           = "simulator/imuHz";
constexpr const char* kPhysHzKey          = "simulator/physicsHz";
constexpr const char* kPoseHzKey          = "simulator/poseHz";
// Defaults: IMU/firmware loop at 1 kHz (matches real hardware so PID tuning
// transfers); physics at 8 kHz (8 RK4 substeps/sample — finer integration, same
// sample rate); 60 Hz render.
constexpr int kDefImuHz = 1000, kDefPhysHz = 8000, kDefPoseHz = 60;
constexpr const char* kRcEnableKey        = "simulator/rcEnable";
constexpr const char* kRcSourceKey        = "simulator/rcSource";   // 0=js 1=uart
constexpr const char* kRcPathKey          = "simulator/rcPath";     // joystick dev
constexpr const char* kRcUartPathKey      = "simulator/rcUartPath"; // serial dev
constexpr const char* kRcBaudKey          = "simulator/rcBaud";
constexpr const char* kRcMapAxisKey       = "simulator/rcMapAxis";   // + func
constexpr const char* kRcMapInvKey        = "simulator/rcMapInv";    // + func

// Default joystick axis for each function (roll,pitch,throttle,yaw,arm).
constexpr int kRcDefaultAxis[5] = {0, 1, 2, 3, 4};
constexpr const char* kRcFuncName[5] = {"Roll", "Pitch", "Throttle", "Yaw", "Arm"};

QString defaultRepoRoot() {
  QByteArray env = qgetenv("VAYU_REPO");
  if (!env.isEmpty()) return QString::fromUtf8(env);
  return QDir::homePath() + "/Documents/Drone/stack/vayu";
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

  // Stand up the RC bridge NOW, before any vayu_sitl_start: the firmware's
  // RC feeder reads $VAYU_UART_RC_PATH once at boot, so the pty + env must
  // exist beforehand. The bridge runs continuously and streams a neutral
  // frame until enabled; the checkbox toggles joystick influence live.
  m_rc = new RcBridge(this);
  applyRcSource();  // pushes source + device path + baud to the bridge
  m_rc->setEnabled(m_rcEnable->isChecked());
  QString rcErr;
  if (m_rc->openPty(&rcErr)) {
    qputenv("VAYU_UART_RC_PATH", m_rc->slavePath().toLocal8Bit());
    connect(m_rc, &RcBridge::logLine, this,
            [this](const QString& s) { appendLog("rc", s); });
    connect(m_rc, &RcBridge::channelsUpdated, this,
            [this](int r, int pi, int t, int y, int a, int c6) {
              if (m_rcReadout)
                m_rcReadout->setText(QString("RC: R%1 P%2 T%3 Y%4 A%5 C6:%6%7")
                                         .arg(r).arg(pi).arg(t).arg(y).arg(a).arg(c6)
                                         .arg(c6 > 1500 ? " (ACRO)" : " (STAB)"));
            });
    connect(m_rc, &RcBridge::axesUpdated, this,
            [this](QVector<int> au, QVector<int> bu) {
              if (!m_rcAxesLabel) return;
              QString s = QStringLiteral("Ax");
              for (int i = 0; i < au.size(); ++i) s += QString(" %1:%2").arg(i).arg(au[i]);
              s += QStringLiteral("  Btn");
              for (int i = 0; i < bu.size(); ++i) s += QString(" %1:%2").arg(i).arg(bu[i]);
              m_rcAxesLabel->setText(s);
            });
    // Apply the persisted channel→axis mapping to the bridge.
    for (int f = 0; f < 5; ++f)
      if (m_rcAxisCombo[f])
        m_rc->setMapping(f, m_rcAxisCombo[f]->currentData().toInt(),
                         m_rcInvert[f]->isChecked());
    m_rc->start();
  } else {
    delete m_rc;
    m_rc = nullptr;
  }

  // RC UART starts disconnected; reflect that on the button and gate it to the
  // UART source. (The bridge's uartConnected_ already defaults false, so it has
  // not opened any tty.)
  setRcUartConnected(false);
  if (m_rcConnect && m_rcSource)
    m_rcConnect->setEnabled(m_rcSource->currentData().toInt() == RcBridge::Uart);

  // If another subsystem (board telemetry) claims the RC port, give it up.
  connect(&PortArbiter::instance(), &PortArbiter::revoked, this,
          [this](const QString&, QObject* owner) {
            if (owner == this && m_rcUartConnected) {
              appendLog("rc", tr("RC UART port taken by another connection — "
                                  "disconnected."));
              setRcUartConnected(false);
            }
          });
}

SimulatorWidget::~SimulatorWidget() {
  // Detach the UART2 callback FIRST. The firmware threads outlive this
  // widget (vayu_sitl is one-shot — they run until the process exits), and
  // would otherwise invoke the trampoline on a half-destroyed widget:
  // "QMetaObject::invokeMethod: No such method QWidget::onUartBytes" and a
  // dangling-pointer crash risk.
  if (m_ifaceInit) {
    vsim_iface_set_uart2_callback(&m_iface, nullptr, nullptr);
  }
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
  root->setSpacing(10);

  // ---- Header: title + mode tabs (Vehicle | World) + back ----
  {
    auto* header = new QHBoxLayout();
    auto* title = new QLabel(tr("Simulator"), this);
    title->setStyleSheet(
        QString("color: %1; font-size: 18px; font-weight: bold;")
            .arg(Theme::hex(Theme::kAccent)));
    header->addWidget(title);
    header->addSpacing(18);

    m_vehicleTab = new QPushButton(tr("Vehicle"), this);
    m_worldTab   = new QPushButton(tr("World"), this);
    m_tuneTab    = new QPushButton(tr("Autotune"), this);
    auto* grp = new QButtonGroup(this);
    grp->setExclusive(true);
    for (auto* b : {m_vehicleTab, m_worldTab, m_tuneTab}) {
      b->setCheckable(true);
      b->setObjectName("ToggleButton");
      b->setCursor(Qt::PointingHandCursor);
    }
    m_vehicleTab->setToolTip(tr("Configure the airframe (locked while simulating)"));
    m_worldTab->setToolTip(tr("Design the world and run the simulation"));
    m_tuneTab->setToolTip(tr("Auto-tune the PID gains for the current vehicle"));
    grp->addButton(m_vehicleTab, 0);
    grp->addButton(m_worldTab, 1);
    grp->addButton(m_tuneTab, 2);
    m_vehicleTab->setChecked(true);
    header->addWidget(m_vehicleTab);
    header->addWidget(m_worldTab);
    header->addWidget(m_tuneTab);

    header->addStretch();
    auto* backBtn = new ui::BackButton(this);
    backBtn->setToolTip(tr("Return to home"));
    connect(backBtn, &QPushButton::clicked, this,
            [this] { emit backToHomeRequested(); });
    header->addWidget(backBtn);
    root->addLayout(header);

    connect(grp, &QButtonGroup::idClicked, this, [this](int id) { setMode(id); });
  }

  // ---- Split: 3D viewport | stacked properties panel ----
  auto* splitter = new QSplitter(Qt::Horizontal, this);
  root->addWidget(splitter, 1);

  m_renderer = new vsim::SimRendererWidget(splitter);
  splitter->addWidget(m_renderer);

  // FPV telemetry HUD: full-viewport overlay child, shown while the sim
  // runs. It's mouse-transparent so camera orbit still works; an event
  // filter keeps it sized to the viewport.
  m_hud = new SimHudWidget(m_renderer);
  m_hud->setGeometry(m_renderer->rect());
  m_hud->hide();

  // Compact artificial horizon pinned to the viewport's top-right corner — a
  // persistent attitude reference (the full FPV HUD only shows while running).
  // Sized/positioned by the eventFilter; fed from the sim snapshot in updateHud.
  m_horizon = new HorizonHud(m_renderer);
  m_horizon->setFixedSize(200, 138);
  m_horizon->show();
  m_horizon->raise();

  m_renderer->installEventFilter(this);

  m_rightStack = new QStackedWidget(splitter);
  splitter->addWidget(m_rightStack);

  // ===== Vehicle page: airframe geometry + motor editor =====
  {
    m_geomEditor = new GeometryEditorWidget();
    // Persist on every config change (mesh load, edit, gizmo) — not just on
    // Apply — so the last-selected vehicle is restored on the next launch via
    // restoreGeometry() below.
    connect(m_geomEditor, &GeometryEditorWidget::previewUpdated, this, [this] {
      applyGeometryToRenderer();
      persistGeometry(m_geomEditor->config());
    });
    // Gizmo edits in the 3D view flow back into the editor (CoM frame).
    connect(m_renderer, &vsim::SimRendererWidget::motorEdited, m_geomEditor,
            &GeometryEditorWidget::setMotorFromGizmo);
    connect(m_geomEditor, &GeometryEditorWidget::geometryApplied, this, [this] {
      applyGeometryToRenderer();
      pushFirmwareMotorGeometry(m_geomEditor->physicsConfig());
      if (m_sim) m_sim->sendGeometry(m_geomEditor->physicsConfig());
      persistGeometry(m_geomEditor->config());
      appendLog("geom", tr("geometry applied (m=%1 kg)")
                            .arg(m_geomEditor->config().mass));
    });
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(m_geomEditor);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_rightStack->addWidget(scroll);   // index 0 = Vehicle
  }

  // ===== World page: environment + aerodynamics + simulation =====
  {
    auto* page = new QWidget();
    auto* pv = new QVBoxLayout(page);
    pv->setContentsMargins(0, 0, 0, 0);
    pv->setSpacing(6);

    m_worldEditor = new WorldEditorWidget(page);
    connect(m_worldEditor, &WorldEditorWidget::worldApplied, this, [this] {
      if (m_sim) m_sim->sendWorld(m_worldEditor->config());
      persistWorld(m_worldEditor->config());
      appendLog("world", tr("world applied (g=%1 m/s²)")
                             .arg(m_worldEditor->config().gravity));
    });
    // Obstacles are visual (phase 1): live-update the 3D view + persist on any
    // add/remove/edit, no Apply needed.
    connect(m_worldEditor, &WorldEditorWidget::obstaclesChanged, this, [this] {
      if (m_renderer) m_renderer->setObstacles(m_worldEditor->config().obstacles);
      if (m_sim) m_sim->sendObstacles(m_worldEditor->config().obstacles);
      persistWorld(m_worldEditor->config());
    });
    // 3D gizmo editing of obstacles <-> the editor list/form.
    connect(m_renderer, &vsim::SimRendererWidget::obstacleEdited, m_worldEditor,
            &WorldEditorWidget::setObstacleFromGizmo);
    connect(m_renderer, &vsim::SimRendererWidget::obstacleSelected, m_worldEditor,
            &WorldEditorWidget::selectObstacleRow);
    connect(m_worldEditor, &WorldEditorWidget::obstacleSelectionChanged,
            m_renderer, &vsim::SimRendererWidget::selectObstacle);
    // Imported world mesh (visual): (re)load + render + persist on any change.
    connect(m_worldEditor, &WorldEditorWidget::worldMeshChanged, this, [this] {
      loadWorldMeshToRenderer();
      persistWorld(m_worldEditor->config());
    });
    pv->addWidget(m_worldEditor);

    auto* simSec = new CollapsibleSection(tr("Simulation"), page);
    auto* simBody = new QWidget();
    auto* sv = new QVBoxLayout(simBody);
    sv->setContentsMargins(0, 0, 0, 0);
    sv->setSpacing(6);

    m_simStatusLabel = new QLabel(tr("● Stopped"), simBody);
    m_simStatusLabel->setStyleSheet(
        QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
    sv->addWidget(m_simStatusLabel);

    auto* runRow = new QHBoxLayout();
    m_simStartBtn = new ui::SuccessButton(tr("Start"), simBody);
    m_simStartBtn->setToolTip(tr("Boot firmware (first time) + start the sim"));
    connect(m_simStartBtn, &QPushButton::clicked, this,
            &SimulatorWidget::startInAppSim);
    m_simStopBtn = new ui::DangerButton(tr("Stop"), simBody);
    m_simStopBtn->setEnabled(false);
    m_simStopBtn->setToolTip(tr("Stop the sim worker (firmware stays alive)"));
    connect(m_simStopBtn, &QPushButton::clicked, this,
            &SimulatorWidget::stopInAppSim);
    m_simResetBtn = new ui::GhostButton(tr("Reset"), simBody);
    m_simResetBtn->setEnabled(false);
    m_simResetBtn->setToolTip(tr("Reset the airframe to the spawn pose"));
    connect(m_simResetBtn, &QPushButton::clicked, this,
            [this] { if (m_sim) m_sim->sendReset(); });
    runRow->addWidget(m_simStartBtn);
    runRow->addWidget(m_simStopBtn);
    runRow->addWidget(m_simResetBtn);
    runRow->addStretch();
    m_fpvCheck = new QCheckBox(tr("FPV cam"), simBody);
    m_fpvCheck->setToolTip(tr("Onboard first-person camera that rides the drone "
                              "(looks forward). Off = 3rd-person orbit. "
                              "Available while the sim is running."));
    m_fpvCheck->setEnabled(false);   // enabled on Start (locks to the drone)
    connect(m_fpvCheck, &QCheckBox::toggled, this,
            [this](bool on) { if (m_renderer) m_renderer->setFpv(on); });
    runRow->addWidget(m_fpvCheck);

    auto* audio = new QCheckBox(tr("Prop audio"), simBody);
    audio->setToolTip(tr("Propeller sound synthesized from motor rpm "
                         "(pitch + loudness rise with throttle)."));
    {
      QSettings st;
      audio->setChecked(st.value(kAudioKey, false).toBool());
      m_propAudio.setEnabled(audio->isChecked());
    }
    connect(audio, &QCheckBox::toggled, this, [this](bool on) {
      QSettings().setValue(kAudioKey, on);
      m_propAudio.setEnabled(on);
    });
    runRow->addWidget(audio);
    sv->addLayout(runRow);

    // -- Loop rates (IMU/firmware, physics substeps, render) --
    {
      QSettings st;
      auto* rg = new QGridLayout();
      rg->setHorizontalSpacing(6);
      auto mkCombo = [&](const QList<int>& opts, int def, const char* key,
                         const QString& tip) {
        auto* c = new QComboBox(simBody);
        for (int v : opts) c->addItem(QString::number(v) + " Hz", v);
        const int cur = st.value(key, def).toInt();
        int idx = c->findData(cur);
        if (idx < 0) { c->addItem(QString::number(cur) + " Hz", cur); idx = c->count() - 1; }
        c->setCurrentIndex(idx);
        c->setToolTip(tip);
        connect(c, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                [this, c, key] {
                  QSettings().setValue(key, c->currentData().toInt());
                  pushRatesToSim();
                });
        return c;
      };
      auto* imuC = mkCombo({200, 500, 1000, 2000}, kDefImuHz, kImuHzKey,
                           tr("IMU emit + firmware inner-loop rate. Match your "
                              "real hardware (e.g. 1 kHz) so PID tuning transfers."));
      auto* physC = mkCombo({1000, 2000, 4000, 8000, 10000}, kDefPhysHz, kPhysHzKey,
                            tr("RK4 physics rate. Run as substeps per IMU sample "
                               "(snapped to a multiple of the IMU rate): higher = "
                               "finer integration / less collision tunnelling, same "
                               "sample rate."));
      auto* poseC = mkCombo({30, 60, 120}, kDefPoseHz, kPoseHzKey,
                            tr("Pose / 3D-render update rate."));
      rg->addWidget(new QLabel(tr("IMU/loop:")), 0, 0);   rg->addWidget(imuC, 0, 1);
      rg->addWidget(new QLabel(tr("Physics:")), 0, 2);    rg->addWidget(physC, 0, 3);
      rg->addWidget(new QLabel(tr("Render:")), 1, 0);     rg->addWidget(poseC, 1, 1);
      sv->addLayout(rg);
    }

    m_simPoseLabel = new QLabel(tr("pose: -"), simBody);
    m_simPoseLabel->setStyleSheet(
        QString("color: %1; font-family: monospace;")
            .arg(Theme::hex(Theme::kAccent)));
    sv->addWidget(m_simPoseLabel);

    // RC transmitter (USB joystick) → firmware RC. Must be set before the
    // first Start (firmware reads the RC path once at boot).
    {
      QSettings st;
      auto* rcRow = new QHBoxLayout();
      m_rcEnable = new QCheckBox(tr("RC transmitter"), simBody);
      m_rcEnable->setChecked(st.value(kRcEnableKey, true).toBool());
      m_rcEnable->setToolTip(tr("Feed a USB RC transmitter (joystick) into "
                                "the sim. Enable before the first Start."));
      connect(m_rcEnable, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(kRcEnableKey, on);
        if (m_rc) m_rc->setEnabled(on);   // live, no restart
      });
      rcRow->addWidget(m_rcEnable);

      // RC input source: a USB joystick (axes → channels) or a UART that
      // streams CSV microsecond frames straight off a serial port.
      m_rcSource = new QComboBox(simBody);
      m_rcSource->addItem(tr("USB joystick"), RcBridge::Joystick);
      m_rcSource->addItem(tr("UART (CSV)"), RcBridge::Uart);
      m_rcSource->setCurrentIndex(
          st.value(kRcSourceKey, RcBridge::Joystick).toInt() == RcBridge::Uart
              ? 1
              : 0);
      m_rcSource->setToolTip(tr("RC input source. USB joystick maps axes to "
                                "channels; UART reads CSV µs frames directly."));
      connect(m_rcSource, QOverload<int>::of(&QComboBox::currentIndexChanged),
              this, [this] { applyRcSource(); });
      rcRow->addWidget(m_rcSource);

      // Editable combo: type a device path, or drop down to pick an enumerated
      // serial port. Items + current text filled by applyRcSource().
      m_rcPath = new QComboBox(simBody);
      m_rcPath->setEditable(true);
      m_rcPath->setInsertPolicy(QComboBox::NoInsert);
      m_rcPath->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
      // Each enumerated item shows the friendly description ("ttyUSB0 — FT232
      // USB UART") but carries the bare /dev path in itemData. Picking one
      // writes that path into the editable field — what the bridge opens.
      connect(m_rcPath, QOverload<int>::of(&QComboBox::activated), this,
              [this](int i) {
                const QVariant d = m_rcPath->itemData(i);
                if (d.isValid()) m_rcPath->setEditText(d.toString());
                commitRcPath();
              });
      connect(m_rcPath->lineEdit(), &QLineEdit::editingFinished, this,
              [this] { commitRcPath(); });
      rcRow->addWidget(m_rcPath, 1);

      m_rcBaud = new QComboBox(simBody);
      for (int b : {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600})
        m_rcBaud->addItem(QString::number(b), b);
      {
        const int idx = m_rcBaud->findData(st.value(kRcBaudKey, 115200).toInt());
        m_rcBaud->setCurrentIndex(idx >= 0 ? idx : 4);
      }
      m_rcBaud->setToolTip(tr("UART baud (UART source only)."));
      connect(m_rcBaud, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
              [this] {
                const int b = m_rcBaud->currentData().toInt();
                QSettings().setValue(kRcBaudKey, b);
                if (m_rc) m_rc->setUartBaud(b);
              });
      rcRow->addWidget(m_rcBaud);

      // Explicit Connect for the UART source. The sim does NOT open the serial
      // device until this is pressed, so it can't fight the board telemetry for
      // the port. Disconnected by default; only meaningful for the UART source.
      m_rcConnect = new QPushButton(tr("Connect"), simBody);
      m_rcConnect->setObjectName("ToggleButton");
      m_rcConnect->setCheckable(true);
      m_rcConnect->setToolTip(tr("Connect/disconnect the RC UART device. The sim "
                                 "only grabs the serial port while connected."));
      connect(m_rcConnect, &QPushButton::clicked, this,
              [this] { setRcUartConnected(!m_rcUartConnected); });
      rcRow->addWidget(m_rcConnect);

      // Acro (rate) flight-mode toggle. This IS the simulated RC ch6 switch: it
      // drives RcBridge ch6, the firmware reads ch6 and resolves the mode (no GCS
      // override, so the switch is authoritative — matching real RC). Off = angle
      // / stabilize (bank-angle limited); on = acro (body-rate, flips allowed).
      m_acroChk = new QCheckBox(tr("Acro (ch6)"), simBody);
      m_acroChk->setToolTip(tr("Acro / rate mode on RC channel 6: sticks command "
                               "body rate directly, no bank-angle limit. This is "
                               "the ch6 switch — the firmware follows it."));
      const bool acroOn = st.value(QStringLiteral("sim/rcAcro"), false).toBool();
      m_acroChk->setChecked(acroOn);
      if (m_rc) m_rc->setAcro(acroOn);
      flight_mode_release();   // ensure no stale GCS override blocks the switch
      connect(m_acroChk, &QCheckBox::toggled, this, [this](bool on) {
        QSettings().setValue(QStringLiteral("sim/rcAcro"), on);
        if (m_rc) m_rc->setAcro(on);   // toggles ch6 -> firmware switches mode
      });
      rcRow->addWidget(m_acroChk);
      sv->addLayout(rcRow);

      m_flightModeLabel = new QLabel(tr("mode: —"), simBody);
      m_flightModeLabel->setStyleSheet(
          QString("color: %1; font-family: monospace; font-size: 11px;")
              .arg(Theme::hex(Theme::kTextMuted)));
      sv->addWidget(m_flightModeLabel);

      m_rcReadout = new QLabel(tr("RC: —"), simBody);
      m_rcReadout->setStyleSheet(
          QString("color: %1; font-family: monospace; font-size: 11px;")
              .arg(Theme::hex(Theme::kTextMuted)));
      sv->addWidget(m_rcReadout);

      // Live per-axis readout — move a control and see which axis changes.
      m_rcAxesLabel = new QLabel(tr("Axes: —"), simBody);
      m_rcAxesLabel->setStyleSheet(
          QString("color: %1; font-family: monospace; font-size: 11px;")
              .arg(Theme::hex(Theme::kTextDim)));
      sv->addWidget(m_rcAxesLabel);

      // Channel → axis mapping (+ invert), one row per function.
      auto* mapGrid = new QGridLayout();
      mapGrid->setHorizontalSpacing(6);
      mapGrid->setVerticalSpacing(2);
      for (int f = 0; f < 5; ++f) {
        const int savedSrc =
            st.value(QString(kRcMapAxisKey) + QString::number(f), kRcDefaultAxis[f]).toInt();
        const bool savedInv =
            st.value(QString(kRcMapInvKey) + QString::number(f), false).toBool();
        mapGrid->addWidget(new QLabel(tr(kRcFuncName[f]), simBody), f, 0);
        auto* combo = new QComboBox(simBody);
        for (int a = 0; a < 6; ++a) combo->addItem(tr("Axis %1").arg(a), a);
        for (int b = 0; b < 3; ++b)
          combo->addItem(tr("Button %1").arg(b), RcBridge::kButtonBase + b);
        const int idx = combo->findData(savedSrc);
        combo->setCurrentIndex(idx >= 0 ? idx : 0);
        mapGrid->addWidget(combo, f, 1);
        auto* inv = new QCheckBox(tr("invert"), simBody);
        inv->setChecked(savedInv);
        mapGrid->addWidget(inv, f, 2);
        m_rcAxisCombo[f] = combo;
        m_rcInvert[f] = inv;
        connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                [this, f] { pushRcMapping(f); });
        connect(inv, &QCheckBox::toggled, this, [this, f] { pushRcMapping(f); });
      }
      sv->addLayout(mapGrid);

      // Software arm — for TXs with no usable arm switch. Overrides the
      // Arm channel: checked = armed (2000), unchecked = use the mapping.
      m_swArm = new QCheckBox(tr("Software arm (override)"), simBody);
      m_swArm->setToolTip(tr("Force the arm channel high. Use with throttle "
                             "down to ARM when your TX has no arm switch."));
      connect(m_swArm, &QCheckBox::toggled, this, [this](bool on) {
        if (m_rc) m_rc->setArmOverride(on ? 1 : -1);
      });
      sv->addWidget(m_swArm);

      // Now that the device field, baud, and mapping widgets exist, apply the
      // saved source: fills the path field and enables/disables baud+mapping.
      applyRcSource();
    }

    auto* repoRow = new QHBoxLayout();
    repoRow->addWidget(new QLabel(tr("Repo root:"), simBody));
    m_repoRootEdit = new QLineEdit(m_repoRoot, simBody);
    m_repoRootEdit->setToolTip(tr("Path to the vayu repo. Used for log paths."));
    connect(m_repoRootEdit, &QLineEdit::editingFinished, this, [this] {
      m_repoRoot = m_repoRootEdit->text();
      QSettings().setValue(kRepoRootSettingKey, m_repoRoot);
    });
    repoRow->addWidget(m_repoRootEdit, 1);
    sv->addLayout(repoRow);

    auto* logRow = new QHBoxLayout();
    logRow->addWidget(new QLabel(tr("Log dir:"), simBody));
    m_logDirEdit = new QLineEdit(m_logDir, simBody);
    m_logDirEdit->setToolTip(tr("Directory the per-run UART2 byte log is written into"));
    connect(m_logDirEdit, &QLineEdit::editingFinished, this, [this] {
      m_logDir = m_logDirEdit->text();
      QSettings().setValue(kLogDirSettingKey, m_logDir);
    });
    logRow->addWidget(m_logDirEdit, 1);
    auto* openBtn = new ui::GhostButton(tr("Open"), simBody);
    openBtn->setToolTip(tr("Open the log directory in your file manager"));
    connect(openBtn, &QPushButton::clicked, this, [this] {
      QDesktopServices::openUrl(QUrl::fromLocalFile(m_logDir));
    });
    logRow->addWidget(openBtn);
    sv->addLayout(logRow);

    m_logPathLabel = new QLabel(tr("Current log: (none)"), simBody);
    m_logPathLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kTextMuted)));
    m_logPathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    sv->addWidget(m_logPathLabel);

    m_log = new QPlainTextEdit(simBody);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    m_log->setMinimumHeight(150);
    sv->addWidget(m_log);

    simSec->setContentWidget(simBody);
    pv->addWidget(simSec);
    pv->addStretch();

    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_rightStack->addWidget(scroll);   // index 1 = World
  }

  // ===== Autotune page: PID gain search against the current vehicle =====
  {
    auto* page = new QWidget();
    buildAutotunePage(page);
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_rightStack->addWidget(scroll);   // index 2 = Autotune
  }

  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);

  // Restore persisted configs (geometry setConfig also loads the mesh +
  // recomputes, firing previewUpdated -> applyGeometryToRenderer).
  m_geomEditor->setConfig(restoreGeometry());
  m_worldEditor->setConfig(restoreWorld());
  if (m_renderer) m_renderer->setObstacles(m_worldEditor->config().obstacles);
  loadWorldMeshToRenderer();

  setMode(0);   // start in Vehicle (sim stopped)
}

void SimulatorWidget::setMode(int mode) {
  if (!m_rightStack) return;
  // Vehicle is config-only and locked while the sim runs.
  if (mode == 0 && m_sim) mode = 1;
  m_rightStack->setCurrentIndex(mode);
  if (m_vehicleTab) m_vehicleTab->setChecked(mode == 0);
  if (m_worldTab)   m_worldTab->setChecked(mode == 1);
  if (m_tuneTab)    m_tuneTab->setChecked(mode == 2);
  // Motor gizmos belong to Vehicle mode; obstacle gizmos to World mode. Both
  // only when stopped (a running sim owns the airframe + the live world).
  if (m_renderer) {
    m_renderer->setMotorsEditable(mode == 0 && !m_sim);
    m_renderer->setObstacleEditMode(mode == 1 && !m_sim);
    // Vehicle = just the airframe; World = the whole world.
    m_renderer->setWorldVisible(mode == 1);
    // Free-roam the world only while stopped; a running sim locks a 3rd-person
    // (orbit) view on the drone, with the FPV toggle for onboard.
    m_renderer->setFreeFly(mode == 1 && !m_sim);
  }
}

void SimulatorWidget::buildAutotunePage(QWidget* page) {
  auto* v = new QVBoxLayout(page);
  v->setContentsMargins(10, 10, 10, 10);
  v->setSpacing(8);

  auto* title = new QLabel(tr("PID Autotune"), page);
  title->setStyleSheet(QString("color:%1; font-size:15px; font-weight:bold;")
                           .arg(Theme::hex(Theme::kAccent)));
  v->addWidget(title);

  auto* info = new QLabel(
      tr("Searches the inner rate + outer angle gains for the CURRENT vehicle "
         "geometry, in an isolated headless test-rig sim (tools/autotune). The "
         "live sim keeps running; tuned gains persist to the shared store and "
         "load on the next sim start."),
      page);
  info->setWordWrap(true);
  info->setStyleSheet(QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
  v->addWidget(info);

  auto* form = new QGridLayout();
  int r = 0;
  form->addWidget(new QLabel(tr("Optimizer:"), page), r, 0);
  m_tuneOptimizer = new QComboBox(page);
  m_tuneOptimizer->addItems({"hybrid", "portfolio", "structured", "spsa",
                             "nelder-mead", "coordinate", "fdgd", "random"});
  m_tuneOptimizer->setToolTip(tr("hybrid/portfolio explore then refine (best); "
                                 "structured = manual-style one-gain-at-a-time "
                                 "sweep (P→D→I→outer); spsa is fast."));
  form->addWidget(m_tuneOptimizer, r++, 1);

  form->addWidget(new QLabel(tr("Budget (rollouts):"), page), r, 0);
  m_tuneBudget = new QSpinBox(page);
  m_tuneBudget->setRange(5, 200);
  m_tuneBudget->setValue(30);
  m_tuneBudget->setToolTip(tr("More rollouts = better tune, slower (~6 s each)."));
  form->addWidget(m_tuneBudget, r++, 1);

  form->addWidget(new QLabel(tr("Excitation (µs):"), page), r, 0);
  m_tuneStep = new QSpinBox(page);
  m_tuneStep->setRange(1550, 1950);
  m_tuneStep->setSingleStep(25);
  m_tuneStep->setValue(1800);
  m_tuneStep->setToolTip(tr("Doublet stick amplitude (1500=center, 2000=full; "
                            "~21° at 1800). Lower it (e.g. 1650) to soften the "
                            "excitation on twitchy airframes so a marginal seed "
                            "doesn't flip."));
  form->addWidget(m_tuneStep, r++, 1);

  form->addWidget(new QLabel(tr("Repeats / eval:"), page), r, 0);
  m_tuneRepeats = new QSpinBox(page);
  m_tuneRepeats->setRange(1, 5);
  m_tuneRepeats->setValue(2);
  m_tuneRepeats->setToolTip(tr("Rollouts averaged per evaluation (--repeats). "
                               "Higher = less noisy cost, proportionally slower."));
  form->addWidget(m_tuneRepeats, r++, 1);

  form->addWidget(new QLabel(tr("Rig tether:"), page), r, 0);
  m_tuneTether = new QDoubleSpinBox(page);
  m_tuneTether->setRange(0.0, 100.0);
  m_tuneTether->setSingleStep(5.0);
  m_tuneTether->setValue(30.0);
  m_tuneTether->setToolTip(tr("Soft-rig spring stiffness [1/s²] (--rig-tether). The "
                              "body translates during the doublet so the accel sees "
                              "free-flight thrust-tilt (estimator-aware). 0 = legacy "
                              "hard pin (no translation)."));
  form->addWidget(m_tuneTether, r++, 1);

  m_tuneYaw = new QCheckBox(tr("Tune yaw too"), page);
  m_tuneYaw->setToolTip(tr("Also tune the yaw RATE loop (--yaw): yaw_rate kp/ki/kd "
                           "+ gyro LPF, excited by a yaw-rate doublet. Yaw is "
                           "rate-controlled, so there is no yaw angle gain to tune."));
  form->addWidget(m_tuneYaw, r++, 1);
  m_tuneCompare = new QCheckBox(tr("Compare all optimizers"), page);
  m_tuneCompare->setToolTip(tr("Run every optimizer on equal budgets and keep the "
                               "best (--compare). Much slower (N× the budget)."));
  form->addWidget(m_tuneCompare, r++, 1);
  m_tuneBuzz = new QCheckBox(tr("Throttle buzz check"), page);
  m_tuneBuzz->setChecked(true);
  m_tuneBuzz->setToolTip(tr("Penalize gains that limit-cycle at hover+ throttle "
                            "(off = --no-buzz-check). The rig can't see this; the "
                            "check sweeps throttle and scores rate-output chatter. "
                            "Adds ~3 s per eval but stops a buzzy tune from winning."));
  form->addWidget(m_tuneBuzz, r++, 1);
  m_tuneValidate = new QCheckBox(tr("Free-flight validation"), page);
  m_tuneValidate->setChecked(true);
  m_tuneValidate->setToolTip(tr("After the search, lift off and step-recover each "
                                "axis to confirm the winner flies (off = "
                                "--no-validate)."));
  form->addWidget(m_tuneValidate, r++, 1);
  m_tuneApply = new QCheckBox(tr("Apply + persist best gains on finish"), page);
  m_tuneApply->setChecked(true);
  form->addWidget(m_tuneApply, r++, 1);
  m_tunePlot = new QCheckBox(tr("Live dashboard (attitude + cost window)"), page);
  m_tunePlot->setChecked(true);
  m_tunePlot->setToolTip(tr("Opens the autotuner's live plot: the drone's "
                            "attitude response to each excitation + cost "
                            "convergence, gains and per-axis traces."));
  form->addWidget(m_tunePlot, r++, 1);
  m_tuneVerbose = new QCheckBox(tr("Verbose log (print every eval)"), page);
  m_tuneVerbose->setToolTip(tr("Print each evaluation's gains + cost to the log "
                               "(--verbose)."));
  form->addWidget(m_tuneVerbose, r++, 1);
  v->addLayout(form);

  // --- Advanced: reproducibility seeds (rarely changed) -----------------
  auto* advGroup = new QGroupBox(tr("Advanced (reproducibility)"), page);
  auto* advForm = new QGridLayout(advGroup);
  int ar = 0;
  advForm->addWidget(new QLabel(tr("Optimizer seed:"), page), ar, 0);
  m_tuneSeed = new QSpinBox(page);
  m_tuneSeed->setRange(0, 1000000);
  m_tuneSeed->setValue(1);
  m_tuneSeed->setToolTip(tr("RNG seed for the optimizer's random choices "
                            "(--seed). Same seed = same search trajectory."));
  advForm->addWidget(m_tuneSeed, ar++, 1);
  advForm->addWidget(new QLabel(tr("Sim noise seed:"), page), ar, 0);
  m_tuneSimSeed = new QSpinBox(page);
  m_tuneSimSeed->setRange(0, 2000000000);
  m_tuneSimSeed->setValue(0xC0FFEE);   // DEFAULT_SIM_SEED in autotune.py
  m_tuneSimSeed->setToolTip(tr("Base seed for the sensor-noise reset (--sim-seed); "
                               "repeat i uses seed+i. Makes eval(x) reproducible. "
                               "0 = free-running noise (legacy)."));
  advForm->addWidget(m_tuneSimSeed, ar++, 1);
  v->addWidget(advGroup);

  // Persist every autotune option so a configured run is remembered next time
  // the GCS starts (the controls otherwise reset to defaults each session).
  {
    QSettings st;
    st.beginGroup(QStringLiteral("autotune"));
    m_tuneOptimizer->setCurrentText(
        st.value(QStringLiteral("optimizer"), m_tuneOptimizer->currentText()).toString());
    m_tuneBudget->setValue(st.value(QStringLiteral("budget"), m_tuneBudget->value()).toInt());
    m_tuneStep->setValue(st.value(QStringLiteral("stepUs"), m_tuneStep->value()).toInt());
    m_tuneRepeats->setValue(st.value(QStringLiteral("repeats"), m_tuneRepeats->value()).toInt());
    m_tuneTether->setValue(st.value(QStringLiteral("tether"), m_tuneTether->value()).toDouble());
    m_tuneSeed->setValue(st.value(QStringLiteral("seed"), m_tuneSeed->value()).toInt());
    m_tuneSimSeed->setValue(st.value(QStringLiteral("simSeed"), m_tuneSimSeed->value()).toInt());
    m_tuneYaw->setChecked(st.value(QStringLiteral("yaw"), m_tuneYaw->isChecked()).toBool());
    m_tuneCompare->setChecked(st.value(QStringLiteral("compare"), m_tuneCompare->isChecked()).toBool());
    m_tuneBuzz->setChecked(st.value(QStringLiteral("buzz"), m_tuneBuzz->isChecked()).toBool());
    m_tuneValidate->setChecked(st.value(QStringLiteral("validate"), m_tuneValidate->isChecked()).toBool());
    m_tuneApply->setChecked(st.value(QStringLiteral("apply"), m_tuneApply->isChecked()).toBool());
    m_tunePlot->setChecked(st.value(QStringLiteral("plot"), m_tunePlot->isChecked()).toBool());
    m_tuneVerbose->setChecked(st.value(QStringLiteral("verbose"), m_tuneVerbose->isChecked()).toBool());
    st.endGroup();
  }
  auto saveTune = [](const QString& key, const QVariant& val) {
    QSettings s;
    s.setValue(QStringLiteral("autotune/") + key, val);
  };
  connect(m_tuneOptimizer, &QComboBox::currentTextChanged, this,
          [saveTune](const QString& t) { saveTune(QStringLiteral("optimizer"), t); });
  connect(m_tuneBudget, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("budget"), v); });
  connect(m_tuneStep, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("stepUs"), v); });
  connect(m_tuneRepeats, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("repeats"), v); });
  connect(m_tuneTether, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [saveTune](double v) { saveTune(QStringLiteral("tether"), v); });
  connect(m_tuneSeed, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("seed"), v); });
  connect(m_tuneSimSeed, QOverload<int>::of(&QSpinBox::valueChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("simSeed"), v); });
  connect(m_tuneYaw, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("yaw"), v); });
  connect(m_tuneCompare, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("compare"), v); });
  connect(m_tuneBuzz, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("buzz"), v); });
  connect(m_tuneValidate, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("validate"), v); });
  connect(m_tuneApply, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("apply"), v); });
  connect(m_tunePlot, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("plot"), v); });
  connect(m_tuneVerbose, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("verbose"), v); });

  // --- Test-rig pose: pin the airframe and tilt it on the stand ---------
  auto* rigGroup = new QGroupBox(tr("Test-rig pose"), page);
  auto* rigV = new QVBoxLayout(rigGroup);
  m_rigEnable = new QCheckBox(tr("Rig mode (pin translation, free rotation)"), page);
  m_rigEnable->setToolTip(tr("Pins the airframe so you can tilt it on a stand. "
                             "Disarm first — armed, the controller fights the pose."));
  rigV->addWidget(m_rigEnable);
  auto* rigGrid = new QGridLayout();
  auto mkSlider = [&](const QString& name, int row) {
    auto* s = new QSlider(Qt::Horizontal, page);
    s->setRange(-180, 180);
    s->setValue(0);
    s->setEnabled(false);
    rigGrid->addWidget(new QLabel(name, page), row, 0);
    rigGrid->addWidget(s, row, 1);
    return s;
  };
  m_rigRoll = mkSlider(tr("Roll°"), 0);
  m_rigPitch = mkSlider(tr("Pitch°"), 1);
  m_rigYaw = mkSlider(tr("Yaw°"), 2);
  rigV->addLayout(rigGrid);
  m_rigReadout = new QLabel(tr("rig: off"), page);
  m_rigReadout->setStyleSheet(
      QString("color:%1; font-family:monospace; font-size:11px;")
          .arg(Theme::hex(Theme::kTextMuted)));
  rigV->addWidget(m_rigReadout);
  v->addWidget(rigGroup);

  connect(m_rigEnable, &QCheckBox::toggled, this, [this](bool on) {
    if (m_rigRoll) m_rigRoll->setEnabled(on);
    if (m_rigPitch) m_rigPitch->setEnabled(on);
    if (m_rigYaw) m_rigYaw->setEnabled(on);
    if (m_sim) m_sim->sendTestRig(on);
    if (on) {
      applyRigPose();
    } else {
      if (m_sim) m_sim->sendReset();           // back to level, free flight
      if (m_rigReadout) m_rigReadout->setText(tr("rig: off"));
    }
  });
  // Apply the pose ONCE per change, not on every intermediate drag value: each
  // sendRigPose is a full CTL_RESET, so streaming them while dragging floods the
  // daemon (vsim_d: reset spam), spikes the synthetic accel (gravity re-projected
  // at each tilt), and pegs the CPU. While dragging we only preview the numbers;
  // the reset fires on release (or immediately for a keyboard/click step).
  auto previewRig = [this] {
    if (m_rigReadout && m_rigRoll)
      m_rigReadout->setText(
          tr("rig: roll %1°  pitch %2°  yaw %3°  (release to apply)")
              .arg(m_rigRoll->value()).arg(m_rigPitch->value()).arg(m_rigYaw->value()));
  };
  for (QSlider* s : {m_rigRoll, m_rigPitch, m_rigYaw}) {
    connect(s, &QSlider::valueChanged, this, [this, s, previewRig] {
      if (s->isSliderDown()) previewRig();   // mid-drag: preview only
      else applyRigPose();                    // click / keyboard step: apply now
    });
    connect(s, &QSlider::sliderReleased, this, [this] { applyRigPose(); });
  }
  setRigControlsEnabled(m_sim != nullptr);   // disabled until the sim runs

  // Embedded convergence chart — cost per evaluation + best-so-far.
  m_tuneChart = new TuneChart(page);
  v->addWidget(m_tuneChart);

  auto* btnRow = new QHBoxLayout();
  m_tuneStart = new QPushButton(tr("Start Autotune"), page);
  m_tuneStop = new QPushButton(tr("Stop"), page);
  m_tuneStop->setEnabled(false);
  connect(m_tuneStart, &QPushButton::clicked, this, [this] { startAutotune(); });
  connect(m_tuneStop, &QPushButton::clicked, this, [this] { stopAutotune(); });
  btnRow->addWidget(m_tuneStart);
  btnRow->addWidget(m_tuneStop);
  btnRow->addStretch();
  v->addLayout(btnRow);

  m_tuneResult = new QLabel(tr("—"), page);
  m_tuneResult->setWordWrap(true);
  m_tuneResult->setStyleSheet(QString("color:%1; font-family:monospace; font-size:11px;")
                                  .arg(Theme::hex(Theme::kOk)));
  v->addWidget(m_tuneResult);

  m_tuneLog = new QPlainTextEdit(page);
  m_tuneLog->setReadOnly(true);
  m_tuneLog->setMaximumBlockCount(4000);
  m_tuneLog->setStyleSheet("font-family:monospace; font-size:10px;");
  v->addWidget(m_tuneLog, 1);
}

QString SimulatorWidget::exportVehicleGeometryJson() {
  const vsim::GeometryConfig g = m_geomEditor->physicsConfig();
  QJsonObject root;
  root["mass"] = g.mass;
  QJsonArray inertia;
  for (int i = 0; i < 9; ++i) inertia.append(g.inertia[i]);
  root["inertia"] = inertia;
  QJsonArray motors;
  for (const auto& m : g.motors) {
    QJsonObject mo;
    mo["pos"] = QJsonArray{m.pos.x(), m.pos.y(), m.pos.z()};
    mo["axis"] = QJsonArray{m.axis.x(), m.axis.y(), m.axis.z()};
    mo["spin"] = m.spin;
    mo["k_thrust"] = m.k_thrust;
    mo["k_moment"] = m.k_moment;
    mo["max_omega"] = m.max_omega;
    motors.append(mo);
  }
  root["motors"] = motors;
  QDir().mkpath(m_logDir);
  const QString path = QDir(m_logDir).absoluteFilePath("autotune_vehicle.json");
  QFile f(path);
  if (f.open(QIODevice::WriteOnly)) {
    f.write(QJsonDocument(root).toJson(QJsonDocument::Compact));
    f.close();
  }
  return path;
}

QString SimulatorWidget::exportWorldJson() {
  const vsim::WorldConfig w = m_worldEditor->config();
  QJsonObject root;
  root["gravity"] = w.gravity;
  root["ground_z"] = w.ground_z;
  root["restitution"] = w.restitution;
  root["linear_drag"] = w.linear_drag;
  root["angular_drag"] = w.angular_drag;          // the one that matters for damping
  root["ground_right_gain"] = w.ground_right_gain;
  root["ground_right_damp"] = w.ground_right_damp;
  QDir().mkpath(m_logDir);
  const QString path = QDir(m_logDir).absoluteFilePath("autotune_world.json");
  QFile f(path);
  if (f.open(QIODevice::WriteOnly)) {
    f.write(QJsonDocument(root).toJson(QJsonDocument::Compact));
    f.close();
  }
  return path;
}

void SimulatorWidget::startAutotune() {
  if (m_tuneProc || !m_geomEditor) return;
  const QString root = defaultRepoRoot();
  const QString script = root + "/tools/autotune/autotune.py";
  if (!QFileInfo::exists(script)) {
    m_tuneLog->appendPlainText(tr("[error] autotuner not found: %1").arg(script));
    return;
  }
  const QString geomJson = exportVehicleGeometryJson();
  const QString worldJson = exportWorldJson();   // tune the SAME world (drag!)
  const QString outJson = QDir(m_logDir).absoluteFilePath("autotune_gcs.json");

  // The tuner runs its own isolated SITL stack. Give it a known FIFO suffix so
  // the GCS can attach its renderer to that sim and show the tuning live. Stop
  // the interactive sim first (frees vsim_d/firmware) and lock Vehicle/World so
  // the airframe can't change mid-search.
  if (m_sim) stopInAppSim();
  if (m_vehicleTab) m_vehicleTab->setEnabled(false);
  if (m_worldTab) m_worldTab->setEnabled(false);
  const QString tuneSuffix =
      QStringLiteral("_attune%1").arg(QCoreApplication::applicationPid());

  QStringList args;
  args << script << "--geometry" << geomJson << "--world" << worldJson
       << "--optimizer" << m_tuneOptimizer->currentText()
       << "--budget" << QString::number(m_tuneBudget->value())
       << "--repeats" << QString::number(m_tuneRepeats->value())
       << "--step-us" << QString::number(m_tuneStep->value())
       << "--rig-tether" << QString::number(m_tuneTether->value())
       << "--seed" << QString::number(m_tuneSeed->value())
       << "--sim-seed" << QString::number(m_tuneSimSeed->value())
       << "--fifo-suffix" << tuneSuffix
       << "--out" << outJson << "--repo-root" << root;
  if (m_tuneYaw->isChecked()) args << "--yaw";
  if (m_tuneCompare->isChecked()) args << "--compare";
  if (m_tuneApply->isChecked()) args << "--apply";
  if (m_tunePlot->isChecked()) args << "--plot";
  if (m_tuneVerbose->isChecked()) args << "--verbose";
  if (!m_tuneValidate->isChecked()) args << "--no-validate";
  if (!m_tuneBuzz->isChecked()) args << "--no-buzz-check";

  m_tuneChart->reset();
  m_tuneProc = new QProcess(this);
  m_tuneProc->setProcessChannelMode(QProcess::MergedChannels);
  m_tuneProc->setWorkingDirectory(root + "/tools/autotune");
  connect(m_tuneProc, &QProcess::readyReadStandardOutput, this, [this] {
    const QByteArray buf = m_tuneProc->readAllStandardOutput();
    const QList<QByteArray> lines = buf.split('\n');
    for (const QByteArray& raw : lines) {
      const QString s = QString::fromUtf8(raw).trimmed();
      if (s.startsWith("#EVAL")) {              // structured progress -> chart
        const QStringList p = s.split(' ', Qt::SkipEmptyParts);
        if (p.size() >= 4) {
          const double cost = p[2].toDouble(), best = p[3].toDouble();
          m_tuneChart->addPoint(cost, best);
          m_tuneResult->setText(tr("eval %1   cost %2   best %3")
                                    .arg(p[1]).arg(cost, 0, 'f', 2).arg(best, 0, 'f', 2));
        }
        continue;
      }
      if (s.isEmpty() || s.startsWith("host_main") || s.contains("alive @") ||
          s.startsWith("QSocketNotifier") || s.startsWith("qt.qpa") ||
          s.startsWith("vsim_d:") || s.startsWith("vayu_sitl:"))
        continue;
      m_tuneLog->appendPlainText(s);
      if (s.startsWith("baseline cost")) {
        m_tuneChart->setBaseline(s.section(':', 1).toDouble());
      }
      if (s.startsWith("ABORT")) {
        m_tuneResult->setText(tr("⚠ unstable plant — see log (motor layout?)"));
        m_tuneResult->setStyleSheet("color:#ff8c50; font-weight:bold; font-size:11px;");
      } else if (s.startsWith("BEST:") || s.startsWith("gains:")) {
        m_tuneResult->setText(s);
      }
    }
  });
  connect(m_tuneProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this](int code, QProcess::ExitStatus) {
            m_tuneLog->appendPlainText(tr("[autotune finished, exit %1]").arg(code));
            if (m_tuneApply && m_tuneApply->isChecked())
              m_tuneLog->appendPlainText(
                  tr("[best gains persisted — restart the sim to fly them]"));
            if (m_tuneStart) m_tuneStart->setEnabled(true);
            if (m_tuneStop) m_tuneStop->setEnabled(false);
            detachTuneSim();          // stop mirroring, unlock Vehicle/World
            m_tuneProc->deleteLater();
            m_tuneProc = nullptr;
          });
  m_tuneLog->clear();
  m_tuneResult->setText(tr("running…"));
  // Confirm we're tuning the Vehicle-section geometry (physicsConfig), so it's
  // obvious which airframe the search runs against.
  {
    const vsim::GeometryConfig g = m_geomEditor->physicsConfig();
    const QString veh = m_geomEditor->loadedVehicleName();
    m_tuneLog->appendPlainText(
        tr("[geometry] Vehicle section%1: mass=%2 kg, inertia diag=[%3, %4, %5] kg·m²"
           "  (exported to autotune_vehicle.json)")
            .arg(veh.isEmpty() ? QString() : tr(" (%1)").arg(veh))
            .arg(g.mass, 0, 'f', 3)
            .arg(g.inertia[0], 0, 'g', 4)
            .arg(g.inertia[4], 0, 'g', 4)
            .arg(g.inertia[8], 0, 'g', 4));
  }
  m_tuneLog->appendPlainText(tr("[launching] python3 %1").arg(args.join(' ')));
  m_tuneStart->setEnabled(false);
  m_tuneStop->setEnabled(true);
  m_tuneProc->start(QStringLiteral("python3"), args);
  attachTuneSim(tuneSuffix);   // mirror the tuner's drone in the 3D view
}

void SimulatorWidget::stopAutotune() {
  if (m_tuneProc) m_tuneProc->kill();
  // finished() handler runs detachTuneSim(); detach now too in case kill races.
  detachTuneSim();
}

// Attach the renderer to the tuner's own SITL sim (pose FIFO at the agreed
// suffix), so the 3D view shows the live excitation while the search runs.
void SimulatorWidget::attachTuneSim(const QString& suffix) {
  if (m_tuneSim) return;
  const QString posePath = QStringLiteral("/tmp/vsim_pose%1").arg(suffix);
  m_tuneSim = new vsim::SimWorker(this);
  connect(m_tuneSim, &vsim::SimWorker::poseUpdated,
          m_renderer, &vsim::SimRendererWidget::setSnapshot);
  connect(m_tuneSim, &vsim::SimWorker::poseUpdated, this,
          [this](vsim::SimSnapshot snap) { updateHud(snap); });
  connect(m_tuneSim, &vsim::SimWorker::logLine, this,
          [this](const QString& s) { appendLog("tune-sim", s); });
  m_tuneSim->startAttach(posePath);
  if (m_hud) { m_hud->setGeometry(m_renderer->rect()); m_hud->raise(); m_hud->show(); }
  if (m_horizon) m_horizon->raise();
}

void SimulatorWidget::detachTuneSim() {
  if (m_tuneSim) {
    m_tuneSim->requestStop();
    if (!m_tuneSim->wait(2000)) { m_tuneSim->terminate(); m_tuneSim->wait(1000); }
    m_tuneSim->deleteLater();
    m_tuneSim = nullptr;
  }
  if (m_vehicleTab) m_vehicleTab->setEnabled(true);
  if (m_worldTab) m_worldTab->setEnabled(true);
}

void SimulatorWidget::updateHud(const vsim::SimSnapshot& s) {
  if (m_hud) m_hud->setSnapshot(s);
  if (m_horizon) {
    float roll, pitch, yaw;
    vsim::quatToEulerNED(s.att, &roll, &pitch, &yaw);
    m_horizon->setAttitude(roll, pitch, yaw);
  }
}

void SimulatorWidget::loadWorldMeshToRenderer() {
  if (!m_renderer || !m_worldEditor) return;
  const vsim::WorldConfig& w = m_worldEditor->config();
  if (w.worldMeshPath.isEmpty()) {
    m_renderer->setWorldMesh({}, {});
    if (m_sim) m_sim->clearWorldMesh();
    return;
  }
  // Bake the source up-axis into NED (up = -Z): Z-up needs a 180° flip about
  // X; Y-up a -90° rotation about X. Then a world-space placement offset (NED)
  // so the user can move the world off the drone's spawn. Translate is applied
  // last (M = T·R) so the offset is in final NED metres. The same baked mesh
  // drives both the render and the collision BVH, so they stay in lockstep.
  QMatrix4x4 xform;
  xform.translate(w.worldMeshOffset);
  if (w.worldUpAxis == 1) xform.rotate(-90.0f, 1, 0, 0);
  else                    xform.rotate(180.0f, 1, 0, 0);
  QString err;
  const vsim::LoadedMesh m =
      vsim::loadMesh(w.worldMeshPath, w.worldScale, xform, &err);
  if (!m.valid) {
    appendLog("world", tr("world mesh load failed: %1").arg(err));
    m_renderer->setWorldMesh({}, {});
    if (m_sim) m_sim->clearWorldMesh();
    return;
  }
  m_renderer->setWorldMesh(m.positions, m.normals, m.colors);
  appendLog("world", tr("world mesh loaded: %1 tris").arg(m.triangleCount()));

  // Hand the same baked geometry to the physics daemon as a collision BVH.
  // Render and collision therefore share one transform → they never disagree.
  if (m_sim) sendWorldMeshToSim(m);
}

void SimulatorWidget::sendWorldMeshToSim(const vsim::LoadedMesh& m) {
  if (!m_sim || !m.valid) return;
  const vsim::WorldConfig& w = m_worldEditor->config();

  // Flatten the QVector3D soup to the contiguous float[3*nverts] the BVH
  // builder expects (NED world space; transform already baked into positions).
  const uint32_t nverts = static_cast<uint32_t>(m.positions.size());
  std::vector<float> verts;
  verts.reserve(static_cast<size_t>(nverts) * 3);
  for (const QVector3D& p : m.positions) {
    verts.push_back(p.x());
    verts.push_back(p.y());
    verts.push_back(p.z());
  }
  uint32_t nodes = 0;
  std::vector<uint8_t> blob =
      vsim::buildWorldBvh(verts.data(), nverts, w.worldMeshDoubleSided, nodes);
  if (blob.empty()) {
    appendLog("world", tr("world mesh BVH build failed"));
    return;
  }

  // Publish atomically: write a temp file then rename() over the target so the
  // daemon never mmaps a half-written blob. Per-instance suffix keeps two
  // Navigators isolated (matches SimWorker's fifoSuffixed()).
  const QByteArray suffix = qgetenv("VSIM_FIFO_SUFFIX");
  const QString path = QStringLiteral("/tmp/vsim_world") +
                       QString::fromLocal8Bit(suffix) + QStringLiteral(".bin");
  const QString tmp = path + QStringLiteral(".tmp");
  {
    QFile f(tmp);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
      appendLog("world", tr("world mesh: cannot write %1").arg(tmp));
      return;
    }
    const qint64 wrote =
        f.write(reinterpret_cast<const char*>(blob.data()),
                static_cast<qint64>(blob.size()));
    f.close();
    if (wrote != static_cast<qint64>(blob.size())) {
      appendLog("world", tr("world mesh: short write to %1").arg(tmp));
      QFile::remove(tmp);
      return;
    }
  }
  if (::rename(tmp.toLocal8Bit().constData(), path.toLocal8Bit().constData()) != 0) {
    appendLog("world", tr("world mesh: rename to %1 failed").arg(path));
    QFile::remove(tmp);
    return;
  }

  const uint32_t ntris = nverts / 3;
  m_sim->sendWorldMesh(path, nverts, ntris, nodes, w.worldMeshRestitution,
                       w.worldMeshDoubleSided);
  appendLog("world",
            tr("world mesh → daemon: %1 tris, %2 nodes (%3 KiB)")
                .arg(ntris).arg(nodes).arg(blob.size() / 1024));
}

void SimulatorWidget::pushRatesToSim() {
  if (!m_sim) return;
  QSettings s;
  m_sim->sendRates(s.value(kImuHzKey, kDefImuHz).toInt(),
                   s.value(kPhysHzKey, kDefPhysHz).toInt(),
                   s.value(kPoseHzKey, kDefPoseHz).toInt());
}

void SimulatorWidget::hudSetStatus(const QString& s) {
  if (m_hud) m_hud->setStatus(s);
}

void SimulatorWidget::setRigControlsEnabled(bool simRunning) {
  // The rig poses the in-app sim (m_sim); with no sim running there's nothing
  // to pose. Disable + uncheck and say so, rather than silently no-op.
  if (m_rigEnable) {
    m_rigEnable->setEnabled(simRunning);
    if (!simRunning) {
      QSignalBlocker block(m_rigEnable);
      m_rigEnable->setChecked(false);
    }
  }
  const bool slidersOn = simRunning && m_rigEnable && m_rigEnable->isChecked();
  for (QSlider* s : {m_rigRoll, m_rigPitch, m_rigYaw})
    if (s) s->setEnabled(slidersOn);
  if (m_rigReadout)
    m_rigReadout->setText(simRunning ? tr("rig: off")
                                     : tr("rig: start the simulator to pose"));
}

void SimulatorWidget::applyRigPose() {
  if (!m_rigEnable || !m_rigEnable->isChecked()) return;
  const int r = m_rigRoll->value(), p = m_rigPitch->value(), y = m_rigYaw->value();
  // Always reflect the slider values so dragging is visibly registering, even
  // if the sim isn't connected — that tells us whether the issue is the sliders
  // or the channel.
  if (!m_sim) {
    if (m_rigReadout)
      m_rigReadout->setText(
          tr("rig: r%1 p%2 y%3 — sim not running, start it first").arg(r).arg(p).arg(y));
    return;
  }
  m_sim->sendRigPose(static_cast<float>(r), static_cast<float>(p),
                     static_cast<float>(y));
  if (m_rigReadout)
    m_rigReadout->setText(
        tr("rig: roll %1°  pitch %2°  yaw %3°  (sent)").arg(r).arg(p).arg(y));
}

void SimulatorWidget::setFlightModeStatus(quint8 mode, quint8 source) {
  const bool acro = (mode == kFmAcro);
  const bool gcs = (source == kFmSrcGcs);
  if (m_acroChk && m_acroChk->isChecked() != acro) {
    // Reflect firmware state without re-issuing the command (e.g. an RC switch
    // flip while no override is active).
    QSignalBlocker block(m_acroChk);
    m_acroChk->setChecked(acro);
  }
  const QString modeStr = QString("%1 (%2)")
                              .arg(acro ? tr("ACRO") : tr("STABILISE"),
                                   gcs ? tr("GCS") : tr("RC"));
  if (m_flightModeLabel) m_flightModeLabel->setText(tr("mode: ") + modeStr);
  if (m_hud) m_hud->setFlightMode(modeStr);   // show it in the viewport HUD too
}

void SimulatorWidget::hudSetImu(const float acc[3], const float gyr[3]) {
  if (m_hud) m_hud->setImu(acc, gyr);
}

void SimulatorWidget::pushRcMapping(int func) {
  if (func < 0 || func >= 5 || !m_rcAxisCombo[func]) return;
  const int axis = m_rcAxisCombo[func]->currentData().toInt();
  const bool inv = m_rcInvert[func]->isChecked();
  if (m_rc) m_rc->setMapping(func, axis, inv);
  QSettings s;
  s.setValue(QString(kRcMapAxisKey) + QString::number(func), axis);
  s.setValue(QString(kRcMapInvKey) + QString::number(func), inv);
}

void SimulatorWidget::applyRcSource() {
  if (!m_rcSource) return;
  const bool uart = m_rcSource->currentData().toInt() == RcBridge::Uart;
  QSettings st;
  st.setValue(kRcSourceKey, uart ? RcBridge::Uart : RcBridge::Joystick);

  // Swap the device field to the saved path for the selected source.
  if (m_rcPath) {
    const QString path =
        uart ? st.value(kRcUartPathKey, "/dev/ttyUSB0").toString()
             : st.value(kRcPathKey, "/dev/input/js0").toString();
    QSignalBlocker block(m_rcPath);
    m_rcPath->clear();
    if (uart) {
      // Enumerate serial ports so the user can pick instead of typing.
      for (const QSerialPortInfo& info : QSerialPortInfo::availablePorts()) {
        const QString desc = info.description();
        m_rcPath->addItem(desc.isEmpty()
                              ? info.portName()
                              : QStringLiteral("%1 — %2").arg(info.portName(),
                                                              desc),
                          info.systemLocation());
      }
    }
    m_rcPath->setEditText(path);
    m_rcPath->setToolTip(
        uart ? tr("Serial device streaming CSV RC frames "
                  "(roll,pitch,throttle,yaw,arm,…), µs per channel.")
             : tr("Joystick device (Linux js API)."));
  }
  if (m_rcBaud) m_rcBaud->setEnabled(uart);

  // The explicit Connect button only applies to the UART source. Switching to
  // the joystick (or away from UART) drops any held serial port.
  if (!uart && m_rcUartConnected) setRcUartConnected(false);
  if (m_rcConnect) m_rcConnect->setEnabled(uart);

  // Axis mapping only applies to a joystick; a CSV stream is already
  // channelised, so grey the mapping out under UART.
  for (int f = 0; f < 5; ++f) {
    if (m_rcAxisCombo[f]) m_rcAxisCombo[f]->setEnabled(!uart);
    if (m_rcInvert[f]) m_rcInvert[f]->setEnabled(!uart);
  }

  // Push to the bridge (no-op until it exists; the ctor calls this again).
  if (m_rc) {
    if (uart) {
      if (m_rcPath) m_rc->setUartPath(m_rcPath->currentText());
      if (m_rcBaud) m_rc->setUartBaud(m_rcBaud->currentData().toInt());
    } else if (m_rcPath) {
      m_rc->setJoystickPath(m_rcPath->currentText());
    }
    m_rc->setSource(uart ? RcBridge::Uart : RcBridge::Joystick);
  }
}

// Persist the current device-field text for the active source and push it to
// the bridge. Shared by the editable combo's pick + edit-finished signals.
void SimulatorWidget::commitRcPath() {
  if (!m_rcPath || !m_rcSource) return;
  const bool uart = m_rcSource->currentData().toInt() == RcBridge::Uart;
  const QString path = m_rcPath->currentText();
  QSettings().setValue(uart ? kRcUartPathKey : kRcPathKey, path);
  if (m_rc) {
    if (uart) m_rc->setUartPath(path);
    else      m_rc->setJoystickPath(path);
  }
}

void SimulatorWidget::setRcUartConnected(bool on) {
  m_rcUartConnected = on;
  const QString path = m_rcPath ? m_rcPath->currentText() : QString();
  if (m_rc) {
    if (on) {
      // Claim the port first — this revokes it from the board telemetry (or any
      // other holder), which disconnects in response, so only one owner ever
      // has the tty open.
      PortArbiter::instance().acquire(path, this);
      m_rc->setUartPath(path);
      m_rc->setUartConnected(true);
    } else {
      m_rc->setUartConnected(false);
      PortArbiter::instance().release(path, this);
    }
  }
  if (m_rcConnect) {
    QSignalBlocker b(m_rcConnect);
    m_rcConnect->setChecked(on);
    m_rcConnect->setText(on ? tr("Disconnect") : tr("Connect"));
  }
}

bool SimulatorWidget::eventFilter(QObject* obj, QEvent* ev) {
  if (obj == m_renderer && ev->type() == QEvent::Resize) {
    if (m_hud) m_hud->setGeometry(m_renderer->rect());
    if (m_horizon) {                       // re-pin to the top-right corner
      const int m = 12;
      m_horizon->move(m_renderer->width() - m_horizon->width() - m, m);
      m_horizon->raise();
    }
  }
  return QWidget::eventFilter(obj, ev);
}

// ----------------------------------------------------------------------------
// Sim lifecycle
// ----------------------------------------------------------------------------

void SimulatorWidget::startInAppSim() {
  if (m_sim) return;

  // Per-instance FIFO isolation (roadmap sim-integration #1): publish a
  // suffix for this Navigator process in the environment BEFORE vayu_sitl_start
  // (the firmware shim caches its FIFO/pty paths on first use) and before
  // SimWorker spawns vsim_d (which inherits it via environ). All three then
  // agree on /tmp/vsim_*<suffix>; without this the firmware can wait on a FIFO
  // nobody feeds -> frozen IMU, 0 Hz attitude. Respect an externally-set value.
  if (qEnvironmentVariableIsEmpty("VSIM_FIFO_SUFFIX")) {
    qputenv("VSIM_FIFO_SUFFIX",
            QByteArrayLiteral("_nav") + QByteArray::number(::getpid()));
  }

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
  // (Re)attach the firmware -> GCS telemetry callback so IMU / attitude /
  // heartbeat flow to the home-screen panels while running. It's detached on
  // Stop (below) — the in-process firmware threads can't actually be stopped,
  // so without this they'd keep streaming heartbeats and the LIVE blinker
  // would keep blinking after Stop.
  vsim_iface_set_uart2_callback(&m_iface, &uart2_to_widget_trampoline, this);

  m_sim = new vsim::SimWorker(this);
  connect(m_sim, &vsim::SimWorker::stoppedCleanly, this,
          &SimulatorWidget::onSimWorkerExited);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          m_renderer, &vsim::SimRendererWidget::setSnapshot);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          this, [this](vsim::SimSnapshot snap) {
            float pitch, yaw, roll;
            vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);
            // Fixed field widths so a leading '-' (or "-0.0") doesn't widen the
            // string and resize the label every frame — that caused the flicker.
            // Monospace + space-padding keeps each number a constant pixel width.
            m_simPoseLabel->setText(
                QString("pos=(%1, %2, %3) m   rpy=(%4, %5, %6) deg")
                    .arg(snap.pos_w.x(), 7, 'f', 2)
                    .arg(snap.pos_w.y(), 7, 'f', 2)
                    .arg(snap.pos_w.z(), 7, 'f', 2)
                    .arg(roll, 6, 'f', 1)
                    .arg(pitch, 6, 'f', 1)
                    .arg(yaw, 6, 'f', 1));
            updateHud(snap);
            m_propAudio.setMotors(snap.motor_omega);
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString& s) { appendLog("vsim", s); });
  // Once the daemon's FIFOs are up, push the configured airframe + world
  // so the sim flies the edited params from frame one.
  connect(m_sim, &vsim::SimWorker::online, this, [this] {
    if (!m_sim) return;
    const auto cfg = m_geomEditor->physicsConfig();
    pushRatesToSim();   // apply configured loop rates before geometry/world
    pushFirmwareMotorGeometry(cfg);   // firmware roll/pitch/yaw mix signs
    m_sim->sendGeometry(cfg);          // vsim_d physics motor layout
    m_sim->sendWorld(m_worldEditor->config());
    m_sim->sendObstacles(m_worldEditor->config().obstacles);
    loadWorldMeshToRenderer();  // re-loads + ships the collision BVH now m_sim exists
    appendLog("geom", tr("firmware roll-mix %1 (default -+ ; mismatch = inverted "
                          "roll). pushed to firmware + vsim_d.")
                          .arg(rollMixString(cfg)));
    // Belt-and-suspenders: re-assert BOTH the firmware mix AND the vsim_d layout
    // shortly after start. If a restart race left one side on its default while
    // the other had the real (possibly Y-mirrored) geometry, roll inverts and
    // the craft topples at lift-off; this guarantees they agree before arming.
    QTimer::singleShot(800, this, [this] {
      if (!m_sim) return;
      const auto c = m_geomEditor->physicsConfig();
      pushFirmwareMotorGeometry(c);
      m_sim->sendGeometry(c);
      pushRatesToSim();   // re-assert loop rates too: the daemon boots at its
                          // compiled default, so a dropped/raced SET_RATES would
                          // otherwise leave it there (e.g. physics stuck at 8k).
    });
  });
  m_sim->start(QThread::TimeCriticalPriority);

  m_simStartBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  if (m_simResetBtn) m_simResetBtn->setEnabled(true);
  if (m_fpvCheck) m_fpvCheck->setEnabled(true);  // FPV usable now we ride the drone
  m_simStatusLabel->setText(tr("● Running"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
  // Vehicle configuration is locked while simulating; force World mode.
  if (m_vehicleTab) m_vehicleTab->setEnabled(false);
  setMode(1);
  // Show the telemetry HUD over the viewport.
  if (m_hud) { m_hud->setGeometry(m_renderer->rect()); m_hud->raise(); m_hud->show(); }
  if (m_horizon) m_horizon->raise();   // keep the corner horizon above the HUD
  setRigControlsEnabled(true);         // rig pose is now available
  emit simRunningChanged(true);
}

void SimulatorWidget::stopInAppSim() {
  if (!m_sim) return;
  // Detach the telemetry callback first: the firmware threads keep running
  // (vayu_sitl is one-shot), so without this they'd keep streaming heartbeats
  // and IMU to the home screen after Stop — the LIVE blinker would never stop.
  // Dropping the callback makes the display go quiet, consistent with Stopped.
  vsim_iface_set_uart2_callback(&m_iface, nullptr, nullptr);
  m_sim->requestStop();
  if (!m_sim->wait(2000)) {
    m_sim->terminate();
    m_sim->wait(1000);
  }
  m_sim->deleteLater();
  m_sim = nullptr;
  m_propAudio.setMotors({0.0f, 0.0f, 0.0f, 0.0f});  // silence once stopped
  m_simStartBtn->setEnabled(true);
  m_simStopBtn->setEnabled(false);
  if (m_simResetBtn) m_simResetBtn->setEnabled(false);
  if (m_fpvCheck) {                       // FPV is drone-only; back to free-roam
    QSignalBlocker block(m_fpvCheck);
    m_fpvCheck->setChecked(false);
    m_fpvCheck->setEnabled(false);
  }
  if (m_renderer) m_renderer->setFpv(false);
  m_simStatusLabel->setText(tr("● Stopped (firmware idle)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  // Vehicle config is editable again; stay in World until the user
  // switches back (clicking the Vehicle tab re-enables motor gizmos).
  if (m_vehicleTab) m_vehicleTab->setEnabled(true);
  // Re-apply the current mode now that m_sim is null: re-enables World-mode
  // free-roam + obstacle gizmos (the running sim had locked the camera).
  if (m_rightStack) setMode(m_rightStack->currentIndex());
  if (m_hud) m_hud->hide();
  setRigControlsEnabled(false);        // no sim to pose anymore
  // Close the per-run log file so its trailing bytes flush to disk.
  closeLogFile();
  // Note: we don't call vayu_sitl_stop() here on the Stop button.
  // The firmware threads stay alive but receive no fresh IMU samples
  // (SimWorker isn't pushing). Restarting the sim resumes the pipe.
  emit simRunningChanged(false);
}

void SimulatorWidget::onSimWorkerExited() {
  // The worker emitted stoppedCleanly (spawn failure, startup-grace timeout,
  // or the daemon died) without the user clicking Stop. If we haven't already
  // torn down (explicit Stop nulls m_sim before the queued signal arrives),
  // run the normal stop path so the buttons / status reflect the dead worker.
  if (!m_sim) return;
  appendLog("vsim", "[worker exited — resetting to Stopped]");
  stopInAppSim();
}

// ----------------------------------------------------------------------------
// Geometry editor <-> renderer / daemon / persistence
// ----------------------------------------------------------------------------

void SimulatorWidget::applyGeometryToRenderer() {
  if (!m_renderer || !m_geomEditor) return;
  // CoM frame: mesh is already recentered, motor arms are CoM-relative,
  // and the CoM sits at the body origin.
  const auto cfg = m_geomEditor->physicsConfig();
  if (m_geomEditor->hasMesh()) {
    m_renderer->setDroneMesh(m_geomEditor->meshPositions(),
                             m_geomEditor->meshNormals());
  }
  m_renderer->setComMarker(QVector3D(0, 0, 0));  // CoM == body origin now
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
  s.setValue("tx", g.translate.x()); s.setValue("ty", g.translate.y()); s.setValue("tz", g.translate.z());
  s.setValue("rx", g.rotate.x());    s.setValue("ry", g.rotate.y());    s.setValue("rz", g.rotate.z());
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
  g.translate = QVector3D(s.value("tx").toFloat(), s.value("ty").toFloat(),
                          s.value("tz").toFloat());
  g.rotate = QVector3D(s.value("rx").toFloat(), s.value("ry").toFloat(),
                       s.value("rz").toFloat());
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

void SimulatorWidget::persistWorld(const vsim::WorldConfig& w) {
  QSettings s;
  s.beginGroup(kWorldGroup);
  s.setValue("gravity", w.gravity);
  s.setValue("ground_z", w.ground_z);
  s.setValue("restitution", w.restitution);
  s.setValue("linear_drag", w.linear_drag);
  s.setValue("angular_drag", w.angular_drag);
  s.setValue("ground_right_gain", w.ground_right_gain);
  s.setValue("ground_right_damp", w.ground_right_damp);
  s.setValue("worldMeshPath", w.worldMeshPath);
  s.setValue("worldScale", w.worldScale);
  s.setValue("worldUpAxis", w.worldUpAxis);
  s.setValue("worldMeshRestitution", w.worldMeshRestitution);
  s.setValue("worldMeshDoubleSided", w.worldMeshDoubleSided);
  s.setValue("worldMeshOffX", w.worldMeshOffset.x());
  s.setValue("worldMeshOffY", w.worldMeshOffset.y());
  s.setValue("worldMeshOffZ", w.worldMeshOffset.z());
  s.beginWriteArray("obstacles", w.obstacles.size());
  for (int i = 0; i < w.obstacles.size(); ++i) {
    const vsim::Obstacle& o = w.obstacles[i];
    s.setArrayIndex(i);
    s.setValue("type", o.type);
    s.setValue("px", o.pos.x()); s.setValue("py", o.pos.y()); s.setValue("pz", o.pos.z());
    s.setValue("sx", o.size.x()); s.setValue("sy", o.size.y()); s.setValue("sz", o.size.z());
    s.setValue("rx", o.rotate.x()); s.setValue("ry", o.rotate.y()); s.setValue("rz", o.rotate.z());
    s.setValue("rest", o.restitution);
  }
  s.endArray();
  s.endGroup();
}

vsim::WorldConfig SimulatorWidget::restoreWorld() {
  vsim::WorldConfig w;   // defaults if nothing saved
  QSettings s;
  s.beginGroup(kWorldGroup);
  if (!s.contains("gravity")) {
    s.endGroup();
    return w;
  }
  w.gravity      = s.value("gravity", w.gravity).toFloat();
  w.ground_z     = s.value("ground_z", w.ground_z).toFloat();
  w.restitution  = s.value("restitution", w.restitution).toFloat();
  w.linear_drag  = s.value("linear_drag", w.linear_drag).toFloat();
  w.angular_drag = s.value("angular_drag", w.angular_drag).toFloat();
  w.ground_right_gain = s.value("ground_right_gain", w.ground_right_gain).toFloat();
  w.ground_right_damp = s.value("ground_right_damp", w.ground_right_damp).toFloat();
  w.worldMeshPath = s.value("worldMeshPath", w.worldMeshPath).toString();
  w.worldScale = s.value("worldScale", w.worldScale).toFloat();
  w.worldUpAxis = s.value("worldUpAxis", w.worldUpAxis).toInt();
  w.worldMeshRestitution = s.value("worldMeshRestitution", w.worldMeshRestitution).toFloat();
  w.worldMeshDoubleSided = s.value("worldMeshDoubleSided", w.worldMeshDoubleSided).toBool();
  w.worldMeshOffset = QVector3D(
      s.value("worldMeshOffX", w.worldMeshOffset.x()).toFloat(),
      s.value("worldMeshOffY", w.worldMeshOffset.y()).toFloat(),
      s.value("worldMeshOffZ", w.worldMeshOffset.z()).toFloat());
  const int n = s.beginReadArray("obstacles");
  w.obstacles.clear();
  for (int i = 0; i < n; ++i) {
    s.setArrayIndex(i);
    vsim::Obstacle o;
    o.type = s.value("type", 0).toInt();
    o.pos = QVector3D(s.value("px").toFloat(), s.value("py").toFloat(), s.value("pz").toFloat());
    o.size = QVector3D(s.value("sx", 1.0).toFloat(), s.value("sy", 1.0).toFloat(), s.value("sz", 1.0).toFloat());
    o.rotate = QVector3D(s.value("rx").toFloat(), s.value("ry").toFloat(), s.value("rz").toFloat());
    o.restitution = s.value("rest", 0.3).toFloat();
    w.obstacles.push_back(o);
  }
  s.endArray();
  s.endGroup();
  return w;
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
