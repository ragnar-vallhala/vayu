#include "SimulatorWidget.h"

#include "CollapsibleSection.h"
#include "core/Theme.h"
#include "core/ui/Buttons.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QUrl>
#include <QByteArray>
#include <QVBoxLayout>

#include <cmath>

#include <unistd.h>

namespace {

constexpr const char* kRepoRootSettingKey = "simulator/repoRoot";
constexpr const char* kLogDirSettingKey   = "simulator/logDir";
constexpr const char* kGeomGroup          = "simulator/geometry";
constexpr const char* kWorldGroup         = "simulator/world";
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
            [this](int r, int pi, int t, int y, int a) {
              if (m_rcReadout)
                m_rcReadout->setText(QString("RC: R%1 P%2 T%3 Y%4 A%5")
                                         .arg(r).arg(pi).arg(t).arg(y).arg(a));
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
    auto* grp = new QButtonGroup(this);
    grp->setExclusive(true);
    for (auto* b : {m_vehicleTab, m_worldTab}) {
      b->setCheckable(true);
      b->setObjectName("ToggleButton");
      b->setCursor(Qt::PointingHandCursor);
    }
    m_vehicleTab->setToolTip(tr("Configure the airframe (locked while simulating)"));
    m_worldTab->setToolTip(tr("Design the world and run the simulation"));
    grp->addButton(m_vehicleTab, 0);
    grp->addButton(m_worldTab, 1);
    m_vehicleTab->setChecked(true);
    header->addWidget(m_vehicleTab);
    header->addWidget(m_worldTab);

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
      persistWorld(m_worldEditor->config());
    });
    // 3D gizmo editing of obstacles <-> the editor list/form.
    connect(m_renderer, &vsim::SimRendererWidget::obstacleEdited, m_worldEditor,
            &WorldEditorWidget::setObstacleFromGizmo);
    connect(m_renderer, &vsim::SimRendererWidget::obstacleSelected, m_worldEditor,
            &WorldEditorWidget::selectObstacleRow);
    connect(m_worldEditor, &WorldEditorWidget::obstacleSelectionChanged,
            m_renderer, &vsim::SimRendererWidget::selectObstacle);
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
    sv->addLayout(runRow);

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

      m_rcPath = new QLineEdit(simBody);  // text filled by applyRcSource()
      connect(m_rcPath, &QLineEdit::editingFinished, this, [this] {
        const bool uart = m_rcSource->currentData().toInt() == RcBridge::Uart;
        QSettings().setValue(uart ? kRcUartPathKey : kRcPathKey,
                             m_rcPath->text());
        if (m_rc) {
          if (uart) m_rc->setUartPath(m_rcPath->text());
          else      m_rc->setJoystickPath(m_rcPath->text());
        }
      });
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
      sv->addLayout(rcRow);

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

  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);

  // Restore persisted configs (geometry setConfig also loads the mesh +
  // recomputes, firing previewUpdated -> applyGeometryToRenderer).
  m_geomEditor->setConfig(restoreGeometry());
  m_worldEditor->setConfig(restoreWorld());
  if (m_renderer) m_renderer->setObstacles(m_worldEditor->config().obstacles);

  setMode(0);   // start in Vehicle (sim stopped)
}

void SimulatorWidget::setMode(int mode) {
  if (!m_rightStack) return;
  // Vehicle is config-only and locked while the sim runs.
  if (mode == 0 && m_sim) mode = 1;
  m_rightStack->setCurrentIndex(mode);
  if (m_vehicleTab) m_vehicleTab->setChecked(mode == 0);
  if (m_worldTab)   m_worldTab->setChecked(mode == 1);
  // Motor gizmos belong to Vehicle mode; obstacle gizmos to World mode. Both
  // only when stopped (a running sim owns the airframe + the live world).
  if (m_renderer) {
    m_renderer->setMotorsEditable(mode == 0 && !m_sim);
    m_renderer->setObstacleEditMode(mode == 1 && !m_sim);
  }
}

void SimulatorWidget::updateHud(const vsim::SimSnapshot& s) {
  if (m_hud) m_hud->setSnapshot(s);
}

void SimulatorWidget::hudSetStatus(const QString& s) {
  if (m_hud) m_hud->setStatus(s);
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
    m_rcPath->setText(path);
    m_rcPath->setToolTip(
        uart ? tr("Serial device streaming CSV RC frames "
                  "(roll,pitch,throttle,yaw,arm,…), µs per channel.")
             : tr("Joystick device (Linux js API)."));
  }
  if (m_rcBaud) m_rcBaud->setEnabled(uart);

  // Axis mapping only applies to a joystick; a CSV stream is already
  // channelised, so grey the mapping out under UART.
  for (int f = 0; f < 5; ++f) {
    if (m_rcAxisCombo[f]) m_rcAxisCombo[f]->setEnabled(!uart);
    if (m_rcInvert[f]) m_rcInvert[f]->setEnabled(!uart);
  }

  // Push to the bridge (no-op until it exists; the ctor calls this again).
  if (m_rc) {
    if (uart) {
      if (m_rcPath) m_rc->setUartPath(m_rcPath->text());
      if (m_rcBaud) m_rc->setUartBaud(m_rcBaud->currentData().toInt());
    } else if (m_rcPath) {
      m_rc->setJoystickPath(m_rcPath->text());
    }
    m_rc->setSource(uart ? RcBridge::Uart : RcBridge::Joystick);
  }
}

bool SimulatorWidget::eventFilter(QObject* obj, QEvent* ev) {
  if (obj == m_renderer && ev->type() == QEvent::Resize && m_hud) {
    m_hud->setGeometry(m_renderer->rect());
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
            snap.att.getEulerAngles(&pitch, &yaw, &roll);
            m_simPoseLabel->setText(
                QString("pos=(%1, %2, %3) m   rpy=(%4, %5, %6) deg")
                    .arg(snap.pos_w.x(), 0, 'f', 2)
                    .arg(snap.pos_w.y(), 0, 'f', 2)
                    .arg(snap.pos_w.z(), 0, 'f', 2)
                    .arg(roll, 0, 'f', 1)
                    .arg(pitch, 0, 'f', 1)
                    .arg(yaw, 0, 'f', 1));
            updateHud(snap);
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString& s) { appendLog("vsim", s); });
  // Once the daemon's FIFOs are up, push the configured airframe + world
  // so the sim flies the edited params from frame one.
  connect(m_sim, &vsim::SimWorker::online, this, [this] {
    if (!m_sim) return;
    m_sim->sendGeometry(m_geomEditor->physicsConfig());
    m_sim->sendWorld(m_worldEditor->config());
  });
  m_sim->start(QThread::TimeCriticalPriority);

  m_simStartBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  if (m_simResetBtn) m_simResetBtn->setEnabled(true);
  m_simStatusLabel->setText(tr("● Running"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
  // Vehicle configuration is locked while simulating; force World mode.
  if (m_vehicleTab) m_vehicleTab->setEnabled(false);
  setMode(1);
  // Show the telemetry HUD over the viewport.
  if (m_hud) { m_hud->setGeometry(m_renderer->rect()); m_hud->raise(); m_hud->show(); }
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
  m_simStartBtn->setEnabled(true);
  m_simStopBtn->setEnabled(false);
  if (m_simResetBtn) m_simResetBtn->setEnabled(false);
  m_simStatusLabel->setText(tr("● Stopped (firmware idle)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kTextMuted)));
  // Vehicle config is editable again; stay in World until the user
  // switches back (clicking the Vehicle tab re-enables motor gizmos).
  if (m_vehicleTab) m_vehicleTab->setEnabled(true);
  if (m_hud) m_hud->hide();
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
