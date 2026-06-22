#include "SimulatorWidget.h"

#include "core/Units.h"

#include "../../vsim/MeshLoader.h"
#include "../../vsim/ProceduralWorld.h"
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
#include <QFutureWatcher>
#include <QTimer>
#include <QtConcurrent>
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
#include <QDataStream>
#include <QFile>
#include <QVBoxLayout>
#include <QProcess>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QLabel>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>
#include <QTableWidget>
#include <QHeaderView>
#include <QPainter>
#include <QPainterPath>
#include <QFrame>
#include <QMouseEvent>

#include "AutotuneWorker.h"
#include "Space.h"

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

// Live step/chirp response plot: the roll-axis setpoint (dashed) vs measured
// (solid) angle from the latest autotune excitation window. No Q_OBJECT — it's
// driven by setData() from a lambda, not signals/slots.
class ResponsePlot : public QWidget {
 public:
  explicit ResponsePlot(QWidget* parent = nullptr) : QWidget(parent) {
    setMinimumHeight(110);
  }
  void setData(const QVector<double>& sp, const QVector<double>& meas) {
    sp_ = sp;
    meas_ = meas;
    update();
  }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = rect().adjusted(2, 2, -2, -2);
    p.fillRect(r, QColor(0x1A, 0x1D, 0x27));
    p.setPen(QPen(QColor(0x3E, 0x44, 0x52), 1));
    p.drawRect(r);
    if (sp_.isEmpty() && meas_.isEmpty()) {
      QFont f = p.font();
      f.setBold(true);
      f.setPixelSize(std::max(18, int(r.height() / 2)));
      p.setFont(f);
      p.setPen(QColor(0xE8, 0xF0, 0xFE, 40));  // faded watermark
      p.drawText(r, Qt::AlignCenter, QStringLiteral("NA"));
      return;
    }
    // Symmetric vertical scale around 0, fit to the data (deg).
    double mx = 1.0;
    for (double v : sp_) mx = std::max(mx, std::abs(v));
    for (double v : meas_) mx = std::max(mx, std::abs(v));
    mx *= 1.1;
    const int n = std::max(sp_.size(), meas_.size());
    auto toPt = [&](int i, double v) {
      const double x = r.left() + r.width() * (n > 1 ? double(i) / (n - 1) : 0.5);
      const double y = r.center().y() - (v / mx) * (r.height() / 2 - 4);
      return QPointF(x, y);
    };
    // Zero line.
    p.setPen(QPen(QColor(255, 255, 255, 24), 1));
    p.drawLine(QPointF(r.left(), r.center().y()), QPointF(r.right(), r.center().y()));
    auto poly = [&](const QVector<double>& d, QColor c, Qt::PenStyle st) {
      if (d.size() < 2) return;
      QPainterPath path;
      path.moveTo(toPt(0, d[0]));
      for (int i = 1; i < d.size(); ++i) path.lineTo(toPt(i, d[i]));
      p.setPen(QPen(c, 1.5, st));
      p.drawPath(path);
    };
    poly(sp_, QColor(0x8A, 0x92, 0xA6), Qt::DashLine);   // setpoint
    poly(meas_, QColor(0xE0, 0x6C, 0x75), Qt::SolidLine);  // measured (roll)
  }

 private:
  QVector<double> sp_, meas_;
};

// Draggable + resizable picture-in-picture frame floating over the FPV view
// (mockup .pip). A title strip drags the frame; the bottom-right 16 px corner
// resizes it. Both are clamped to the parent. Hosts any content widget.
class PipOverlay : public QFrame {
 public:
  PipOverlay(const QString& title, QWidget* content, QWidget* parent)
      : QFrame(parent), content_(content) {
    setObjectName("PipOverlay");
    setStyleSheet("#PipOverlay{background:#11141b;border:1px solid #3E4452;}");
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(1, 1, 1, 1);
    v->setSpacing(0);
    title_ = new QLabel(title, this);
    title_->setStyleSheet("background:#21252B; color:#8A92A6; font-size:9px; "
                          "font-weight:bold; letter-spacing:.5px; padding:2px 5px;");
    title_->setCursor(Qt::SizeAllCursor);
    title_->setFixedHeight(16);
    v->addWidget(title_);
    content_->setParent(this);
    v->addWidget(content_, 1);
    resize(220, 150);
    setMouseTracking(true);
  }

 protected:
  static constexpr int kGrip = 16;
  bool inGrip(const QPoint& p) const {
    return p.x() >= width() - kGrip && p.y() >= height() - kGrip;
  }
  void mousePressEvent(QMouseEvent* e) override {
    start_ = e->globalPosition().toPoint();
    startGeo_ = geometry();
    if (inGrip(e->pos())) mode_ = Resize;
    else if (e->pos().y() < title_->height()) mode_ = Move;
    else mode_ = None;
  }
  void mouseMoveEvent(QMouseEvent* e) override {
    setCursor(inGrip(e->pos()) ? Qt::SizeFDiagCursor : Qt::ArrowCursor);
    if (mode_ == None || !parentWidget()) return;
    const QPoint d = e->globalPosition().toPoint() - start_;
    if (mode_ == Move) {
      QPoint np = startGeo_.topLeft() + d;
      np.setX(qBound(0, np.x(), parentWidget()->width() - width()));
      np.setY(qBound(0, np.y(), parentWidget()->height() - height()));
      move(np);
    } else {  // Resize
      int w = qMax(120, startGeo_.width() + d.x());
      int h = qMax(90, startGeo_.height() + d.y());
      w = qMin(w, parentWidget()->width() - x());
      h = qMin(h, parentWidget()->height() - y());
      resize(w, h);
    }
  }
  void mouseReleaseEvent(QMouseEvent*) override { mode_ = None; }

 private:
  enum Mode { None, Move, Resize } mode_ = None;
  QLabel* title_ = nullptr;
  QWidget* content_ = nullptr;
  QPoint start_;
  QRect startGeo_;
};

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

  // Tear down the autotune machinery synchronously. Normally a tune is stopped
  // via stopAutotune() -> cancel() -> queued done -> onTuneDone(), but at app
  // exit the event loop is already gone, so that queued signal never arrives.
  // m_tuneThread and m_tuneSim are both QThread-derived children of this widget;
  // if left running they'd be "destroyed while still running" by ~QObject,
  // which qFatal()s (the abort seen on exit) AND orphans m_tuneSim's vsim_d
  // daemon (which keeps spinning, dragging the system). Join them here.
  if (m_tuneWorker) m_tuneWorker->cancel();  // break the blocking engine.run()
  if (m_tuneThread) {
    m_tuneThread->quit();
    if (!m_tuneThread->wait(3000)) {
      m_tuneThread->terminate();
      m_tuneThread->wait(1000);
    }
    delete m_tuneThread;       // joined: direct delete is safe (no event loop)
    m_tuneThread = nullptr;
  }
  if (m_tuneWorker) {
    delete m_tuneWorker;
    m_tuneWorker = nullptr;
  }
  detachTuneSim();             // joins m_tuneSim -> ~SimWorker kills its vsim_d

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

  // Artificial horizon, now hosted in a draggable/resizable PiP (mockup
  // .pip.adi). Fed from the sim snapshot in updateHud.
  m_horizon = new HorizonHud(m_renderer);
  m_horizonPip = new PipOverlay(tr("HORIZON"), m_horizon, m_renderer);
  m_horizonPip->resize(210, 150);
  m_horizonPip->move(8, 8);
  m_horizonPip->show();
  m_horizonPip->raise();

  // Down-cam PiP (mockup .pip.downcam): a second renderer looking straight down
  // at the drone. Fed the same pose + drone mesh as the main view.
  m_downRenderer = new vsim::SimRendererWidget();
  m_downRenderer->setDownCam(true);
  m_downRenderer->setWorldVisible(true);
  m_downPip = new PipOverlay(tr("DOWN CAM"), m_downRenderer, m_renderer);
  m_downPip->resize(210, 170);
  m_downPip->move(8, 166);
  m_downPip->show();
  m_downPip->raise();

  // Contour minimap PiP: top-down hypsometric + contour view of the procedural
  // terrain, centred on the drone (running) or the free-fly camera. Helps read
  // height, which the perspective view hides. Driven by m_minimapTimer.
  m_minimap = new ContourMinimapWidget(m_renderer);
  m_minimapPip = new PipOverlay(tr("CONTOURS"), m_minimap, m_renderer);
  m_minimapPip->resize(200, 200);
  m_minimapPip->move(8, 344);
  m_minimapPip->show();
  m_minimapPip->raise();
  m_minimapTimer = new QTimer(this);
  m_minimapTimer->setInterval(120);  // ~8 Hz: cheap, smooth enough to follow
  connect(m_minimapTimer, &QTimer::timeout, this, &SimulatorWidget::updateMinimap);
  m_minimapTimer->start();

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
    // Vehicle page = geometry editor + the Fault Injection panel (mockup
    // groups them in the vehicle config column).
    auto* vehWrap = new QWidget();
    auto* vehV = new QVBoxLayout(vehWrap);
    vehV->setContentsMargins(0, 0, 0, 0);
    vehV->setSpacing(6);
    vehV->addWidget(m_geomEditor);
    vehV->addWidget(buildSensorPanel());
    vehV->addWidget(buildFaultPanel());
    vehV->addStretch();

    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(vehWrap);
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
    // Wind & turbulence: its own staged Apply → push + persist.
    connect(m_worldEditor, &WorldEditorWidget::windApplied, this, [this] {
      const vsim::WindConfig w = m_worldEditor->windConfig();
      if (m_sim) m_sim->sendWind(w);
      persistWind(w);
      appendLog("world", w.enabled
                             ? tr("wind applied (steady %1/%2/%3 m/s, turb σ=%4)")
                                   .arg(w.steady.x()).arg(w.steady.y())
                                   .arg(w.steady.z()).arg(w.turbSigma)
                             : tr("wind disabled"));
    });
    // Obstacles are visual (phase 1): live-update the 3D view + persist on any
    // add/remove/edit, no Apply needed.
    connect(m_worldEditor, &WorldEditorWidget::obstaclesChanged, this, [this] {
      if (m_renderer) m_renderer->setObstacles(m_worldEditor->config().obstacles);
      if (m_downRenderer)
        m_downRenderer->setObstacles(m_worldEditor->config().obstacles);
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

    // ---- Training: glowing halo-gate course flown through the world ----
    {
      auto* sec = new CollapsibleSection(tr("Training"), page);
      auto* body = new QWidget();
      auto* col = new QVBoxLayout(body);
      col->setContentsMargins(0, 0, 0, 0);
      col->setSpacing(6);

      auto* row = new QHBoxLayout();
      row->addWidget(new QLabel(tr("Mode:"), body));
      m_trainingMode = new QComboBox(body);
      m_trainingMode->addItem(tr("Off"),    vsim::TrainingCourse::Off);
      m_trainingMode->addItem(tr("Easy"),   vsim::TrainingCourse::Easy);
      m_trainingMode->addItem(tr("Medium"), vsim::TrainingCourse::Medium);
      m_trainingMode->addItem(tr("Hard"),   vsim::TrainingCourse::Hard);
      m_trainingMode->setToolTip(tr(
          "Fly the drone through the glowing halo gates. The course starts at "
          "the floor and gets progressively harder (smaller, farther, more "
          "weave). A yellow arrow over the drone points to the next gate. "
          "Selecting a mode resets the airframe to the floor."));
      connect(m_trainingMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
              this, [this] {
                if (m_trainingMode)
                  setTrainingMode(m_trainingMode->currentData().toInt());
              });
      row->addWidget(m_trainingMode, 1);
      col->addLayout(row);

      m_trainingStatus = new QLabel(tr("Training off"), body);
      m_trainingStatus->setStyleSheet(
          QString("color:%1; font-size:11px;").arg(Theme::hex(Theme::kTextMuted)));
      col->addWidget(m_trainingStatus);

      sec->setContentWidget(body);
      pv->addWidget(sec);
    }

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
    connect(m_simResetBtn, &QPushButton::clicked, this, [this] {
      if (!m_sim) return;
      // On procedural terrain, respawn at the origin a clear margin ABOVE the
      // surface (heights are positive, so z=-0.05 would bury it under the hill).
      if (m_terrainHeightAt)
        m_sim->sendResetPose(0.0f, 0.0f, -m_terrainHeightAt(0.0f, 0.0f) - 1.5f);
      else
        m_sim->sendReset();
    });
    m_simAttachBtn = new ui::GhostButton(tr("Attach Ext"), simBody);
    m_simAttachBtn->setToolTip(tr("Render an EXTERNAL vsim_d's pose stream "
        "(/tmp/vsim_pose) over the loaded world — e.g. a headless sitl_lab.py "
        "run. No firmware/daemon is started here; this view just mirrors it."));
    connect(m_simAttachBtn, &QPushButton::clicked, this,
            &SimulatorWidget::attachExternalSim);
    runRow->addWidget(m_simStartBtn);
    runRow->addWidget(m_simStopBtn);
    runRow->addWidget(m_simResetBtn);
    runRow->addWidget(m_simAttachBtn);
    runRow->addStretch();
    m_fpvCheck = new QCheckBox(tr("FPV cam"), simBody);
    m_fpvCheck->setToolTip(tr("Onboard first-person camera that rides the drone "
                              "(looks forward). Off = 3rd-person orbit. "
                              "Available while the sim is running."));
    m_fpvCheck->setEnabled(false);   // enabled on Start (locks to the drone)
    connect(m_fpvCheck, &QCheckBox::toggled, this,
            [this](bool on) { if (m_renderer) m_renderer->setFpv(on); });
    runRow->addWidget(m_fpvCheck);

    // PiP visibility toggles (mockup Down Cam / Horizon chips).
    auto* downChk = new QCheckBox(tr("Down Cam"), simBody);
    downChk->setChecked(true);
    downChk->setToolTip(tr("Show the draggable bird's-eye down-camera PiP"));
    connect(downChk, &QCheckBox::toggled, this, [this](bool on) {
      if (m_downPip) m_downPip->setVisible(on);
    });
    runRow->addWidget(downChk);

    auto* horizonChk = new QCheckBox(tr("Horizon"), simBody);
    horizonChk->setChecked(true);
    horizonChk->setToolTip(tr("Show the draggable artificial-horizon PiP"));
    connect(horizonChk, &QCheckBox::toggled, this, [this](bool on) {
      if (m_horizonPip) m_horizonPip->setVisible(on);
    });
    runRow->addWidget(horizonChk);

    auto* contourChk = new QCheckBox(tr("Contours"), simBody);
    contourChk->setChecked(true);
    contourChk->setToolTip(tr("Show the draggable top-down contour minimap "
                              "(procedural terrain height)."));
    connect(contourChk, &QCheckBox::toggled, this, [this](bool on) {
      if (m_minimapPip) m_minimapPip->setVisible(on);
    });
    runRow->addWidget(contourChk);

    m_propAudioChk = new QCheckBox(tr("Prop audio"), simBody);
    m_propAudioChk->setToolTip(tr("Propeller sound synthesized from motor rpm "
                                  "(pitch + loudness rise with throttle)."));
    {
      QSettings st;
      m_propAudioChk->setChecked(st.value(kAudioKey, false).toBool());
      m_propAudio.setEnabled(m_propAudioChk->isChecked());
    }
    connect(m_propAudioChk, &QCheckBox::toggled, this, [this](bool on) {
      QSettings().setValue(kAudioKey, on);
      m_propAudio.setEnabled(on);
    });
    runRow->addWidget(m_propAudioChk);
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
  m_worldEditor->setWindConfig(restoreWind());
  if (m_renderer) m_renderer->setObstacles(m_worldEditor->config().obstacles);
  if (m_downRenderer)
    m_downRenderer->setObstacles(m_worldEditor->config().obstacles);
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

void SimulatorWidget::pushFaults() {
  if (!m_sim) return;
  m_sim->sendFaults({m_motorKill[0], m_motorKill[1], m_motorKill[2],
                     m_motorKill[3]},
                    m_imuDropout);
}

void SimulatorWidget::pushNoise() {
  if (!m_sim) return;
  auto& a = m_sensorRow[0];
  auto& g = m_sensorRow[1];
  auto& m = m_sensorRow[2];
  m_sim->sendNoise(a.sigma->value(), a.clip->value(), a.en->isChecked(),
                   g.sigma->value(), g.clip->value(), g.en->isChecked(),
                   m.sigma->value(), m.clip->value(), m.en->isChecked());
}

QWidget* SimulatorWidget::buildSensorPanel() {
  // Mockup Vehicle ▸ Sensor Models: per-sensor white-noise σ + bias clip, plus
  // an enable toggle that doubles as a dropout fault. Defaults match the
  // daemon's SensorNoise (BMX160-class).
  auto* grp = new QGroupBox(tr("Sensor Models"), this);
  auto* grid = new QGridLayout(grp);
  grid->setHorizontalSpacing(10);
  grid->setVerticalSpacing(4);
  grid->addWidget(new QLabel(tr("Sensor"), grp), 0, 0);
  grid->addWidget(new QLabel(tr("On"), grp), 0, 1);
  grid->addWidget(new QLabel(tr("Noise σ"), grp), 0, 2);
  grid->addWidget(new QLabel(tr("Bias clip"), grp), 0, 3);

  struct Def { const char* name; double sigma; double clip; int dec; double step; };
  const Def defs[3] = {
      {"Accel (m/s²)", 0.03, 0.08, 4, 0.005},
      {"Gyro (rad/s)", 0.0014, 0.012, 5, 0.0005},
      {"Mag (µT)", 0.3, 5.0, 3, 0.1},
  };
  for (int i = 0; i < 3; ++i) {
    auto& r = m_sensorRow[i];
    grid->addWidget(new QLabel(tr(defs[i].name), grp), i + 1, 0);
    r.en = new QCheckBox(grp);
    r.en->setChecked(true);
    r.en->setToolTip(tr("Off = drop this sensor's feed (dropout fault)"));
    grid->addWidget(r.en, i + 1, 1, Qt::AlignCenter);
    r.sigma = new QDoubleSpinBox(grp);
    r.sigma->setRange(0.0, 100.0);
    r.sigma->setDecimals(defs[i].dec);
    r.sigma->setSingleStep(defs[i].step);
    r.sigma->setValue(defs[i].sigma);
    grid->addWidget(r.sigma, i + 1, 2);
    r.clip = new QDoubleSpinBox(grp);
    r.clip->setRange(0.0, 100.0);
    r.clip->setDecimals(defs[i].dec);
    r.clip->setSingleStep(defs[i].step);
    r.clip->setValue(defs[i].clip);
    grid->addWidget(r.clip, i + 1, 3);

    connect(r.en, &QCheckBox::toggled, this, [this] { pushNoise(); });
    connect(r.sigma, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this] { pushNoise(); });
    connect(r.clip, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this] { pushNoise(); });
  }
  return grp;
}

QWidget* SimulatorWidget::buildFaultPanel() {
  // Mockup Vehicle ▸ Fault Injection: kill any rotor mid-flight, drop the RC
  // feed (firmware failsafe), or a GPS glitch (no GPS model in SITL yet).
  auto* grp = new QGroupBox(tr("Fault Injection"), this);
  auto* v = new QVBoxLayout(grp);
  v->setSpacing(6);

  auto* killRow = new QHBoxLayout();
  killRow->addWidget(new QLabel(tr("Kill motor:"), grp));
  for (int i = 0; i < 4; ++i) {
    m_killBtn[i] = new QPushButton(QString("M%1").arg(i + 1), grp);
    m_killBtn[i]->setCheckable(true);
    m_killBtn[i]->setFixedWidth(40);
    m_killBtn[i]->setToolTip(tr("Cut rotor %1 (dead ESC) while flying").arg(i + 1));
    connect(m_killBtn[i], &QPushButton::toggled, this, [this, i](bool on) {
      m_motorKill[i] = on;
      pushFaults();
      appendLog("fault", on ? tr("motor %1 killed").arg(i + 1)
                            : tr("motor %1 restored").arg(i + 1));
    });
    killRow->addWidget(m_killBtn[i]);
  }
  killRow->addStretch();
  v->addLayout(killRow);

  auto* rcLoss = new QCheckBox(tr("RC link loss (→ firmware failsafe)"), grp);
  rcLoss->setToolTip(tr("Stop feeding RC so the firmware enters failsafe"));
  connect(rcLoss, &QCheckBox::toggled, this, [this](bool on) {
    // On: cut the RC feed. Off: restore whatever the RC-enable box says.
    if (m_rc) m_rc->setEnabled(on ? false : (m_rcEnable && m_rcEnable->isChecked()));
    appendLog("fault", on ? tr("RC link loss injected") : tr("RC link restored"));
  });
  v->addWidget(rcLoss);

  auto* gps = new QCheckBox(tr("GPS glitch"), grp);
  gps->setEnabled(false);  // no GPS model in the SITL yet
  gps->setToolTip(tr("No GPS is simulated yet — placeholder for parity"));
  v->addWidget(gps);

  return grp;
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

  // Excitation waveform: step doublet vs swept-sine chirp (conditional freq row).
  form->addWidget(new QLabel(tr("Waveform:"), page), r, 0);
  m_tuneExcitation = new QComboBox(page);
  m_tuneExcitation->addItem(tr("Step (doublet)"));
  m_tuneExcitation->addItem(tr("Chirp (freq sweep)"));
  m_tuneExcitation->setToolTip(tr("Step = one doublet per axis; Chirp = swept "
                                  "sine f0→f1 over the hold, probing a band."));
  form->addWidget(m_tuneExcitation, r++, 1);

  auto* chirpLbl = new QLabel(tr("Chirp f0–f1 (Hz):"), page);
  form->addWidget(chirpLbl, r, 0);
  m_chirpRow = new QWidget(page);
  auto* chirpH = new QHBoxLayout(m_chirpRow);
  chirpH->setContentsMargins(0, 0, 0, 0);
  m_tuneChirpF0 = new QDoubleSpinBox(m_chirpRow);
  m_tuneChirpF0->setRange(0.1, 50.0);
  m_tuneChirpF0->setDecimals(1);
  m_tuneChirpF0->setValue(1.0);
  m_tuneChirpF1 = new QDoubleSpinBox(m_chirpRow);
  m_tuneChirpF1->setRange(0.1, 50.0);
  m_tuneChirpF1->setDecimals(1);
  m_tuneChirpF1->setValue(12.0);
  chirpH->addWidget(m_tuneChirpF0);
  chirpH->addWidget(new QLabel(QStringLiteral("→"), m_chirpRow));
  chirpH->addWidget(m_tuneChirpF1);
  form->addWidget(m_chirpRow, r++, 1);
  auto syncChirpVis = [this, chirpLbl] {
    const bool chirp = m_tuneExcitation->currentIndex() == 1;
    if (m_chirpRow) m_chirpRow->setVisible(chirp);
    chirpLbl->setVisible(chirp);
  };
  connect(m_tuneExcitation, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [syncChirpVis](int) { syncChirpVis(); });
  syncChirpVis();

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
  m_tuneFast = new QCheckBox(tr("Fast sim (~70× realtime)"), page);
  m_tuneFast->setToolTip(tr(
      "Run the search on the in-process real-vaios backend (vayu_sitl_rtos): a "
      "seeded arm+doublet, ~70× realtime and deterministic — a full sweep "
      "finishes in seconds. It uses the SAME cost as the realtime tuner (angle "
      "IAE + overshoot + chatter, with a large divergence penalty), so a "
      "tumbling tune scores high, never low. The current vehicle geometry is "
      "applied (physics + firmware mix).\n"
      "Scope: tunes rate kp/ki/kd, angle_kp and yaw_rate_kp — the gains this "
      "backend can set; gyro_lpf and the yaw ki/kd/lpf are left out of the "
      "search.\nThe World tab sim is unaffected and always runs realtime."));
  form->addWidget(m_tuneFast, r++, 1);

  m_tuneSysId = new QCheckBox(tr("System ID (analytic design)"), page);
  m_tuneSysId->setToolTip(tr(
      "Instead of an optimizer search, fly ONE broadband chirp on the fast "
      "backend, fit a physical plant model (omega/u = K/(s(tau·s+1))) per axis, "
      "and compute the gains analytically by loop-shaping (PD zero on the "
      "actuator pole; crossover at a fraction of the actuator bandwidth).\n"
      "Why: the search minimizes a tracking cost that the OUTER angle loop "
      "dominates, so it never values a stiff inner rate loop and settles on a "
      "near-zero rate_kp. System-ID has no cost to game — the rate loop gets a "
      "real gain by construction, in ONE rollout instead of a full sweep.\n"
      "Implies the fast backend. Budget/optimizer/cost-function are ignored; the "
      "chirp band + rig come from the Excitation settings. The result is applied "
      "and a verification doublet is flown to confirm it tracks."));
  form->addWidget(m_tuneSysId, r++, 1);

  form->addWidget(new QLabel(tr("Design bandwidth:"), page), r, 0);
  m_tuneSysIdBw = new QDoubleSpinBox(page);
  m_tuneSysIdBw->setRange(0.10, 0.50);
  m_tuneSysIdBw->setSingleStep(0.05);
  m_tuneSysIdBw->setDecimals(2);
  m_tuneSysIdBw->setValue(0.33);
  m_tuneSysIdBw->setToolTip(tr(
      "System-ID only: the rate-loop crossover as a fraction of the identified "
      "actuator bandwidth (1/tau). This is the aggressiveness lever — it sets "
      "rate_kp = wc/K and angle_kp = 0.25·wc.\n"
      "• Higher (→0.5): stiffer, faster disturbance rejection, but closer to the "
      "actuator pole — risks buzz; rate_kp is hard-capped at the buzz knee.\n"
      "• Lower (→0.1): gentler, more phase margin, softer response.\n"
      "0.33 (crossover at ~1/3 of the actuator BW) is a balanced default."));
  form->addWidget(m_tuneSysIdBw, r++, 1);

  form->addWidget(new QLabel(tr("Cost function:"), page), r, 0);
  m_tuneCostFn = new QComboBox(page);
  m_tuneCostFn->addItem(tr("Angle tracking (realtime cost)"));
  m_tuneCostFn->addItem(tr("Angle + rate tracking"));
  m_tuneCostFn->setToolTip(tr(
      "Cost used by the Fast backend.\n"
      "• Angle tracking: the exact realtime cost — angle-loop IAE + overshoot + "
      "chatter, with the divergence penalty. Scores the OUTER loop.\n"
      "• Angle + rate tracking: adds a roll/pitch RATE-loop tracking term so the "
      "search also values a tracking, non-buzzy inner loop (the analysis's "
      "Part-D rate term). Note: on the current sim plant the rate loop buzzes "
      "before it stiffens, so this mainly penalises buzzy high-rate_kp tunes "
      "rather than pushing rate_kp up.\n"
      "Applies to the Fast backend only; the realtime tuner always uses angle "
      "tracking."));
  form->addWidget(m_tuneCostFn, r++, 1);

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
  // AT-1: no auto-apply. The search only proposes; committing to firmware is a
  // separate explicit click (the Apply button below).
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
    m_tuneFast->setChecked(st.value(QStringLiteral("fast"), m_tuneFast->isChecked()).toBool());
    m_tuneSysId->setChecked(st.value(QStringLiteral("sysid"), m_tuneSysId->isChecked()).toBool());
    m_tuneSysIdBw->setValue(st.value(QStringLiteral("sysidBw"), m_tuneSysIdBw->value()).toDouble());
    m_tuneCostFn->setCurrentIndex(st.value(QStringLiteral("costFn"), m_tuneCostFn->currentIndex()).toInt());
    m_tuneCompare->setChecked(st.value(QStringLiteral("compare"), m_tuneCompare->isChecked()).toBool());
    m_tuneBuzz->setChecked(st.value(QStringLiteral("buzz"), m_tuneBuzz->isChecked()).toBool());
    m_tuneValidate->setChecked(st.value(QStringLiteral("validate"), m_tuneValidate->isChecked()).toBool());
    m_tunePlot->setChecked(st.value(QStringLiteral("plot"), m_tunePlot->isChecked()).toBool());
    m_tuneVerbose->setChecked(st.value(QStringLiteral("verbose"), m_tuneVerbose->isChecked()).toBool());
    st.endGroup();
  }
  // Design-bandwidth knob is meaningful only for the System-ID path.
  m_tuneSysIdBw->setEnabled(m_tuneSysId->isChecked());
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
  connect(m_tuneFast, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("fast"), v); });
  // System-ID implies the fast backend; auto-check Fast so the UI state is honest.
  connect(m_tuneSysId, &QCheckBox::toggled, this,
          [this, saveTune](bool v) {
            saveTune(QStringLiteral("sysid"), v);
            if (v && m_tuneFast)
              m_tuneFast->setChecked(true);
            if (m_tuneSysIdBw)
              m_tuneSysIdBw->setEnabled(v);
          });
  connect(m_tuneSysIdBw, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [saveTune](double v) { saveTune(QStringLiteral("sysidBw"), v); });
  connect(m_tuneCostFn, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [saveTune](int v) { saveTune(QStringLiteral("costFn"), v); });
  connect(m_tuneCompare, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("compare"), v); });
  connect(m_tuneBuzz, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("buzz"), v); });
  connect(m_tuneValidate, &QCheckBox::toggled, this,
          [saveTune](bool v) { saveTune(QStringLiteral("validate"), v); });
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

  // Live step/chirp response of the roll axis (setpoint vs measured), updated
  // once per evaluation from the latest excitation window.
  v->addWidget(new QLabel(tr("Roll response — setpoint (dashed) vs measured"), page));
  m_tuneResponse = new ResponsePlot(page);
  v->addWidget(m_tuneResponse);

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

  // AT-1: Proposed Gains (the search's best) + an explicit Apply to firmware.
  m_tuneProposed = new QLabel(tr("Proposed gains: —"), page);
  m_tuneProposed->setWordWrap(true);
  m_tuneProposed->setStyleSheet(
      QStringLiteral("font-family:monospace; font-size:11px;"));
  v->addWidget(m_tuneProposed);

  // AT-2: live current-vs-best gains. `current` jumps as the optimizer explores;
  // `best` improves monotonically and locks in (turns green) on convergence.
  m_tuneGainsTable = new QTableWidget(0, 3, page);
  m_tuneGainsTable->setHorizontalHeaderLabels(
      {tr("Gain"), tr("Current"), tr("Best")});
  m_tuneGainsTable->verticalHeader()->setVisible(false);
  m_tuneGainsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_tuneGainsTable->setSelectionMode(QAbstractItemView::NoSelection);
  m_tuneGainsTable->setFocusPolicy(Qt::NoFocus);
  m_tuneGainsTable->horizontalHeader()->setStretchLastSection(true);
  m_tuneGainsTable->setMaximumHeight(220);
  v->addWidget(m_tuneGainsTable);

  m_tuneApplyBtn = new QPushButton(tr("Apply Gains"), page);
  m_tuneApplyBtn->setEnabled(false);  // enabled once a search proposes gains
  m_tuneApplyBtn->setToolTip(
      tr("Apply the proposed (best) gains. With a flight controller connected "
         "they go to the board via CMD_SET_PID; otherwise they apply to the "
         "in-app sim and persist to 0:pid.bin (reloaded on the next sim start). "
         "The autotuner never writes them on its own."));
  connect(m_tuneApplyBtn, &QPushButton::clicked, this,
          [this] { applyProposedGains(); });
  v->addWidget(m_tuneApplyBtn);

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
  if (m_tuneThread || !m_geomEditor) return;
  const QString root = defaultRepoRoot();

  // Locate the SITL binaries the C++ stack spawns (no python3). Prefer the
  // tools/sim_host build dir; fall back to a repo-root build_sitl.
  AutotuneWorker::Params p;
  // Prefer the canonical in-tree build (tools/vsim/build); fall back to the
  // legacy repo-root build_vsim. A stale build_vsim/vsim_d at the wrong
  // VSIM_PROTO_VERSION desyncs the IMU feed (consumer rejects every frame).
  for (const QString &cand : {root + "/tools/vsim/build/vsim_d",
                              root + "/build_vsim/vsim_d"}) {
    if (QFileInfo::exists(cand)) {
      p.sitl.vsimBin = cand;
      break;
    }
  }
  for (const QString &cand : {root + "/tools/sim_host/build_sitl/vayu_sitl",
                              root + "/build_sitl/vayu_sitl"}) {
    if (QFileInfo::exists(cand)) {
      p.sitl.sitlBin = cand;
      break;
    }
  }

  // Fast backend: the single in-process binary replaces the vsim_d+vayu_sitl
  // pair. Locate it and validate the right binary set for the chosen engine.
  // System-ID is an analytic design path that also runs on the fast binary, so
  // it implies fastRtos (the optimizer/cost settings are ignored for it).
  p.sysId = m_tuneSysId && m_tuneSysId->isChecked();
  if (m_tuneSysIdBw)
    p.sysIdBwFrac = m_tuneSysIdBw->value();
  p.fastRtos = p.sysId || (m_tuneFast && m_tuneFast->isChecked());
  p.rtosRateCost = m_tuneCostFn && m_tuneCostFn->currentIndex() == 1;
  for (const QString &cand : {root + "/build_sitl_rtos/vayu_sitl_rtos",
                              root + "/tools/sim_host/build_sitl_rtos/vayu_sitl_rtos"}) {
    if (QFileInfo::exists(cand)) {
      p.rtosBin = cand;
      break;
    }
  }
  if (p.fastRtos) {
    if (p.rtosBin.isEmpty()) {
      m_tuneLog->appendPlainText(tr(
          "[error] fast backend not found. Build it with:\n"
          "  cmake -S tools/sim_host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON\n"
          "  cmake --build build_sitl_rtos --target vayu_sitl_rtos"));
      return;
    }
  } else if (!QFileInfo::exists(p.sitl.vsimBin) || p.sitl.sitlBin.isEmpty() ||
             !QFileInfo::exists(p.sitl.sitlBin)) {
    m_tuneLog->appendPlainText(
        tr("[error] SITL binaries not found (build vsim_d + vayu_sitl):\n  %1\n  %2")
            .arg(p.sitl.vsimBin, p.sitl.sitlBin));
    return;
  }

  // Stop the interactive sim (frees vsim_d/firmware) and lock Vehicle/World so
  // the airframe can't change mid-search.
  if (m_sim) stopInAppSim();
  if (m_vehicleTab) m_vehicleTab->setEnabled(false);
  if (m_worldTab) m_worldTab->setEnabled(false);

  const QString tuneSuffix =
      QStringLiteral("_attune%1").arg(QCoreApplication::applicationPid());
  p.sitl.suffix = tuneSuffix;

  // Tune the actual airframe + environment (mirrors the Python --geometry/
  // --world). Build the vsim_ctl bodies from the editors.
  {
    const vsim::GeometryConfig g = m_geomEditor->physicsConfig();
    vsim_ctl_geometry_t gb{};
    gb.mass = g.mass;
    for (int i = 0; i < 9; ++i) gb.inertia[i] = g.inertia[i];
    for (int i = 0; i < 4; ++i) {
      const auto &m = g.motors[i];
      gb.motors[i].pos[0] = m.pos.x();
      gb.motors[i].pos[1] = m.pos.y();
      gb.motors[i].pos[2] = m.pos.z();
      gb.motors[i].axis[0] = m.axis.x();
      gb.motors[i].axis[1] = m.axis.y();
      gb.motors[i].axis[2] = m.axis.z();
      gb.motors[i].spin = float(m.spin);
      gb.motors[i].k_thrust = m.k_thrust;
      gb.motors[i].k_moment = m.k_moment;
      gb.motors[i].max_omega = m.max_omega;
      gb.motors[i].tau = m.tau;
    }
    p.sitl.geometry = gb;
    p.sitl.hasGeometry = true;
    // Firmware mixer uses the SAME (physics-frame) layout as vsim: the roll/
    // pitch torque is generated in the rotated physics frame, so the mix signs
    // must match it. SitlStack derives the mix from geometry (no separate
    // mixer layout). The real fix for the rotated-vehicle spin was the §10.5
    // time-sync handshake (SitlStack::start) — without it set_motor_geometry
    // was rejected and the firmware ran its default mix, which is wrong here.

    const vsim::WorldConfig w = m_worldEditor->config();
    vsim_ctl_world_t wb{};
    wb.gravity = w.gravity;
    wb.ground_z = w.ground_z;
    wb.restitution = w.restitution;
    wb.linear_drag = w.linear_drag;
    wb.angular_drag = w.angular_drag;
    wb.ground_right_gain = w.ground_right_gain;
    wb.ground_right_damp = w.ground_right_damp;
    p.sitl.world = wb;
    p.sitl.hasWorld = true;

    m_tuneLog->clear();
    m_tuneLog->appendPlainText(
        tr("[geometry] mass=%1 kg, inertia diag=[%2, %3, %4] kg·m²")
            .arg(g.mass, 0, 'f', 3)
            .arg(g.inertia[0], 0, 'g', 4)
            .arg(g.inertia[4], 0, 'g', 4)
            .arg(g.inertia[8], 0, 'g', 4));
    m_tuneLog->appendPlainText(
        tr("[mix] firmware roll-mix=%1 (physics frame)").arg(rollMixString(g)));
  }

  p.tuneYaw = m_tuneYaw->isChecked();
  p.optimizer = m_tuneOptimizer->currentText();
  p.budget = m_tuneBudget->value();
  p.repeats = m_tuneRepeats->value();  // rollouts averaged per eval (smooths noise)
  p.optSeed = quint64(m_tuneSeed->value());
  p.rollout.stepUs = m_tuneStep->value();
  p.rollout.tetherK = m_tuneTether->value();
  p.rollout.seed = quint32(m_tuneSimSeed->value());
  p.rollout.excite = (m_tuneExcitation->currentIndex() == 1)
                         ? autotune::Excitation::Chirp
                         : autotune::Excitation::Step;
  p.rollout.chirpF0 = m_tuneChirpF0->value();
  p.rollout.chirpF1 = m_tuneChirpF1->value();

  m_tuneChart->reset();
  // Seed the current-vs-best table with this run's param rows (AT-2). The fast
  // backend tunes a restricted set, so the table must match it.
  {
    const autotune::Space space(p.tuneYaw, p.fastRtos);
    const auto names = space.names();
    m_tuneGainsTable->setRowCount(int(names.size()));
    for (int i = 0; i < int(names.size()); ++i) {
      m_tuneGainsTable->setItem(
          i, 0, new QTableWidgetItem(QString::fromStdString(names[i])));
      m_tuneGainsTable->setItem(i, 1, new QTableWidgetItem("—"));
      m_tuneGainsTable->setItem(i, 2, new QTableWidgetItem("—"));
    }
  }
  m_tuneResult->setText(tr("running…"));
  m_tuneStart->setEnabled(false);
  m_tuneStop->setEnabled(true);

  // Run the engine on a worker thread; its signals queue back to the GUI.
  m_tuneThread = new QThread(this);
  m_tuneWorker = new AutotuneWorker(p);
  m_tuneWorker->moveToThread(m_tuneThread);
  connect(m_tuneThread, &QThread::started, m_tuneWorker, &AutotuneWorker::run);
  connect(m_tuneWorker, &AutotuneWorker::evaluated, this,
          &SimulatorWidget::onTuneEvaluated);
  connect(m_tuneWorker, &AutotuneWorker::responseWindow, this,
          [this](const QVector<double> &sp, const QVector<double> &meas) {
            if (m_tuneResponse)
              static_cast<ResponsePlot *>(m_tuneResponse)->setData(sp, meas);
          });
  connect(m_tuneWorker, &AutotuneWorker::finished, this,
          &SimulatorWidget::onTuneFinished);
  connect(m_tuneWorker, &AutotuneWorker::failed, this, [this](const QString &e) {
    m_tuneLog->appendPlainText("[error] " + e);
  });
  connect(m_tuneWorker, &AutotuneWorker::log, this,
          [this](const QString &l) { m_tuneLog->appendPlainText(l); });
  connect(m_tuneWorker, &AutotuneWorker::done, this,
          &SimulatorWidget::onTuneDone);

  attachTuneSim(tuneSuffix);  // mirror the tuner's drone in the 3D view
  m_tuneThread->start();
  emit autotuneRunningChanged(true);  // drives the source FSM → Autotune state
}

void SimulatorWidget::stopAutotune() {
  if (m_tuneWorker) m_tuneWorker->cancel();  // unwinds the optimizer + stack
}

void SimulatorWidget::onTuneEvaluated(const QVector<double> &current,
                                      const QVector<double> &best, double cost,
                                      double bestCost, int n) {
  m_tuneChart->addPoint(cost, bestCost);
  m_tuneResult->setText(tr("eval %1   cost %2   best %3")
                            .arg(n)
                            .arg(cost, 0, 'f', 2)
                            .arg(bestCost, 0, 'f', 2));
  // AT-2: current jumps around as the search explores; best settles.
  const int rows = m_tuneGainsTable->rowCount();
  for (int i = 0; i < rows; ++i) {
    if (i < current.size())
      m_tuneGainsTable->item(i, 1)->setText(QString::number(current[i], 'g', 4));
    if (i < best.size())
      m_tuneGainsTable->item(i, 2)->setText(QString::number(best[i], 'g', 4));
  }
}

void SimulatorWidget::onTuneFinished(const QVector<double> &bestX,
                                     const QStringList &names, double bestCost) {
  // AT-1: surface the best gains as a proposal; applying is the explicit click.
  m_tuneParams = names;
  m_tuneBestX = bestX;
  QStringList parts;
  for (int i = 0; i < names.size() && i < bestX.size(); ++i)
    parts << QStringLiteral("%1=%2").arg(names[i]).arg(bestX[i], 0, 'g', 4);
  m_tuneProposed->setText(tr("Proposed gains (cost %1): %2")
                              .arg(bestCost, 0, 'f', 2)
                              .arg(parts.join(", ")));
  m_tuneApplyBtn->setEnabled(!autotuneGainsToCommands(names, bestX).isEmpty());

  // AT-2: best is final — lock the column in green and show it as current too.
  const QColor ok(Theme::hex(Theme::kOk));
  for (int i = 0; i < bestX.size() && i < m_tuneGainsTable->rowCount(); ++i) {
    const QString v = QString::number(bestX[i], 'g', 4);
    m_tuneGainsTable->item(i, 1)->setText(v);
    QTableWidgetItem *b = m_tuneGainsTable->item(i, 2);
    b->setText(v);
    b->setForeground(ok);
  }
}

void SimulatorWidget::onTuneDone() {
  if (m_tuneThread) {
    m_tuneThread->quit();
    m_tuneThread->wait(3000);
    m_tuneThread->deleteLater();
    m_tuneThread = nullptr;
  }
  // The worker's thread has fully stopped, so direct deletion is safe
  // (deleteLater would never run — that thread's event loop is gone).
  if (m_tuneWorker) {
    delete m_tuneWorker;
    m_tuneWorker = nullptr;
  }
  if (m_tuneStart) m_tuneStart->setEnabled(true);
  if (m_tuneStop) m_tuneStop->setEnabled(false);
  detachTuneSim();  // stop mirroring, unlock Vehicle/World
  emit autotuneRunningChanged(false);  // source FSM → Idle
}

void SimulatorWidget::parseProposedGains() {
  // AT-1: read the search result (a record on disk) and surface its best gains
  // as a *proposal*. Nothing is applied here — that's the explicit Apply click.
  m_tuneParams.clear();
  m_tuneBestX.clear();
  m_tuneProposed->setText(tr("Proposed gains: —"));
  m_tuneApplyBtn->setEnabled(false);

  QFile f(m_tuneOutJson);
  if (!f.open(QIODevice::ReadOnly)) return;
  const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
  f.close();
  if (!doc.isObject()) return;
  const QJsonObject root = doc.object();
  for (const QJsonValue& n : root.value(QStringLiteral("params")).toArray())
    m_tuneParams << n.toString();
  const QJsonArray results = root.value(QStringLiteral("results")).toArray();
  if (results.isEmpty()) return;
  // The autotuner sorts results best-first.
  for (const QJsonValue& x :
       results.first().toObject().value(QStringLiteral("best_x")).toArray())
    m_tuneBestX << x.toDouble();
  if (m_tuneParams.isEmpty() || m_tuneBestX.isEmpty()) return;

  QStringList parts;
  for (int i = 0; i < m_tuneParams.size() && i < m_tuneBestX.size(); ++i)
    parts << QStringLiteral("%1=%2").arg(m_tuneParams[i])
                 .arg(m_tuneBestX[i], 0, 'g', 4);
  m_tuneProposed->setText(tr("Proposed gains: %1").arg(parts.join(", ")));
  m_tuneApplyBtn->setEnabled(
      !autotuneGainsToCommands(m_tuneParams, m_tuneBestX).isEmpty());
}

void SimulatorWidget::applyProposedGains() {
  const QVector<PidSetCmd> cmds =
      autotuneGainsToCommands(m_tuneParams, m_tuneBestX);
  if (cmds.isEmpty()) {
    m_tuneLog->appendPlainText(tr("[apply] no proposed gains to apply"));
    return;
  }
  // MainWindow turns these into CMD_SET_PID frames and sends them over the
  // live link (and refuses if disconnected or in replay).
  emit applyPidGainsRequested(cmds);
  m_tuneLog->appendPlainText(
      tr("[apply] requested firmware gain update — %1 PID slot(s)")
          .arg(cmds.size()));
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
  if (m_horizonPip) m_horizonPip->raise();
  if (m_downPip) m_downPip->raise();
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
  m_lastDronePos = s.pos_w;  // tracked for the lift-onto-terrain logic
  if (m_hud) m_hud->setSnapshot(s);
  if (m_horizon) {
    float roll, pitch, yaw;
    vsim::quatToEulerNED(s.att, &roll, &pitch, &yaw);
    m_horizon->setAttitude(roll, pitch, yaw);
  }
  // Live wind readout (horizontal speed + compass heading the wind blows TOWARD).
  if (m_worldEditor) {
    const float wN = s.wind_w.x(), wE = s.wind_w.y();
    const float spd = std::sqrt(wN * wN + wE * wE);
    float dir = std::atan2(wE, wN) * 180.0f / 3.14159265358979323846f;
    if (dir < 0.0f) dir += 360.0f;
    m_worldEditor->setWindReadout(spd, dir);
  }

  // Training course: feed the live pose, advance through gates, refresh the
  // active-gate highlight + guidance arrow in both renderers.
  if (m_training.active()) {
    const bool cleared = m_training.advance(s.pos_w);
    const bool show = !m_training.finished();
    if (m_renderer) m_renderer->setTrainingActive(m_training.activeIndex(), show);
    if (m_downRenderer)
      m_downRenderer->setTrainingActive(m_training.activeIndex(), show);
    if (cleared) {
      updateTrainingProgress();
      if (m_training.finished())
        appendLog("train", tr("course complete — %1 gates cleared!")
                               .arg(m_training.total()));
      else
        appendLog("train", tr("gate %1 / %2 cleared")
                               .arg(m_training.passedCount())
                               .arg(m_training.total()));
    }
  }
}

void SimulatorWidget::setTrainingMode(int difficulty) {
  m_training.generate(static_cast<vsim::TrainingCourse::Difficulty>(difficulty));
  pushTrainingGates();
  updateTrainingProgress();
  // Start every run from the floor: re-spawn the airframe at the level pose.
  if (m_training.active() && m_sim) m_sim->sendReset();
  if (m_training.active())
    appendLog("train", tr("training course armed (%1 gates)")
                           .arg(m_training.total()));
}

void SimulatorWidget::pushTrainingGates() {
  const bool show = m_training.active() && !m_training.finished();
  if (m_renderer) {
    m_renderer->setTrainingGates(m_training.gates());
    m_renderer->setTrainingActive(m_training.activeIndex(), show);
  }
  if (m_downRenderer) {
    m_downRenderer->setTrainingGates(m_training.gates());
    m_downRenderer->setTrainingActive(m_training.activeIndex(), show);
  }
}

void SimulatorWidget::updateTrainingProgress() {
  if (!m_trainingStatus) return;
  if (!m_training.active()) {
    m_trainingStatus->setText(tr("Training off"));
  } else if (m_training.finished()) {
    m_trainingStatus->setText(tr("Course complete — %1 / %1 gates")
                                  .arg(m_training.total()));
  } else {
    m_trainingStatus->setText(tr("Next: gate %1 / %2")
                                  .arg(m_training.passedCount() + 1)
                                  .arg(m_training.total()));
  }
}

void SimulatorWidget::loadWorldMeshToRenderer() {
  if (!m_renderer || !m_worldEditor) return;
  const vsim::WorldConfig& w = m_worldEditor->config();

  const bool endless =
      w.proceduralBiome.compare(QStringLiteral("endless"), Qt::CaseInsensitive) == 0;

  // Point the contour minimap at the active terrain's height source. The
  // endless lambda reads the streamer's field lazily (configured below); the
  // finite lambda owns its heightfield via the captured shared_ptr.
  // One terrain-height sampler (world N,E -> elevation [m]) for the active world,
  // shared by the minimap and the "lift drone onto the surface" logic.
  std::function<float(float, float)> heightAt;
  float minimapRange = 200.0f;
  if (endless) {
    heightAt = [this](float n, float e) {
      const auto f = m_chunkStreamer.field();
      return f ? f->height(n, e) : 0.0f;
    };
  } else if (vsim::isKnownBiome(w.proceduralBiome)) {
    auto hf = std::make_shared<vsim::procgen::Heightfield>(
        vsim::proceduralMeadowHeightfield(w));
    heightAt = [hf](float n, float e) { return hf->sampleWorld(n, e); };
    minimapRange = w.proceduralSizeM * 0.5f;  // fit the finite arena
  }
  m_terrainHeightAt = heightAt;  // null for imported / no world
  if (m_minimap) {
    m_minimap->setSampler(heightAt);
    if (heightAt) m_minimap->setRangeM(minimapRange);
  }

  // Leaving the endless biome: stop the streamer and drop its chunks so a
  // finite mesh / grid shows cleanly.
  if (!endless && m_chunkStreamer.active()) {
    ++m_streamGen;  // invalidate in-flight builds
    m_chunkStreamer.deactivate();
    m_chunkCache.clear();
    m_collisionPending = false;
    if (m_streamTimer) m_streamTimer->stop();
    m_renderer->clearWorldChunks();
    if (m_downRenderer) m_downRenderer->clearWorldChunks();
  }

  // Endless streaming biome: drive the chunk streamer instead of one mesh.
  if (endless) {
    m_renderer->setWorldMesh({}, {});
    if (m_downRenderer) m_downRenderer->setWorldMesh({}, {});
    m_renderer->clearWorldChunks();
    if (m_downRenderer) m_downRenderer->clearWorldChunks();

    // New generation: invalidate any in-flight builds from a prior config and
    // drop the mesh cache.
    ++m_streamGen;
    m_chunkCache.clear();
    m_collisionPending = false;

    vsim::ChunkStreamer::Config sc;
    sc.field.seed = static_cast<uint32_t>(w.proceduralSeed);
    m_chunkStreamer.configure(sc);

    if (!m_streamTimer) {
      m_streamTimer = new QTimer(this);
      m_streamTimer->setInterval(100);  // 10 Hz poll of the view centre
      connect(m_streamTimer, &QTimer::timeout, this,
              &SimulatorWidget::onStreamTick);
    }
    m_streamTimer->start();
    onStreamTick();  // stream the initial neighbourhood immediately
    appendLog("world", tr("endless procedural terrain (seed %1) streaming")
                           .arg(w.proceduralSeed));
    return;
  }

  // Procedural world takes precedence over an imported mesh: generate it from
  // the biome params and feed the same render + collision pipeline.
  if (!w.proceduralBiome.isEmpty()) {
    const vsim::LoadedMesh m = vsim::generateProceduralWorld(w);
    if (!m.valid) {
      appendLog("world", tr("procedural world: unknown biome '%1'")
                             .arg(w.proceduralBiome));
      m_renderer->setWorldMesh({}, {});
      if (m_downRenderer) m_downRenderer->setWorldMesh({}, {});
      if (m_sim) m_sim->clearWorldMesh();
      return;
    }
    m_renderer->setWorldMesh(m.positions, m.normals, m.colors);
    if (m_downRenderer)
      m_downRenderer->setWorldMesh(m.positions, m.normals, m.colors);
    appendLog("world", tr("procedural world '%1' (seed %2): %3 tris")
                           .arg(w.proceduralBiome)
                           .arg(w.proceduralSeed)
                           .arg(m.triangleCount()));
    if (m_sim) {
      sendWorldMeshToSim(m);     // collision ready -> safe to lift onto it
      liftDroneToSurface();
    }
    return;
  }

  if (w.worldMeshPath.isEmpty()) {
    m_renderer->setWorldMesh({}, {});
    if (m_downRenderer) m_downRenderer->setWorldMesh({}, {});
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
    if (m_downRenderer) m_downRenderer->setWorldMesh({}, {});
    if (m_sim) m_sim->clearWorldMesh();
    return;
  }
  m_renderer->setWorldMesh(m.positions, m.normals, m.colors);
  if (m_downRenderer)
    m_downRenderer->setWorldMesh(m.positions, m.normals, m.colors);
  appendLog("world", tr("world mesh loaded: %1 tris").arg(m.triangleCount()));

  // Hand the same baked geometry to the physics daemon as a collision BVH.
  // Render and collision therefore share one transform → they never disagree.
  if (m_sim) sendWorldMeshToSim(m);
}

void SimulatorWidget::updateMinimap() {
  if (!m_minimap || !m_minimapPip || !m_minimapPip->isVisible()) return;
  if (!m_renderer || !m_minimap->hasSampler()) return;
  const QVector3D c = m_renderer->streamCenter();
  m_minimap->setView(c.x(), c.y(), m_renderer->viewHeadingRad());
}

void SimulatorWidget::onStreamTick() {
  if (!m_chunkStreamer.active() || !m_renderer) return;
  const QVector3D c = m_renderer->streamCenter();
  const vsim::StreamPlan p = m_chunkStreamer.plan(c.x(), c.y());

  for (qint64 key : p.toRemove) {
    m_renderer->removeWorldChunk(key);
    if (m_downRenderer) m_downRenderer->removeWorldChunk(key);
    m_chunkCache.erase(key);
    m_chunkStreamer.forget(key);
  }

  if (p.collisionDue) {
    m_collisionPending = true;
    m_colCx = p.colCx;
    m_colCy = p.colCy;
  }

  // Mesh the requested chunks OFF the UI thread; apply the results on the main
  // thread when each future finishes. The field is const + shared, so parallel
  // sampling is safe; the generation tag drops results from a stale config.
  const auto field = m_chunkStreamer.field();
  const float chunkM = m_chunkStreamer.config().chunkM;
  const int res = m_chunkStreamer.config().resolution;
  const int gen = m_streamGen;
  for (const vsim::ChunkReq& req : p.toBuild) {
    auto* watcher = new QFutureWatcher<vsim::procgen::ProcMesh>(this);
    connect(watcher, &QFutureWatcherBase::finished, this, [this, watcher, req, gen]() {
      if (gen == m_streamGen) onChunkMeshed(req.key, watcher->result());
      else m_chunkStreamer.forget(req.key);  // stale: config changed mid-build
      watcher->deleteLater();
    });
    watcher->setFuture(QtConcurrent::run([field, req, chunkM, res]() {
      return vsim::procgen::meshFieldChunk(*field, req.cx, req.cy, chunkM, res);
    }));
  }

  tryBuildCollision();  // in case the neighbourhood is already cached
}

void SimulatorWidget::onChunkMeshed(qint64 key,
                                    const vsim::procgen::ProcMesh& mesh) {
  m_chunkStreamer.markBuilt(key);
  if (!m_chunkStreamer.wanted(key)) {  // drifted out of range while meshing
    m_chunkStreamer.forget(key);
    return;
  }
  m_chunkCache[key] = mesh;  // retained so collision can reuse it (no regen)
  const vsim::ChunkMeshData d = vsim::toChunkMeshData(key, m_chunkCache[key]);
  m_renderer->setWorldChunk(d.key, d.positions, d.normals, d.colors);
  if (m_downRenderer)
    m_downRenderer->setWorldChunk(d.key, d.positions, d.normals, d.colors);
  tryBuildCollision();
}

void SimulatorWidget::tryBuildCollision() {
  if (!m_collisionPending) return;
  if (!m_sim) { m_collisionPending = false; return; }  // no physics -> not needed
  // Build the local collision BVH from the cached chunk meshes (no regen). Wait
  // until the whole collision neighbourhood is cached.
  const std::vector<vsim::ChunkReq> need =
      m_chunkStreamer.collisionChunks(m_colCx, m_colCy);
  std::vector<const vsim::procgen::ProcMesh*> meshes;
  meshes.reserve(need.size());
  for (const vsim::ChunkReq& req : need) {
    auto it = m_chunkCache.find(req.key);
    if (it == m_chunkCache.end()) return;  // not all cached yet; retry on next
    meshes.push_back(&it->second);
  }
  const vsim::LoadedMesh m = vsim::collisionMeshFromChunks(meshes);
  m_collisionPending = false;
  if (m.valid) sendWorldMeshToSim(m);
  // The local terrain just changed under the drone (spawn or a crossing); if
  // that left it buried, re-drop it above the new surface. Self-guarded, so an
  // airborne or resting drone is untouched.
  liftDroneToSurface();
}

void SimulatorWidget::liftDroneToSurface() {
  if (!m_sim || !m_terrainHeightAt) return;
  const float x = m_lastDronePos.x();
  const float y = m_lastDronePos.y();
  // Terrain is a single-valued height field, so the analytic height under the
  // drone is exactly where a ray cast straight down from far above would hit.
  const float h = m_terrainHeightAt(x, y);     // surface elevation [m] above z=0
  const float surfaceZ = -h;                   // NED z of the surface (above z=0)

  // Only re-drop when the drone is genuinely buried — more than buriedEps below
  // the surface. In NED a larger z is lower, so droneZ > surfaceZ + eps means it
  // has sunk into the terrain. A drone resting on or flying above the surface is
  // left alone (no yanking a landed or airborne drone).
  const float buriedEps = 0.3f;
  if (m_lastDronePos.z() <= surfaceZ + buriedEps) return;

  // Re-drop level + stationary a clear margin above the surface, so it falls and
  // settles instead of spawning embedded in a slope (which wedges it tilted).
  const float dropClearance = 1.5f;
  m_sim->sendResetPose(x, y, surfaceZ - dropClearance);
  appendLog("world", tr("re-dropped drone above terrain (surface %.1f m)").arg(h));
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

void SimulatorWidget::setPropAudioDefault(bool on) {
  // Drive the checkbox (its toggled handler enables PropAudio + persists the
  // per-sim key); no-op if it already matches so we don't thrash the device.
  if (m_propAudioChk && m_propAudioChk->isChecked() != on)
    m_propAudioChk->setChecked(on);
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
    // The PiPs float at user-chosen positions; just keep them above the HUD and
    // clamp them back inside if the viewport shrank past them.
    for (QWidget* pip : {m_horizonPip, m_downPip, m_minimapPip}) {
      if (!pip) continue;
      QPoint p = pip->pos();
      p.setX(qBound(0, p.x(), qMax(0, m_renderer->width() - pip->width())));
      p.setY(qBound(0, p.y(), qMax(0, m_renderer->height() - pip->height())));
      pip->move(p);
      pip->raise();
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
  if (m_downRenderer)
    connect(m_sim, &vsim::SimWorker::poseUpdated,
            m_downRenderer, &vsim::SimRendererWidget::setSnapshot);
  // Record physics ground truth for this run (gt-*.bin).
  connect(m_sim, &vsim::SimWorker::poseUpdated, this,
          &SimulatorWidget::logGroundTruth);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          this, [this](vsim::SimSnapshot snap) {
            float pitch, yaw, roll;
            vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);
            // Fixed field widths so a leading '-' (or "-0.0") doesn't widen the
            // string and resize the label every frame — that caused the flicker.
            // Monospace + space-padding keeps each number a constant pixel width.
            // Position is a length (NED metres) → convert via the altitude/
            // length unit; rpy via the angle unit. Decimals from Settings ▸
            // Units & Display; field widths stay fixed to avoid label reflow.
            const int dec = Units::decimals();
            m_simPoseLabel->setText(
                QString("pos=(%1, %2, %3) %7   rpy=(%4, %5, %6)%8")
                    .arg(Units::toAltitude(snap.pos_w.x()), 7, 'f', dec)
                    .arg(Units::toAltitude(snap.pos_w.y()), 7, 'f', dec)
                    .arg(Units::toAltitude(snap.pos_w.z()), 7, 'f', dec)
                    .arg(Units::toAngle(roll), 6, 'f', dec)
                    .arg(Units::toAngle(pitch), 6, 'f', dec)
                    .arg(Units::toAngle(yaw), 6, 'f', dec)
                    .arg(Units::altSuffix(), Units::angleSuffix()));
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
    pushFirmwareMotorGeometry(cfg);   // firmware roll/pitch/yaw mix signs (physics frame)
    m_sim->sendGeometry(cfg);          // vsim_d physics motor layout
    m_sim->sendWorld(m_worldEditor->config());
    m_sim->sendObstacles(m_worldEditor->config().obstacles);
    m_sim->sendWind(m_worldEditor->windConfig());   // restore wind across restart
    pushNoise();   // apply the configured sensor models (σ / enable)
    pushFaults();  // re-assert any latched faults across the restart
    loadWorldMeshToRenderer();  // re-loads + ships the collision BVH now m_sim exists
    // Restart any armed training course from the first gate, flying from the floor.
    if (m_training.active()) {
      m_training.resetProgress();
      pushTrainingGates();
      updateTrainingProgress();
    }
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
  if (m_horizonPip) m_horizonPip->raise();   // keep the PiPs above the HUD
  if (m_downPip) m_downPip->raise();
  setRigControlsEnabled(true);         // rig pose is now available
  emit simRunningChanged(true);
}

void SimulatorWidget::attachExternalSim() {
  if (m_sim) return;
  // Mirror an EXTERNAL vsim_d (default /tmp/vsim_pose) — e.g. a headless
  // sitl_lab.py run driving the real firmware. We spawn no daemon and boot no
  // in-process firmware; this view only renders the external pose stream over
  // the already-loaded vehicle + world meshes. Telemetry (dashboards) still
  // comes via the normal serial connection (the harness's --gcs bridge).
  m_attached = true;
  m_sim = new vsim::SimWorker(this);
  connect(m_sim, &vsim::SimWorker::stoppedCleanly, this,
          &SimulatorWidget::onSimWorkerExited);
  connect(m_sim, &vsim::SimWorker::poseUpdated,
          m_renderer, &vsim::SimRendererWidget::setSnapshot);
  if (m_downRenderer)
    connect(m_sim, &vsim::SimWorker::poseUpdated,
            m_downRenderer, &vsim::SimRendererWidget::setSnapshot);
  connect(m_sim, &vsim::SimWorker::poseUpdated, this,
          [this](vsim::SimSnapshot snap) {
            const int dec = Units::decimals();
            float roll, pitch, yaw;
            vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);
            if (m_simPoseLabel)
              m_simPoseLabel->setText(
                  QString("pos=(%1, %2, %3) %7   rpy=(%4, %5, %6)%8")
                      .arg(Units::toAltitude(snap.pos_w.x()), 7, 'f', dec)
                      .arg(Units::toAltitude(snap.pos_w.y()), 7, 'f', dec)
                      .arg(Units::toAltitude(snap.pos_w.z()), 7, 'f', dec)
                      .arg(Units::toAngle(roll), 6, 'f', dec)
                      .arg(Units::toAngle(pitch), 6, 'f', dec)
                      .arg(Units::toAngle(yaw), 6, 'f', dec)
                      .arg(Units::altSuffix(), Units::angleSuffix()));
            updateHud(snap);
            m_propAudio.setMotors(snap.motor_omega);
          });
  connect(m_sim, &vsim::SimWorker::logLine, this,
          [this](const QString& s) { appendLog("attach", s); });
  m_sim->startAttach(QStringLiteral("/tmp/vsim_pose"));

  m_simStartBtn->setEnabled(false);
  m_simAttachBtn->setEnabled(false);
  m_simStopBtn->setEnabled(true);
  if (m_fpvCheck) m_fpvCheck->setEnabled(true);
  m_simStatusLabel->setText(tr("● Attached (external sim)"));
  m_simStatusLabel->setStyleSheet(
      QString("color: %1;").arg(Theme::hex(Theme::kOk)));
  if (m_vehicleTab) m_vehicleTab->setEnabled(false);
  setMode(1);
  if (m_hud) { m_hud->setGeometry(m_renderer->rect()); m_hud->raise(); m_hud->show(); }
  if (m_horizonPip) m_horizonPip->raise();
  if (m_downPip) m_downPip->raise();
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
  if (m_simAttachBtn) m_simAttachBtn->setEnabled(true);
  m_attached = false;
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

  // Mirror the airframe into the down-cam PiP so its bird's-eye shows the drone.
  if (m_downRenderer) {
    if (m_geomEditor->hasMesh())
      m_downRenderer->setDroneMesh(m_geomEditor->meshPositions(),
                                   m_geomEditor->meshNormals());
    m_downRenderer->setMotorLayout(pos, axis, spin);
  }
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

void SimulatorWidget::persistWind(const vsim::WindConfig& w) {
  QSettings s;
  s.beginGroup(kWorldGroup);
  s.setValue("windN", w.steady.x());
  s.setValue("windE", w.steady.y());
  s.setValue("windD", w.steady.z());
  s.setValue("windGustAmp", w.gustAmp);
  s.setValue("windGustPeriod", w.gustPeriod);
  s.setValue("windTurbSigma", w.turbSigma);
  s.setValue("windTurbTau", w.turbTau);
  s.setValue("windEnabled", w.enabled);
  s.endGroup();
}

vsim::WindConfig SimulatorWidget::restoreWind() {
  vsim::WindConfig w;   // defaults (disabled, zero) if nothing saved
  QSettings s;
  s.beginGroup(kWorldGroup);
  w.steady = QVector3D(s.value("windN", w.steady.x()).toFloat(),
                       s.value("windE", w.steady.y()).toFloat(),
                       s.value("windD", w.steady.z()).toFloat());
  w.gustAmp    = s.value("windGustAmp", w.gustAmp).toFloat();
  w.gustPeriod = s.value("windGustPeriod", w.gustPeriod).toFloat();
  w.turbSigma  = s.value("windTurbSigma", w.turbSigma).toFloat();
  w.turbTau    = s.value("windTurbTau", w.turbTau).toFloat();
  w.enabled    = s.value("windEnabled", w.enabled).toBool();
  s.endGroup();
  return w;
}

void SimulatorWidget::onUartBytes(QByteArray bytes) {
  // Forward the firmware's telemetry downlink to anything connected to
  // dataReceived (MainWindow's DroneProtocol parser typically). The FC view is
  // NOT logged here — that would just duplicate the Export; the per-run log
  // records the physics ground truth instead (see logGroundTruth).
  emit dataReceived(bytes);
}

// Ground-truth record size: 2 x u64 (tick, t_us) + 27 x f32 (pose/vel/rates/
// rpy/motors/airspeed/batt). Keep in sync with logGroundTruth + parse_gt.py.
static constexpr int kGtRecordBytes = 2 * 8 + 27 * 4;  // 124

void SimulatorWidget::openNewLogFile() {
  closeLogFile();
  QDir dir(m_logDir);
  if (!dir.exists() && !dir.mkpath(".")) {
    appendLog("log", QString("[mkdir failed: %1]").arg(m_logDir));
    return;
  }
  QString stamp =
      QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
  // gt- (ground truth), not sim-: this file holds the PHYSICS state, not the FC
  // telemetry. Legacy tools (tools/sim_log_*.py) still parse the old sim-*.bin
  // as FC NavLink, so a distinct name keeps them from mis-reading this format.
  QString path = dir.absoluteFilePath(QString("gt-%1.bin").arg(stamp));
  m_runLog = std::make_unique<QFile>(path);
  if (!m_runLog->open(QIODevice::WriteOnly)) {
    appendLog("log", QString("[open failed: %1]").arg(path));
    m_runLog.reset();
    return;
  }
  m_runLogBytes = 0;
  // Header (little-endian, 20 bytes): magic "VGT1", version, record size,
  // start wall-clock ms. Per-record layout documented in logGroundTruth /
  // docs/journal/log-analysis/parse_gt.py.
  {
    QDataStream hs(m_runLog.get());
    hs.setByteOrder(QDataStream::LittleEndian);
    hs.writeRawData("VGT1", 4);
    hs << quint32(1) << quint32(kGtRecordBytes)
       << quint64(QDateTime::currentMSecsSinceEpoch());
    m_runLogBytes += 20;
  }
  m_runClock.start();  // monotonic per-record timestamp base
  if (m_logPathLabel) {
    m_logPathLabel->setText(QString("Ground-truth log: %1").arg(path));
    m_logPathLabel->setStyleSheet(
        QString("color: %1; font-family: monospace; font-size: 11px;")
            .arg(Theme::hex(Theme::kOk)));
  }
  appendLog("log", QString("[opened %1 — physics ground truth]").arg(path));
}

// One ground-truth record per pose snapshot. Layout (little-endian, packed,
// kGtRecordBytes total) — kept in lock-step with parse_gt.py:
//   u64 tick_count            physics tick (sim virtual-time key)
//   u64 t_us                  microseconds since the log opened (monotonic)
//   f32 pos[3]                NED position [m]
//   f32 quat[4]               body->world quaternion, w first
//   f32 vel[3]                NED velocity [m/s]
//   f32 omega[3]              body angular velocity [rad/s]
//   f32 rpy[3]                NED Tait-Bryan roll/pitch/yaw [deg] (convenience)
//   f32 motor_omega[4]        per-rotor [rad/s]
//   f32 motor_duty[4]         per-rotor [0,1]
//   f32 airspeed              [m/s]
//   f32 batt_voltage          [V]
//   f32 batt_soc              [0,1]
void SimulatorWidget::logGroundTruth(vsim::SimSnapshot snap) {
  if (!m_runLog || !m_runLog->isOpen()) return;
  float roll, pitch, yaw;
  vsim::quatToEulerNED(snap.att, &roll, &pitch, &yaw);

  QByteArray rec;
  rec.reserve(kGtRecordBytes);
  QDataStream s(&rec, QIODevice::WriteOnly);
  s.setByteOrder(QDataStream::LittleEndian);
  s.setFloatingPointPrecision(QDataStream::SinglePrecision);
  s << quint64(snap.tick_count)
    << quint64(m_runClock.nsecsElapsed() / 1000);
  s << snap.pos_w.x() << snap.pos_w.y() << snap.pos_w.z();
  s << snap.att.scalar() << snap.att.x() << snap.att.y() << snap.att.z();
  s << snap.vel_w.x() << snap.vel_w.y() << snap.vel_w.z();
  s << snap.omega_b.x() << snap.omega_b.y() << snap.omega_b.z();
  s << roll << pitch << yaw;
  for (float v : snap.motor_omega) s << v;
  for (float v : snap.motor_duty) s << v;
  s << snap.airspeed << snap.batt_voltage << snap.batt_soc;

  m_runLog->write(rec);
  m_runLogBytes += rec.size();
  // Occasional size readout (every ~5 s at 60 Hz pose).
  static int chatter_div = 0;
  if (++chatter_div % 300 == 0)
    appendLog("gt", QString("logged %1 KB ground truth").arg(m_runLogBytes / 1024));
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
