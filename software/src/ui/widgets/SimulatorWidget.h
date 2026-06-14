#pragma once

#include "vsim/SimWorker.h"
#include "vsim/SimRendererWidget.h"
#include "vsim/RcBridge.h"
#include "GeometryEditorWidget.h"
#include "WorldEditorWidget.h"
#include "SimHudWidget.h"
#include "HorizonHud.h"
#include "TuneChart.h"
#include "AutotuneGains.h"
#include "../../audio/PropAudio.h"

extern "C" {
#include "vsim_iface.h"
}

#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QString>
#include <QWidget>
#include <memory>

class QGridLayout;
class QStackedWidget;
class QPushButton;
class QCheckBox;
class QSlider;
class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class QProcess;

namespace vsim { struct LoadedMesh; }

/**
 * SimulatorWidget - control + monitor page for the SITL.
 *
 * Two-process architecture: this widget owns
 *   - a vsim_iface_t (used only for the UART2 telemetry callback;
 *     PWM and IMU no longer ride through it)
 *   - a vsim::SimWorker, which spawns the standalone vsim_d physics
 *     daemon on Start and reads pose frames off /tmp/vsim_pose
 *   - a vsim::SimRendererWidget (OpenGL view of the airframe)
 * and calls vayu_sitl_start(&iface) to boot the firmware threads
 * inside Navigator. The firmware's host shims (host_navhal +
 * host_imu_feeder) talk to vsim_d via /tmp/vsim_{pwm,imu}.
 *
 * vayu_sitl_start can be called at most ONCE per Navigator process
 * (see host_lifecycle.c comments); the Stop button only tears down
 * the vsim_d process and pose reader. A Navigator restart fully
 * resets the firmware state.
 */
class SimulatorWidget : public QWidget {
  Q_OBJECT
 public:
  explicit SimulatorWidget(QWidget* parent = nullptr);
  ~SimulatorWidget() override;

 signals:
  void backToHomeRequested();

  /* Mirror of SerialManager::dataReceived. Emitted on the GUI thread
   * whenever the in-process firmware writes to UART2 - DroneProtocol
   * packets, vayu_log() text, telemetry. MainWindow connects this to
   * the same DroneProtocol parser the real serial path feeds, so the
   * existing telemetry panels light up without further plumbing. */
  void dataReceived(const QByteArray& bytes);

  /* Emitted when the in-app sim starts (true) / stops (false), so the
   * MainWindow can reflect "Connected: SIM" in the status bar. */
  void simRunningChanged(bool running);

  /* AT-1: the operator clicked "Apply Gains to Firmware". MainWindow turns
   * each PidSetCmd into a CMD_SET_PID frame and sends it over the live link.
   * The autotuner never writes to firmware on its own. */
  void applyPidGainsRequested(const QVector<PidSetCmd>& cmds);

 protected:
  // Keeps the HUD overlay sized to the viewport (watches m_renderer resize).
  bool eventFilter(QObject* obj, QEvent* ev) override;

 public slots:
  // Called from the C UART2 callback via QMetaObject::invokeMethod
  // (Qt::QueuedConnection) so the firmware thread crossing the
  // boundary is safe. Public so the static C trampoline can find it
  // by name through the meta-object system.
  void onUartBytes(QByteArray bytes);

  // Forward telemetry-decoded values (parsed in MainWindow) into the HUD
  // overlay: the flight-state name and the latest IMU accel/gyro sample.
  void hudSetStatus(const QString& s);
  void hudSetImu(const float acc[3], const float gyr[3]);
  // Reflect the firmware's reported flight mode (SYSTEM_ORIGIN_FLIGHT_MODE):
  // syncs the Acro checkbox without re-issuing a command, and shows the source
  // (RC switch vs GCS override).
  void setFlightModeStatus(quint8 mode, quint8 source);

 private:
  void buildUi();
  void appendLog(const QString& tag, const QString& text);

  void startInAppSim();
  void stopInAppSim();
  void pushRatesToSim();   // read persisted rates → m_sim->sendRates
  // Load the configured world mesh (baking up-axis/scale into NED) and push
  // it to the renderer; empty path clears it. When the sim is running, also
  // builds the collision BVH and ships it to the daemon (sendWorldMeshToSim).
  void loadWorldMeshToRenderer();
  // Build a serialized BVH from an already-loaded mesh, write it atomically to
  // an mmap file, and point the daemon at it (VSIM_CTL_SET_WORLD_MESH).
  void sendWorldMeshToSim(const vsim::LoadedMesh& m);
  // Invoked when the SimWorker exits on its own (spawn failure, startup-grace
  // timeout, daemon death) so the UI doesn't get stuck in the Running state.
  void onSimWorkerExited();

  void openNewLogFile();
  void closeLogFile();

  // Push the editor's motor layout + CoM (and mesh, if loaded) into the
  // renderer so the 3D preview matches the configured airframe.
  void applyGeometryToRenderer();
  void persistGeometry(const vsim::GeometryConfig& g);
  vsim::GeometryConfig restoreGeometry();
  void persistWorld(const vsim::WorldConfig& w);
  vsim::WorldConfig restoreWorld();

  // Switch the right-hand properties panel: 0 = Vehicle, 1 = World.
  // Vehicle is locked while the sim is running.
  void setMode(int mode);

  // Push the current rig sliders' orientation to the sim (test-rig pose).
  void applyRigPose();
  // Enable/disable the rig pose controls — they need a running in-app sim to
  // pose against. Called when the sim starts/stops.
  void setRigControlsEnabled(bool simRunning);

  // Refresh the viewport HUD overlay from a pose snapshot.
  void updateHud(const vsim::SimSnapshot& s);

  // ---- repo root (kept for legacy widget consistency) ----
  QString m_repoRoot;
  QLineEdit* m_repoRootEdit = nullptr;

  // ---- per-run raw UART2 byte log ----
  QString m_logDir;                                  // editable in UI
  QLineEdit* m_logDirEdit = nullptr;
  QLabel* m_logPathLabel = nullptr;                  // shows current run's file
  std::unique_ptr<QFile> m_runLog;                   // open while sim running
  qint64 m_runLogBytes = 0;

  // ---- shared iface + in-app sim ----
  vsim_iface_t m_iface{};
  bool m_ifaceInit = false;
  bool m_sitlStarted = false;
  vsim::SimWorker* m_sim = nullptr;
  RcBridge* m_rc = nullptr;        // RC transmitter → firmware RC feeder
  QCheckBox* m_rcEnable = nullptr;
  QComboBox* m_rcSource = nullptr;          // USB joystick vs UART (CSV)
  QComboBox* m_rcBaud = nullptr;            // UART baud (UART source only)
  QComboBox* m_rcPath = nullptr;            // editable: device path or picked port
  QPushButton* m_rcConnect = nullptr;       // explicit connect for the RC UART
  bool m_rcUartConnected = false;           // is the RC UART device opened?
  QLabel* m_rcReadout = nullptr;
  QLabel* m_rcAxesLabel = nullptr;          // live per-axis µs (identify)
  QCheckBox* m_acroChk = nullptr;           // acro toggle; synced from telemetry
  QLabel* m_flightModeLabel = nullptr;      // shows effective mode + source
  QComboBox* m_rcAxisCombo[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  QCheckBox* m_rcInvert[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  QCheckBox* m_swArm = nullptr;             // software arm (no hardware switch)
  void pushRcMapping(int func);             // combo/invert → bridge + persist
  // Apply the selected RC source (joystick vs UART CSV) to the bridge and the
  // UI: swap the device field, enable/disable baud + axis mapping.
  void applyRcSource();
  void commitRcPath();                      // device field → bridge + persist
  // Open/close the RC UART device via the bridge, claiming/releasing the port
  // through PortArbiter so the board telemetry and the sim never collide.
  void setRcUartConnected(bool on);
  vsim::SimRendererWidget* m_renderer = nullptr;
  SimHudWidget* m_hud = nullptr;   // FPV telemetry overlay on the viewport
  HorizonHud* m_horizon = nullptr; // compact attitude indicator, top-right corner
  PropAudio m_propAudio;           // rpm-driven propeller sound
  GeometryEditorWidget* m_geomEditor = nullptr;
  WorldEditorWidget* m_worldEditor = nullptr;
  QStackedWidget* m_rightStack = nullptr;   // 0 = Vehicle, 1 = World
  QPushButton* m_vehicleTab = nullptr;
  QPushButton* m_worldTab = nullptr;
  QPushButton* m_tuneTab = nullptr;

  // ---- Fault injection (mockup Vehicle ▸ Fault Injection) ----
  bool m_motorKill[4] = {false, false, false, false};
  bool m_imuDropout = false;          // driven by the Sensor Models IMU enable
  QPushButton* m_killBtn[4] = {nullptr, nullptr, nullptr, nullptr};
  QWidget* buildFaultPanel();         // kill-motor / RC-loss / GPS-glitch group
  void pushFaults();                  // current fault flags → SimWorker

  // ---- Sensor models (mockup Vehicle ▸ Sensor Models) ----
  struct SensorRow {
    class QCheckBox* en = nullptr;
    class QDoubleSpinBox* sigma = nullptr;
    class QDoubleSpinBox* clip = nullptr;
  };
  SensorRow m_sensorRow[3];           // 0=accel, 1=gyro, 2=mag
  QWidget* buildSensorPanel();        // per-sensor noise σ / bias-clip / enable
  void pushNoise();                   // current sensor model → SimWorker

  // ---- Autotune section (drives tools/autotune against the current vehicle) ----
  QComboBox* m_tuneOptimizer = nullptr;
  QSpinBox* m_tuneBudget = nullptr;
  QSpinBox* m_tuneStep = nullptr;        // excitation amplitude (doublet µs)
  QSpinBox* m_tuneRepeats = nullptr;     // rollouts averaged per eval (--repeats)
  QDoubleSpinBox* m_tuneTether = nullptr; // soft-rig stiffness (--rig-tether)
  QSpinBox* m_tuneSeed = nullptr;        // optimizer RNG seed (--seed)
  QSpinBox* m_tuneSimSeed = nullptr;     // sensor-noise base seed (--sim-seed)
  QCheckBox* m_tuneYaw = nullptr;
  QCheckBox* m_tunePlot = nullptr;
  QCheckBox* m_tuneCompare = nullptr;    // run every optimizer (--compare)
  QCheckBox* m_tuneValidate = nullptr;   // free-flight validation (--no-validate if off)
  QCheckBox* m_tuneBuzz = nullptr;       // throttle buzz check (--no-buzz-check if off)
  QCheckBox* m_tuneVerbose = nullptr;    // print every eval (--verbose)
  QPushButton* m_tuneStart = nullptr;
  QPushButton* m_tuneStop = nullptr;
  QPlainTextEdit* m_tuneLog = nullptr;
  QLabel* m_tuneResult = nullptr;
  TuneChart* m_tuneChart = nullptr;
  // AT-1: autotune proposes; applying to firmware is an explicit click.
  QLabel* m_tuneProposed = nullptr;     // human-readable best gains
  QPushButton* m_tuneApplyBtn = nullptr;  // "Apply Gains to Firmware"
  class QTableWidget* m_tuneGainsTable = nullptr;  // AT-2: current vs best
  QString m_tuneOutJson;                // --out path for the running search
  QStringList m_tuneParams;             // param names from the result
  QVector<double> m_tuneBestX;          // best gain vector (the proposal)
  void parseProposedGains();            // read m_tuneOutJson into the above
  void applyProposedGains();            // emit applyPidGainsRequested
  // Test-rig pose controls: pin the airframe and tilt it on the stand.
  QCheckBox* m_rigEnable = nullptr;
  QSlider* m_rigRoll = nullptr;
  QSlider* m_rigPitch = nullptr;
  QSlider* m_rigYaw = nullptr;
  QLabel* m_rigReadout = nullptr;
  // C++ autotune engine on a worker thread (replaces the python3 subprocess).
  class QThread* m_tuneThread = nullptr;
  class AutotuneWorker* m_tuneWorker = nullptr;
  void onTuneEvaluated(const QVector<double>& current,
                       const QVector<double>& best, double cost,
                       double bestCost, int n);
  void onTuneFinished(const QVector<double>& bestX, const QStringList& names,
                      double bestCost);
  void onTuneDone();
  vsim::SimWorker* m_tuneSim = nullptr;   // attach-only: renders the tuner's sim
  void buildAutotunePage(QWidget* page);
  void attachTuneSim(const QString& suffix);   // mirror the tuner's drone in 3D
  void detachTuneSim();                         // stop mirroring, restore UI
  void startAutotune();
  void stopAutotune();
  QString exportVehicleGeometryJson();
  QString exportWorldJson();   // env (gravity/drag) so the tuner flies the same plant
  QPushButton* m_simStartBtn = nullptr;
  QPushButton* m_simStopBtn = nullptr;
  QPushButton* m_simResetBtn = nullptr;
  QCheckBox*   m_fpvCheck = nullptr;   // onboard FPV (only meaningful running)
  QLabel* m_simStatusLabel = nullptr;
  QLabel* m_simPoseLabel = nullptr;

  // ---- live readouts derived from snapshot ----
  QProgressBar* m_motorBars[4] = {nullptr, nullptr, nullptr, nullptr};
  QLabel* m_motorLabels[4] = {nullptr, nullptr, nullptr, nullptr};

  // ---- log ----
  QPlainTextEdit* m_log = nullptr;
};
