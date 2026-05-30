#pragma once

#include "vsim/SimWorker.h"
#include "vsim/SimRendererWidget.h"

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

 public slots:
  // Called from the C UART2 callback via QMetaObject::invokeMethod
  // (Qt::QueuedConnection) so the firmware thread crossing the
  // boundary is safe. Public so the static C trampoline can find it
  // by name through the meta-object system.
  void onUartBytes(QByteArray bytes);

 private:
  void buildUi();
  void appendLog(const QString& tag, const QString& text);

  void startInAppSim();
  void stopInAppSim();

  void openNewLogFile();
  void closeLogFile();

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
  vsim::SimRendererWidget* m_renderer = nullptr;
  QPushButton* m_simStartBtn = nullptr;
  QPushButton* m_simStopBtn = nullptr;
  QLabel* m_simStatusLabel = nullptr;
  QLabel* m_simPoseLabel = nullptr;

  // ---- live readouts derived from snapshot ----
  QProgressBar* m_motorBars[4] = {nullptr, nullptr, nullptr, nullptr};
  QLabel* m_motorLabels[4] = {nullptr, nullptr, nullptr, nullptr};

  // ---- log ----
  QPlainTextEdit* m_log = nullptr;
};
