#pragma once

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSocketNotifier>
#include <QString>
#include <QTimer>
#include <QWidget>

class QGridLayout;

/**
 * SimulatorWidget — control + monitor page for the native host SITL stack
 * (tools/sim_host/vayu_sitl + tools/sim_gazebo/ bridges + Gazebo Harmonic).
 *
 * Provides:
 *   - per-process Start / Stop + status, with editable commands and
 *     persistent vayu repo root
 *   - "Launch All" / "Stop All" buttons that walk the four processes in
 *     order (gz sim, IMU bridge, vayu_sitl, PWM bridge)
 *   - live PWM duty bars driven by reading /tmp/vayu_pwm.fifo
 *   - merged stdout/stderr log
 */
class SimulatorWidget : public QWidget {
  Q_OBJECT
 public:
  explicit SimulatorWidget(QWidget* parent = nullptr);
  ~SimulatorWidget() override;

 signals:
  void backToHomeRequested();

 private slots:
  void onLaunchAll();
  void onStopAll();
  void onPwmReadable();
  void onVirtualRcToggled(bool on);
  void onRcTimerTick();

 private:
  struct Proc {
    QString name;
    QString tag;            // short tag for log lines
    QString defaultCmd;
    QPushButton* startBtn = nullptr;
    QPushButton* stopBtn = nullptr;
    QLabel* statusLabel = nullptr;
    QLineEdit* commandEdit = nullptr;
    QProcess* process = nullptr;
  };

  void buildUi();
  void buildProcessRow(QGridLayout* grid, int row, Proc* p);
  void connectProcessSignals(Proc* p);

  void startProcess(Proc* p);
  void stopProcess(Proc* p);
  void updateStatusLabel(Proc* p);
  void appendLog(const QString& tag, const QString& text);

  void openPwmFifo();
  void closePwmFifo();
  void parsePwmBuffer();

  bool openVirtualRcPty();
  void closeVirtualRcPty();

  QString workingDir() const;

  // ---- repo root + persistence ----
  QString m_repoRoot;
  QLineEdit* m_repoRootEdit = nullptr;

  // ---- passthrough toggle (env VAYU_SITL_PASSTHROUGH=1 prepended to SITL) ----
  QCheckBox* m_passthroughCheck = nullptr;

  // ---- four managed processes ----
  Proc m_gz;
  Proc m_imuBridge;
  Proc m_pwmBridge;
  Proc m_sitl;

  // ---- PWM monitor ----
  QProgressBar* m_motorBars[4] = {nullptr, nullptr, nullptr, nullptr};
  QLabel* m_motorLabels[4] = {nullptr, nullptr, nullptr, nullptr};
  QLabel* m_pwmStatusLabel = nullptr;
  int m_pwmFd = -1;
  QSocketNotifier* m_pwmNotifier = nullptr;
  QByteArray m_pwmBuf;

  // ---- virtual RC (sliders + arm switch -> pty CSV stream) ----
  //
  // When enabled, the GCS opens a pty pair, writes CSV frames in the
  // same format tools/sim_bridge/ emits (FS-i6 PPM channel widths at
  // 50 Hz) to the master fd, and the slave's /dev/pts/N path is
  // automatically prepended as VAYU_UART_RC_PATH=... to the SITL
  // process command at launch time. So the firmware's RC feeder sees
  // a normal serial source - same code path as the real Arduino.
  QCheckBox* m_virtualRcCheck = nullptr;
  QLabel* m_virtualRcPathLabel = nullptr;
  QSlider* m_rcRollSlider = nullptr;
  QSlider* m_rcPitchSlider = nullptr;
  QSlider* m_rcThrottleSlider = nullptr;
  QSlider* m_rcYawSlider = nullptr;
  QCheckBox* m_rcArmSwitch = nullptr;
  QLabel* m_rcRollLabel = nullptr;
  QLabel* m_rcPitchLabel = nullptr;
  QLabel* m_rcThrottleLabel = nullptr;
  QLabel* m_rcYawLabel = nullptr;
  QPushButton* m_rcRecenterBtn = nullptr;
  int m_ptyMasterFd = -1;
  QString m_ptySlavePath;
  QTimer* m_rcTimer = nullptr;

  // ---- log ----
  QPlainTextEdit* m_log = nullptr;
};
