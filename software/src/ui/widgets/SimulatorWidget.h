#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSocketNotifier>
#include <QString>
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

  QString workingDir() const;

  // ---- repo root + persistence ----
  QString m_repoRoot;
  QLineEdit* m_repoRootEdit = nullptr;

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

  // ---- log ----
  QPlainTextEdit* m_log = nullptr;
};
