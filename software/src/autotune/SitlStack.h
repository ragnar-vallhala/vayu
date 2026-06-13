#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

#include <QObject>
#include <QProcessEnvironment>
#include <QString>

#include "Cost.h"  // autotune::Sample

class QProcess;

// C++ port of tools/autotune/sitl.py SitlStack: spins up an isolated SITL stack
// (a separate vsim_d physics daemon + vayu_sitl firmware host), drives it over
// the same PTY/FIFO channels the Python harness uses, and decodes the firmware's
// navlink telemetry into autotune::Sample windows — so the GCS can run the
// autotuner natively in C++ with no python3 subprocess.
//
// Channels (per the firmware host shim + vsim_proto):
//   - RC:    a PTY we create; firmware reads CSV frames from $VAYU_UART_RC_PATH.
//   - cmds:  the firmware's UART2 PTY (advertised in /tmp/vayu_uart2_pty<suffix>);
//            we write navlink command frames (CommandCodec) to it.
//   - tele:  same UART2 PTY, read back + fed to DroneProtocol.
//   - sim:   vsim_d control FIFO /tmp/vsim_ctl<suffix> (vsim_proto ctl frames).
//
// Linux/POSIX (the SITL stack is Linux-only). One reader thread decodes
// telemetry; one writer thread paces RC; command/ctl writes come from the
// caller (rollout) thread. Buffers are mutex-guarded.
class SitlStack : public QObject {
  Q_OBJECT

public:
  struct Config {
    QString vsimBin;   // path to vsim_d
    QString sitlBin;   // path to vayu_sitl
    QString suffix;    // isolation suffix for FIFO/PTY paths
    int imuHz = 1000;
    int physicsHz = 8000;
    int poseHz = 120;
    bool quiet = true;
  };

  explicit SitlStack(Config cfg, QObject *parent = nullptr);
  ~SitlStack() override;

  bool start(QString *err = nullptr);  // spawn + open channels + wait ready
  void stop();
  bool isRunning() const { return m_running; }

  // --- firmware commands (navlink over the UART2 PTY) ---
  void setPid(int controller, int axis, float kp, float ki, float kd,
              float kff);
  void setGyroLpf(int axis, float rc);
  void setFlightMode(int mode);

  // --- RC (paced to the firmware @ ~50 Hz); -1 keeps the current value ---
  void setRc(int roll = -1, int pitch = -1, int thr = -1, int yaw = -1,
             int arm = -1, int ch6 = -1);

  // --- vsim_d control ---
  void reset(quint32 seed = 0);
  void setTestRig(bool on, float tetherK = 0.0f);

  // --- arm / state (state names come from telemetry: STANDBY/ARMED/...) ---
  bool arm(int timeoutMs = 2000);
  void disarm();
  bool waitState(const QString &name, int timeoutMs);
  QString lastState() const;

  // --- telemetry samples ---
  void clearSamples();
  std::vector<autotune::Sample> snapshot() const;

private:
  bool openCtlFifo(QString *err);
  bool openUart2Pty(QString *err);
  bool openRcPty(QString *err, QString *rcSlavePathOut);
  void sendCtlRates();
  void sendCtlReset(quint32 seed);
  void sendCtlTestRig(bool on, float tetherK);
  void writeNavlink(const QByteArray &frame);
  void readerLoop();
  void rcWriterLoop();
  void onControlLoop(const autotune::Sample &s);
  void onStatus(const QString &state);

  Config m_cfg;
  QProcess *m_vsim = nullptr;
  QProcess *m_sitl = nullptr;
  int m_ctlFd = -1;
  int m_ptyFd = -1;      // firmware UART2 PTY (read telemetry / write commands)
  int m_rcMaster = -1;   // RC PTY master (we write CSV here)
  std::thread m_reader;
  std::thread m_rcWriter;
  std::atomic<bool> m_stop{false};
  std::atomic<bool> m_running{false};

  mutable std::mutex m_sampMtx;
  std::deque<autotune::Sample> m_samples;  // capped
  mutable std::mutex m_rcMtx;
  int m_rc[6] = {1500, 1500, 1000, 1500, 1000, 1000};  // RC_NEUTRAL
  mutable std::mutex m_stateMtx;
  QString m_lastState;
  std::mutex m_writeMtx;  // serialises navlink writes to the PTY
};
