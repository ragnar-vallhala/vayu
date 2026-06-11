#pragma once

#include <QMutex>
#include <QString>
#include <QThread>
#include <QVector>

#include <array>
#include <atomic>

// RcBridge — feeds a USB RC transmitter (a Linux joystick, e.g. the
// "Artery PPM Controller" / FlySky-via-PPM dongle on /dev/input/js0) into
// the in-process firmware's RC input.
//
// The firmware's host_rc_feeder reads CSV microsecond frames from a tty at
// $VAYU_UART_RC_PATH (channels: [0]=roll [1]=pitch [2]=throttle [3]=yaw
// [4]=arm switch ...). We can't point that at a joystick directly, so this
// bridge opens a pty, hands the slave path to the caller (who sets the env
// var before vayu_sitl_start), reads the joystick, and writes one CSV
// frame per ~20 ms to the pty master.
//
// Axis → channel is linear: us = 1500 + axis*500/32767 (axis is
// -32767..32767), optionally inverted per channel. Throttle idles low when
// the stick is down (axis rests at -32767 → 1000 us).
class RcBridge : public QThread {
  Q_OBJECT
 public:
  explicit RcBridge(QObject* parent = nullptr);
  ~RcBridge() override;

  // RC input source. Joystick: read a Linux js device and map axes/buttons
  // to channels. Uart: read CSV microsecond frames straight off a serial
  // port (same wire format the firmware expects) and forward them. The pty
  // the firmware reads is unchanged either way, so the source can switch live.
  enum Source { Joystick = 0, Uart = 1 };
  void setSource(int s) { source_.store(s, std::memory_order_release); }

  void setJoystickPath(const QString& p) {
    QMutexLocker lk(&cfg_mtx_);
    js_path_ = p;
  }
  // Serial device + baud for the Uart source (e.g. /dev/ttyUSB1 @ 115200).
  void setUartPath(const QString& p) {
    QMutexLocker lk(&cfg_mtx_);
    uart_path_ = p;
  }
  void setUartBaud(int b) {
    QMutexLocker lk(&cfg_mtx_);
    uart_baud_ = b;
  }

  // When disabled, the bridge streams a neutral/idle frame (throttle low,
  // everything else centered) instead of the joystick — so toggling RC on
  // the running sim takes effect live, without a restart.
  void setEnabled(bool on) { enabled_.store(on, std::memory_order_release); }

  // Gate for the Uart source: the serial device is NOT opened until this is
  // set true (a deliberate "Connect" from the UI). Keeps the sim from grabbing
  // a tty the board telemetry may be using. No effect on the Joystick source
  // (which never collides with the board port). Default off.
  void setUartConnected(bool on) {
    uartConnected_.store(on, std::memory_order_release);
  }
  bool uartConnected() const {
    return uartConnected_.load(std::memory_order_acquire);
  }

  // Map an output function to a joystick source (+ invert). func: 0=roll,
  // 1=pitch, 2=throttle, 3=yaw, 4=arm. `source` encodes the input:
  // 0..15 = axis index; 1000+b = button b (pressed→2000, released→1000).
  // Live; safe from the GUI thread.
  enum Func { Roll = 0, Pitch = 1, Throttle = 2, Yaw = 3, Arm = 4 };
  static constexpr int kButtonBase = 1000;
  void setMapping(int func, int source, bool invert);

  // Software arm override for TXs with no usable arm switch.
  // v: -1 = use the mapped source; 0 = force disarmed; 1 = force armed.
  void setArmOverride(int v) { armOverride_.store(v, std::memory_order_relaxed); }

  // Flight-mode toggle driving RC channel 6 (acro/rate mode in the firmware).
  // on = acro (ch6 high), off = angle/stabilize. Safe from the GUI thread.
  void setAcro(bool on) { acro_.store(on ? 1 : 0, std::memory_order_relaxed); }
  bool acro() const { return acro_.load(std::memory_order_relaxed) != 0; }

  // Create the pty pair; slavePath() is then valid. Returns false (and
  // sets *err) on failure. Call before start() / before vayu_sitl_start.
  bool openPty(QString* err);
  QString slavePath() const { return slave_path_; }

  void requestStop();

 signals:
  // Latest output channel values (microseconds), for an on-screen readout.
  void channelsUpdated(int roll, int pitch, int thr, int yaw, int arm, int ch6);
  // Per-axis µs + per-button states (0/1), to identify which input each
  // physical control is on.
  void axesUpdated(QVector<int> axisUs, QVector<int> buttons);
  void logLine(QString line);

 protected:
  void run() override;

 private:
  QMutex cfg_mtx_;  // guards js_path_ / uart_path_ / uart_baud_ (live edits)
  QString js_path_ = QStringLiteral("/dev/input/js0");
  QString uart_path_ = QStringLiteral("/dev/ttyUSB0");
  int uart_baud_ = 115200;
  std::atomic<int> source_{Joystick};
  QString slave_path_;
  int master_fd_ = -1;
  std::atomic<bool> stop_{false};
  std::atomic<bool> enabled_{true};
  std::atomic<bool> uartConnected_{false};  // Uart source opened only when true
  std::atomic<int> armOverride_{-1};   // -1 use mapping, 0 disarm, 1 arm
  std::atomic<int> acro_{0};           // 0 = angle mode (ch6 low), 1 = acro
  std::atomic<int> mapAxis_[5];   // func → source code (axis idx or 1000+btn)
  std::atomic<int> mapInv_[5];    // func → invert (0/1)
  std::array<int, 16> axis_{};    // raw -32767..32767
  std::array<int, 16> button_{};  // 0/1
};
