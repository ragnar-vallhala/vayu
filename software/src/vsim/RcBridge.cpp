#include "RcBridge.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <string>

namespace {

// Linux joystick API event (linux/joystick.h), redeclared to avoid a
// kernel-header dependency.
struct js_event {
  uint32_t time;     // ms
  int16_t value;     // -32767..32767 for axes
  uint8_t type;      // 0x01 button, 0x02 axis, |0x80 = synthetic init
  uint8_t number;    // axis/button index
};
constexpr uint8_t kJsButton = 0x01;
constexpr uint8_t kJsAxis = 0x02;

int axisToUs(int v, bool invert) {
  if (invert) v = -v;
  const int us = 1500 + (v * 500) / 32767;
  return std::clamp(us, 1000, 2000);
}

speed_t baudConst(int b) {
  switch (b) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B115200;
  }
}

// Open a serial port in raw mode at `baud` for reading CSV RC frames.
int openUart(const char* path, int baud) {
  int fd = ::open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  struct termios tio;
  if (::tcgetattr(fd, &tio) == 0) {
    ::cfmakeraw(&tio);
    const speed_t sp = baudConst(baud);
    ::cfsetispeed(&tio, sp);
    ::cfsetospeed(&tio, sp);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::tcsetattr(fd, TCSANOW, &tio);
  }
  return fd;
}

// Parse a CSV line of microsecond channel values. Assigns the first up-to-6
// fields to roll/pitch/throttle/yaw/arm/ch6 (clamped 1000..2000); leaves any
// missing trailing channels untouched so a short frame holds the last value.
// ch6 is the acro/angle flight-mode switch (firmware: > 1500 = acro).
void parseCsvFrame(const std::string& line, int* roll, int* pitch, int* thr,
                   int* yaw, int* arm, int* ch6) {
  int* out[6] = {roll, pitch, thr, yaw, arm, ch6};
  const char* p = line.c_str();
  for (int i = 0; i < 6 && *p; ++i) {
    char* end = nullptr;
    const long v = std::strtol(p, &end, 10);
    if (end == p) break;  // no number here
    *out[i] = std::clamp((int)v, 1000, 2000);
    p = end;
    while (*p == ',' || *p == ' ' || *p == '\t') ++p;
  }
}

}  // namespace

RcBridge::RcBridge(QObject* parent) : QThread(parent) {
  // Default 1:1 mapping: func f ← axis f, no invert.
  for (int f = 0; f < 5; ++f) { mapAxis_[f] = f; mapInv_[f] = 0; }
}

void RcBridge::setMapping(int func, int axis, bool invert) {
  if (func < 0 || func >= 5) return;
  mapAxis_[func].store(axis, std::memory_order_relaxed);
  mapInv_[func].store(invert ? 1 : 0, std::memory_order_relaxed);
}

RcBridge::~RcBridge() {
  requestStop();
  if (isRunning()) wait(500);
  if (master_fd_ >= 0) ::close(master_fd_);
}

bool RcBridge::openPty(QString* err) {
  master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 ||
      ::unlockpt(master_fd_) != 0) {
    if (err) *err = QString("pty setup failed: %1").arg(::strerror(errno));
    if (master_fd_ >= 0) { ::close(master_fd_); master_fd_ = -1; }
    return false;
  }
  const char* sn = ::ptsname(master_fd_);
  if (!sn) {
    if (err) *err = QStringLiteral("ptsname failed");
    ::close(master_fd_);
    master_fd_ = -1;
    return false;
  }
  slave_path_ = QString::fromLocal8Bit(sn);
  return true;
}

void RcBridge::requestStop() { stop_.store(true, std::memory_order_release); }

void RcBridge::run() {
  stop_.store(false, std::memory_order_release);

  // Throttle idles low; everything else centered until the device reports.
  axis_.fill(0);
  axis_[2] = -32767;
  button_.fill(0);

  // Effective source actually open: Joystick, Uart, or kSrcNone (no device →
  // neutral frames). The Uart source resolves to kSrcNone until the UI calls
  // setUartConnected(true), so the sim never opens the board's tty on its own.
  constexpr int kSrcNone = -2;
  int cur_src = -99;    // last effective source opened (force first (re)open)
  int fd = -1;          // js or uart fd; the pty (master_fd_) is separate
  std::string uline;    // UART CSV line accumulator
  // Last UART-parsed channels, held between frames.
  int u_roll = 1500, u_pitch = 1500, u_thr = 1000, u_yaw = 1500, u_arm = 1500;
  int u_ch6 = 1000;   // transmitter's ch6 (acro switch); 1000 = stabilise default

  auto close_src = [&]() { if (fd >= 0) { ::close(fd); fd = -1; } };

  while (!stop_.load(std::memory_order_acquire)) {
    const int sel = source_.load(std::memory_order_acquire);
    // Gate the Uart source behind the explicit connect flag.
    const int want =
        (sel == Uart && !uartConnected_.load(std::memory_order_acquire))
            ? kSrcNone
            : sel;

    // (Re)open the input device when the effective source changes. master_fd_
    // (the pty the firmware reads) stays put, so switching source is seamless.
    if (want != cur_src) {
      close_src();
      uline.clear();
      QString path;
      int baud;
      {
        QMutexLocker lk(&cfg_mtx_);
        path = (want == Uart) ? uart_path_ : js_path_;
        baud = uart_baud_;
      }
      if (want == kSrcNone) {
        // No device claimed — stream neutral frames, hold the port free.
        emit logLine(QStringLiteral("RC: UART disconnected — neutral frames"));
        axis_.fill(0);
        axis_[2] = -32767;
        button_.fill(0);
      } else if (want == Uart) {
        fd = openUart(path.toLocal8Bit().constData(), baud);
        if (fd < 0)
          emit logLine(QString("RC: cannot open UART %1 (%2) — neutral frames")
                           .arg(path, ::strerror(errno)));
        else
          emit logLine(
              QString("RC: UART %1 @ %2 → %3").arg(path).arg(baud).arg(slave_path_));
      } else {
        fd = ::open(path.toLocal8Bit().constData(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
          emit logLine(QString("RC: cannot open %1 (%2) — neutral frames")
                           .arg(path, ::strerror(errno)));
        else
          emit logLine(QString("RC: reading %1 → %2").arg(path, slave_path_));
        axis_.fill(0);
        axis_[2] = -32767;
        button_.fill(0);
      }
      cur_src = want;
    }

    const bool en = enabled_.load(std::memory_order_acquire);
    int roll, pitch, thr, yaw, arm;

    if (want == Uart) {
      // Drain available bytes, split on newlines, parse the latest frame.
      if (fd >= 0) {
        char b[256];
        ssize_t r;
        while ((r = ::read(fd, b, sizeof(b))) > 0) {
          for (ssize_t i = 0; i < r; ++i) {
            const char c = b[i];
            if (c == '\n' || c == '\r') {
              if (!uline.empty()) {
                parseCsvFrame(uline, &u_roll, &u_pitch, &u_thr, &u_yaw, &u_arm,
                              &u_ch6);
                uline.clear();
              }
            } else if (uline.size() < 128) {
              uline += c;
            }
          }
        }
      }
      roll  = en ? u_roll  : 1500;
      pitch = en ? u_pitch : 1500;
      thr   = en ? u_thr   : 1000;
      yaw   = en ? u_yaw   : 1500;
      arm   = en ? u_arm   : 1500;
      emit axesUpdated({}, {});  // no per-axis breakdown for a CSV source
    } else {
      // Joystick: pump events, then map axes/buttons to channels.
      if (fd >= 0) {
        js_event e;
        ssize_t r;
        while ((r = ::read(fd, &e, sizeof(e))) == (ssize_t)sizeof(e)) {
          if ((e.type & kJsAxis) && e.number < axis_.size())
            axis_[e.number] = e.value;
          else if ((e.type & kJsButton) && e.number < button_.size())
            button_[e.number] = e.value;
        }
      }
      auto chan = [&](int f) {
        const int src = mapAxis_[f].load(std::memory_order_relaxed);
        const bool inv = mapInv_[f].load(std::memory_order_relaxed) != 0;
        if (src >= kButtonBase) {           // button source: pressed → 2000
          const int b = src - kButtonBase;
          bool on = (b >= 0 && b < (int)button_.size()) ? button_[b] != 0 : false;
          if (inv) on = !on;
          return on ? 2000 : 1000;
        }
        const int raw = (src >= 0 && src < (int)axis_.size()) ? axis_[src] : 0;
        return axisToUs(raw, inv);
      };
      roll  = en ? chan(0) : 1500;
      pitch = en ? chan(1) : 1500;
      thr   = en ? chan(2) : 1000;   // idle when disabled
      yaw   = en ? chan(3) : 1500;
      arm   = en ? chan(4) : 1500;

      QVector<int> au;
      au.reserve(6);
      for (int i = 0; i < 6; ++i) au.push_back(axisToUs(axis_[i], false));
      QVector<int> bu;
      bu.reserve(4);
      for (int i = 0; i < 4; ++i) bu.push_back(button_[i] ? 1 : 0);
      emit axesUpdated(au, bu);
    }

    const int ov = armOverride_.load(std::memory_order_relaxed);
    if (ov >= 0) arm = ov ? 2000 : 1000;   // software arm override

    // CSV frame: roll,pitch,throttle,yaw,arm,ch6  (feeder fills 7..13 = 1500).
    // ch6 carries the acro/angle flight-mode toggle (firmware: > 1500 = acro).
    // In UART mode the transmitter's physical ch6 switch is authoritative (just
    // like real RC); the Acro checkbox drives it only for the generated source.
    const int ch6 = (want == Uart)
                        ? (en ? u_ch6 : 1000)
                        : (acro_.load(std::memory_order_relaxed) ? 2000 : 1000);
    char buf[96];
    const int n = std::snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d\n",
                                roll, pitch, thr, yaw, arm, ch6);
    if (master_fd_ >= 0 && n > 0) {
      const ssize_t w = ::write(master_fd_, buf, (size_t)n);
      (void)w;  // pty buffer full → drop frame; next one is along in 20 ms
    }
    emit channelsUpdated(roll, pitch, thr, yaw, arm, ch6);

    msleep(20);  // ~50 Hz
  }

  close_src();
  emit logLine(QStringLiteral("RC: stopped"));
}
