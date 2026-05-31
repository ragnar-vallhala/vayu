#include "RcBridge.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>

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

  const int js = ::open(js_path_.toLocal8Bit().constData(),
                        O_RDONLY | O_NONBLOCK);
  if (js < 0) {
    emit logLine(QString("RC: cannot open %1 (%2) — sending neutral frames")
                     .arg(js_path_, ::strerror(errno)));
  } else {
    emit logLine(QString("RC: reading %1 → %2").arg(js_path_, slave_path_));
  }

  // Throttle idles low; everything else centered until the device reports.
  axis_.fill(0);
  axis_[2] = -32767;
  button_.fill(0);

  while (!stop_.load(std::memory_order_acquire)) {
    if (js >= 0) {
      js_event e;
      ssize_t r;
      while ((r = ::read(js, &e, sizeof(e))) == (ssize_t)sizeof(e)) {
        if ((e.type & kJsAxis) && e.number < axis_.size())
          axis_[e.number] = e.value;
        else if ((e.type & kJsButton) && e.number < button_.size())
          button_[e.number] = e.value;
      }
    }

    const bool en = enabled_.load(std::memory_order_acquire);
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
    const int roll = en ? chan(0) : 1500;
    const int pitch = en ? chan(1) : 1500;
    const int thr = en ? chan(2) : 1000;   // idle when disabled
    const int yaw = en ? chan(3) : 1500;
    int arm = en ? chan(4) : 1500;
    const int ov = armOverride_.load(std::memory_order_relaxed);
    if (ov >= 0) arm = ov ? 2000 : 1000;   // software arm override

    // CSV frame: roll,pitch,throttle,yaw,arm,aux  (feeder fills 6..13 = 1500)
    char buf[96];
    const int n = std::snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d\n",
                                roll, pitch, thr, yaw, arm, 1500);
    if (master_fd_ >= 0 && n > 0) {
      const ssize_t w = ::write(master_fd_, buf, (size_t)n);
      (void)w;  // pty buffer full → drop frame; next one is along in 20 ms
    }
    emit channelsUpdated(roll, pitch, thr, yaw, arm);

    // Per-axis µs (first 6) + button states (first 4) so the UI can show
    // which input each physical control is on.
    QVector<int> au;
    au.reserve(6);
    for (int i = 0; i < 6; ++i) au.push_back(axisToUs(axis_[i], false));
    QVector<int> bu;
    bu.reserve(4);
    for (int i = 0; i < 4; ++i) bu.push_back(button_[i] ? 1 : 0);
    emit axesUpdated(au, bu);

    msleep(20);  // ~50 Hz
  }

  if (js >= 0) ::close(js);
  emit logLine(QStringLiteral("RC: stopped"));
}
