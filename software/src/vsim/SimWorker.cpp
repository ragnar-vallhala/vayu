#include "SimWorker.h"

#include <QDebug>
#include <QElapsedTimer>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

namespace vsim {

namespace {

constexpr int kPhysicsHz = 1000;             // physics step rate
constexpr int kImuHz     = 200;              // IMU emit rate
constexpr int kRenderHz  = 60;               // pose snapshot emit rate
constexpr float kRad2Deg = 57.29577951308232f;

// Cadence dividers: emit IMU every IMU_DIV ticks, snapshot every RENDER_DIV ticks.
constexpr int kImuDiv    = kPhysicsHz / kImuHz;     // 5
constexpr int kRenderDiv = kPhysicsHz / kRenderHz;  // ~16

// Wire layout of bmx160_all_converted_reading_t: 19 little-endian floats,
// total 76 B. Order:
//   acc[3], gyr[3], mag[3], acc_raw[3], gyr_raw[3], mag_compensated[3], temp
// The firmware static-asserts sizeof(...) == 76 on its end; we mirror that
// exact layout here. (gyr fields are in DEG/S, not rad/s -- conversion
// happens before pack.)
constexpr size_t kImuFrameBytes = 76;

}  // namespace

SimWorker::SimWorker(QObject* parent) : QThread(parent) {
  qRegisterMetaType<SimSnapshot>("vsim::SimSnapshot");
}

SimWorker::~SimWorker() {
  requestStop();
  if (isRunning()) {
    wait(2000);
  }
  if (pwm_fd_ >= 0) ::close(pwm_fd_);
  if (imu_fd_ >= 0) ::close(imu_fd_);
}

void SimWorker::requestStop() {
  stop_flag_.store(true, std::memory_order_release);
}

SimSnapshot SimWorker::snapshot() const {
  std::lock_guard<std::mutex> lk(snap_mtx_);
  return last_snap_;
}

void SimWorker::openPwmFifo() {
  // Match host_navhal.c: O_RDWR|O_NONBLOCK so we don't block waiting
  // for a writer and so the FIFO stays open if vayu_sitl restarts.
  struct stat st;
  if (::stat(pwm_fifo_path_.toLocal8Bit().constData(), &st) != 0) {
    if (::mkfifo(pwm_fifo_path_.toLocal8Bit().constData(), 0666) != 0
        && errno != EEXIST) {
      emit logLine(QString("vsim: mkfifo %1 failed: %2")
                       .arg(pwm_fifo_path_, ::strerror(errno)));
      return;
    }
  }
  pwm_fd_ = ::open(pwm_fifo_path_.toLocal8Bit().constData(),
                   O_RDWR | O_NONBLOCK);
  if (pwm_fd_ < 0) {
    emit logLine(QString("vsim: open %1 failed: %2")
                     .arg(pwm_fifo_path_, ::strerror(errno)));
    return;
  }
  emit logLine(QString("vsim: PWM FIFO open: %1").arg(pwm_fifo_path_));
}

void SimWorker::openImuFifo() {
  // Match host_imu_feeder.c: ensure FIFO exists, open O_WRONLY. The
  // open BLOCKS until the firmware-side reader connects, but the
  // firmware's feeder thread also blocks on open(O_RDONLY), so we
  // briefly poll instead of taking the blocking path here -- otherwise
  // SimWorker.start() hangs the GUI thread before the user even gets
  // a chance to launch the firmware.
  struct stat st;
  if (::stat(imu_fifo_path_.toLocal8Bit().constData(), &st) != 0) {
    if (::mkfifo(imu_fifo_path_.toLocal8Bit().constData(), 0666) != 0
        && errno != EEXIST) {
      emit logLine(QString("vsim: mkfifo %1 failed: %2")
                       .arg(imu_fifo_path_, ::strerror(errno)));
      return;
    }
  }
  // Open in O_RDWR so it never blocks; the reader on the other side
  // doesn't care which mode we're in.
  imu_fd_ = ::open(imu_fifo_path_.toLocal8Bit().constData(),
                   O_RDWR | O_NONBLOCK);
  if (imu_fd_ < 0) {
    emit logLine(QString("vsim: open %1 failed: %2")
                     .arg(imu_fifo_path_, ::strerror(errno)));
    return;
  }
  emit logLine(QString("vsim: IMU FIFO open: %1").arg(imu_fifo_path_));
}

// Read whatever's available on the PWM FIFO right now, split by
// newlines, parse "idx duty" lines, latch the latest duty for each
// motor into the atomics. Tolerates partial reads via pwm_residual_.
void SimWorker::drainPwmFifo() {
  // In-process path: just pull the latest from the shared iface.
  if (iface_) {
    float d[4];
    vsim_iface_get_motor_duty(iface_, d);
    for (int i = 0; i < 4; ++i) {
      motor_duty_a_[i].store(d[i], std::memory_order_relaxed);
    }
    return;
  }

  if (pwm_fd_ < 0) return;
  char buf[512];
  while (true) {
    ssize_t r = ::read(pwm_fd_, buf, sizeof(buf));
    if (r <= 0) break;       // EAGAIN or EOF (we hold O_RDWR so EOF unlikely)
    pwm_residual_.append(buf, static_cast<size_t>(r));
  }
  size_t nl;
  while ((nl = pwm_residual_.find('\n')) != std::string::npos) {
    std::string line = pwm_residual_.substr(0, nl);
    pwm_residual_.erase(0, nl + 1);
    int idx;
    float duty;
    if (std::sscanf(line.c_str(), "%d %f", &idx, &duty) == 2) {
      if (idx >= 0 && idx < 4) {
        motor_duty_a_[idx].store(duty, std::memory_order_relaxed);
      }
    }
  }
}

// Pack one ImuSample into the 76-byte wire format the firmware expects
// (bmx160_all_converted_reading_t). Gyro is converted from rad/s to dps.
// acc_raw / gyr_raw / mag_compensated are filled with the same data the
// firmware sees pre-calibration; they're not actually read by the mahony
// path but the field needs to be present for the struct to be the right
// size.
void SimWorker::writeImuSample(const ImuSample& s) {
  float frame[19];
  // calibrated triplets
  frame[0] = s.acc.x();  frame[1] = s.acc.y();  frame[2] = s.acc.z();
  frame[3] = s.gyr.x() * kRad2Deg;
  frame[4] = s.gyr.y() * kRad2Deg;
  frame[5] = s.gyr.z() * kRad2Deg;
  frame[6] = s.mag.x();  frame[7] = s.mag.y();  frame[8] = s.mag.z();
  // raw triplets (mirror calibrated; firmware doesn't differentiate
  // when the sample comes via the bridge instead of the I2C driver).
  frame[9]  = frame[0]; frame[10] = frame[1]; frame[11] = frame[2];
  frame[12] = frame[3]; frame[13] = frame[4]; frame[14] = frame[5];
  frame[15] = frame[6]; frame[16] = frame[7]; frame[17] = frame[8];
  frame[18] = s.temp;
  static_assert(sizeof(frame) == kImuFrameBytes,
                "vsim IMU frame must match bmx160 struct on the wire");

  // In-process path: push into the shared iface; the firmware's
  // IMU feeder thread is cond-waiting on this.
  if (iface_) {
    vsim_iface_post_imu(iface_, reinterpret_cast<const uint8_t*>(frame));
    return;
  }

  if (imu_fd_ < 0) return;
  ssize_t w = ::write(imu_fd_, frame, kImuFrameBytes);
  if (w < 0) {
    if (errno == EAGAIN) {
      // No reader connected yet; drop this sample silently.
      return;
    }
    emit logLine(QString("vsim: IMU write failed: %1").arg(::strerror(errno)));
  }
}

void SimWorker::publishSnapshot(uint64_t tick) {
  SimSnapshot snap;
  const auto& st = ctl_.state();
  snap.pos_w   = st.pos_w;
  snap.att     = st.att;
  snap.vel_w   = st.vel_w;
  snap.omega_b = st.omega_b;
  snap.motor_omega = ctl_.motorOmegas();
  for (int i = 0; i < 4; ++i) {
    snap.motor_duty[i] = motor_duty_a_[i].load(std::memory_order_relaxed);
  }
  snap.tick_count = tick;
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    last_snap_ = snap;
  }
  emit poseUpdated(snap);
}

void SimWorker::run() {
  using clock = std::chrono::steady_clock;
  const auto dt_dur = std::chrono::microseconds(1000000 / kPhysicsHz);
  const float dt = 1.0f / static_cast<float>(kPhysicsHz);

  // In-process path skips the FIFOs entirely.
  if (!iface_) {
    openPwmFifo();
    openImuFifo();
  } else {
    emit logLine("vsim: in-process iface mode (no FIFOs)");
  }

  // Initial state: airframe at z = a few cm above ground, level.
  // (NED: spawn at z = -0.05 so first step has the drone hovering
  // just above the ground plane.)
  RigidBodyState s0;
  s0.pos_w = Vec3(0.0f, 0.0f, -0.05f);
  ctl_.resetState(s0);

  emit logLine(QString("vsim: starting (%1 Hz physics, %2 Hz IMU)")
                   .arg(kPhysicsHz).arg(kImuHz));

  auto next = clock::now();
  uint64_t tick = 0;
  while (!stop_flag_.load(std::memory_order_acquire)) {
    drainPwmFifo();
    std::array<float, 4> duty;
    for (int i = 0; i < 4; ++i) {
      duty[i] = std::clamp(motor_duty_a_[i].load(std::memory_order_relaxed),
                           0.0f, 1.0f);
    }

    ImuSample s = ctl_.tick(duty, dt);

    if (tick % kImuDiv == 0) writeImuSample(s);
    if (tick % kRenderDiv == 0) publishSnapshot(tick);

    ++tick;
    next += dt_dur;
    auto now = clock::now();
    if (next > now) {
      std::this_thread::sleep_until(next);
    } else {
      // Fell behind; resync the next-deadline so we don't try to
      // catch up by busy-spinning.
      next = now;
    }
  }

  emit logLine("vsim: stopping");
  emit stoppedCleanly();
}

}  // namespace vsim
