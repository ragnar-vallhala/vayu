#pragma once

#include "SimController.h"
#include "VsimTypes.h"

extern "C" {
#include "vsim_iface.h"
}

#include <QObject>
#include <QString>
#include <QThread>
#include <array>
#include <atomic>
#include <mutex>

namespace vsim {

// Snapshot of "what the renderer needs to draw the drone". Sent via
// Qt::QueuedConnection signal from the sim thread to the GUI thread
// at ~60 Hz; intentionally small + trivially copyable.
struct SimSnapshot {
  Vec3  pos_w   = {0, 0, 0};
  Quat  att     = Quat(1, 0, 0, 0);
  Vec3  vel_w   = {0, 0, 0};
  Vec3  omega_b = {0, 0, 0};
  std::array<float, 4> motor_omega = {0, 0, 0, 0};
  std::array<float, 4> motor_duty  = {0, 0, 0, 0};
  uint64_t tick_count = 0;
};

// SimWorker drives SimController in real time on its own thread.
//
// IO is via the existing two FIFOs the firmware already understands:
//   - reads "motor_idx duty\n" ASCII lines from /tmp/vayu_pwm.fifo
//   - writes a 76-byte bmx160_all_converted_reading_t to /tmp/vayu_imu.fifo
//     at 200 Hz
// This means the standalone vayu_sitl binary still works unchanged.
// Once the library refactor lands the FIFO calls can be swapped for
// direct in-process callbacks without touching the rest of the class.
//
// The worker subclasses QThread (not QObject-on-a-thread) because
// we want a tight integration loop, not Qt's event loop, at 1 kHz.
class SimWorker : public QThread {
  Q_OBJECT
 public:
  explicit SimWorker(QObject* parent = nullptr);
  ~SimWorker() override;

  // Configuration (call before start()).
  void setDroneParams(const DroneParams& p) { ctl_.setDroneParams(p); }
  void setMotorParams(const MotorParams& p) { ctl_.setMotorParams(p); }
  void setNoise(const SensorNoise& n)       { ctl_.setNoise(n); }

  void setPwmFifoPath(const QString& p)     { pwm_fifo_path_ = p; }
  void setImuFifoPath(const QString& p)     { imu_fifo_path_ = p; }

  // In-process iface (set before start). When non-null the worker
  // talks through shared memory instead of the two /tmp FIFOs.
  void setIface(vsim_iface_t* iface)        { iface_ = iface; }

  // Snapshot getter for renderers / panels that pull instead of subscribe.
  SimSnapshot snapshot() const;

  // Tell the worker to shut down (also called from destructor).
  void requestStop();

 signals:
  void poseUpdated(SimSnapshot snap);
  void logLine(QString line);   // status / error messages
  void stoppedCleanly();

 protected:
  void run() override;

 private:
  void openPwmFifo();
  void openImuFifo();
  void drainPwmFifo();
  void writeImuSample(const ImuSample& s);
  void publishSnapshot(uint64_t tick);

  SimController ctl_;

  QString pwm_fifo_path_ = "/tmp/vayu_pwm.fifo";
  QString imu_fifo_path_ = "/tmp/vayu_imu.fifo";
  int pwm_fd_ = -1;
  int imu_fd_ = -1;

  // Optional in-process iface for talking to libvayu_sitl_core directly.
  // When set, PWM is pulled via vsim_iface_get_motor_duty and IMU is
  // pushed via vsim_iface_post_imu; the FIFO paths are bypassed.
  vsim_iface_t* iface_ = nullptr;

  // Latest motor duty (shared with the GL renderer for display).
  std::atomic<float> motor_duty_a_[4] = {{0.0f}, {0.0f}, {0.0f}, {0.0f}};

  // Pose latch for snapshot() pull-style readers.
  mutable std::mutex snap_mtx_;
  SimSnapshot last_snap_;

  std::atomic<bool> stop_flag_{false};

  // Partial line buffer for the PWM FIFO reader. Firmware emits
  // "%d %f\n" which we have to reassemble across read() boundaries.
  std::string pwm_residual_;
};

}  // namespace vsim

Q_DECLARE_METATYPE(vsim::SimSnapshot)
