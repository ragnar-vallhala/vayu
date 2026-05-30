#pragma once

#include "VsimTypes.h"

#include <QObject>
#include <QString>
#include <QThread>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace vsim {

// Snapshot of "what the renderer needs to draw the drone". Decoded from
// a vsim_pose_frame_t coming off /tmp/vsim_pose; sent to the GUI thread
// via Qt::QueuedConnection at ~60 Hz; intentionally small + trivially
// copyable.
//
// Types here are still QVector3D / QQuaternion (aliased in VsimTypes.h)
// so SimRendererWidget keeps consuming this struct unchanged.
struct SimSnapshot {
  Vec3  pos_w   = {0, 0, 0};
  Quat  att     = Quat(1, 0, 0, 0);
  Vec3  vel_w   = {0, 0, 0};
  Vec3  omega_b = {0, 0, 0};
  std::array<float, 4> motor_omega = {0, 0, 0, 0};
  std::array<float, 4> motor_duty  = {0, 0, 0, 0};
  uint64_t tick_count = 0;
};

// SimWorker -- vsim_d process supervisor + pose reader.
//
// One-shot lifecycle:
//   1. construct on the GUI thread.
//   2. (optional) setVsimBinary("...path/to/vsim_d") if the daemon
//      isn't in $PATH and isn't beside the Navigator binary.
//   3. start() -- worker thread spawns vsim_d via posix_spawnp, opens
//      /tmp/vsim_pose for reading + /tmp/vsim_ctl for writing, then
//      pumps pose frames until requestStop().
//   4. requestStop() -- worker sends SIGTERM to vsim_d and exits the
//      read loop. wait() joins as usual.
//
// All physics + sensor simulation lives in the vsim_d binary now.
// SimWorker no longer holds a SimController, no longer pushes IMU,
// no longer pulls PWM -- those happen between vsim_d and the firmware
// (whether the firmware is the standalone vayu_sitl binary or living
// inside Navigator). SimWorker exists only to (a) own the daemon's
// lifetime relative to the SimulatorWidget page, (b) decode pose
// frames for the renderer, and (c) ferry control messages back.
class SimWorker : public QThread {
  Q_OBJECT
 public:
  explicit SimWorker(QObject* parent = nullptr);
  ~SimWorker() override;

  // Path to the vsim_d binary. Default is whatever VSIM_BIN_PATH env
  // var points at, falling back to "vsim_d" (PATH lookup).
  void setVsimBinary(const QString& path) { vsim_bin_ = path; }

  // Snapshot getter for pull-style consumers.
  SimSnapshot snapshot() const;

  // Tell the worker to shut down (also called from destructor). Kills
  // the spawned vsim_d via SIGTERM.
  void requestStop();

  // Send a CTL_RESET to vsim_d. With no args, re-spawns at level pose
  // slightly above ground (matches the daemon's own initial state).
  void sendReset();

  // Push mass properties + motor layout (VSIM_CTL_SET_GEOMETRY). The
  // mesh/scale/com fields of `g` stay GCS-side; only mass, the inertia
  // tensor, and the 4-motor params cross the wire.
  void sendGeometry(const GeometryConfig& g);

 signals:
  void poseUpdated(SimSnapshot snap);
  void logLine(QString line);
  void stoppedCleanly();
  // Emitted once the daemon is spawned and the ctl/pose FIFOs are open,
  // so callers can safely push initial control (e.g. geometry).
  void online();

 protected:
  void run() override;

 private:
  bool spawnDaemon();
  void killDaemon();
  bool openFifos();
  void closeFifos();
  void emitFromFrame(const void* frame_bytes);

  QString vsim_bin_;

  // pid of the spawned vsim_d process; -1 if not running.
  int daemon_pid_ = -1;

  int pose_fd_ = -1;
  int ctl_fd_  = -1;

  std::atomic<bool> stop_flag_{false};

  // Pose latch for snapshot() pull-style readers.
  mutable std::mutex snap_mtx_;
  SimSnapshot last_snap_;
};

}  // namespace vsim

Q_DECLARE_METATYPE(vsim::SimSnapshot)
