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
  // sim-fidelity telemetry (pose proto v3); zero until each phase populates it.
  Vec3  wind_w  = {0, 0, 0};  // instantaneous world wind [m/s] NED (Phase 1)
  float airspeed     = 0.0f;  // ‖v_rel‖ [m/s]                    (Phase 2)
  float ge_factor    = 1.0f;  // ground-effect multiplier         (Phase 2)
  float batt_voltage = 0.0f;  // [V]                              (Phase 5)
  float batt_current = 0.0f;  // [A]                              (Phase 5)
  float batt_mah_used= 0.0f;  // [mAh]                            (Phase 5)
  float batt_soc     = 0.0f;  // [0,1]                            (Phase 5)
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

  // Attach-only mode: instead of spawning a daemon, read an EXISTING pose FIFO
  // (e.g. the autotuner's own vsim_d) and emit poseUpdated, so the GCS renderer
  // can mirror a sim it doesn't own. No ctl channel; sendX() are no-ops.
  void startAttach(const QString& posePath);

  // Tell the worker to shut down (also called from destructor). Kills
  // the spawned vsim_d via SIGTERM.
  void requestStop();

  // Send a CTL_RESET to vsim_d. With no args, re-spawns at level pose
  // slightly above ground (matches the daemon's own initial state).
  void sendReset();
  // Re-spawn at a specific NED world position, level attitude, zero velocity.
  // Used to lift the drone onto the terrain surface when a world loads.
  void sendResetPose(float x, float y, float z);

  // Test-rig mode (VSIM_CTL_SET_TESTRIG): pin translation, leave rotation free —
  // a frictionless attitude gimbal. Pair with sendRigPose to pose the airframe.
  void sendTestRig(bool on);

  // Re-spawn at a specific orientation (NED roll/pitch/yaw in degrees). Used with
  // the test rig to tilt the vehicle on the stand for inspection.
  void sendRigPose(float rollDeg, float pitchDeg, float yawDeg);

  // Push mass properties + motor layout (VSIM_CTL_SET_GEOMETRY). The
  // mesh/scale/com fields of `g` stay GCS-side; only mass, the inertia
  // tensor, and the 4-motor params cross the wire.
  void sendGeometry(const GeometryConfig& g);

  // Push environment + aerodynamics (VSIM_CTL_SET_WORLD).
  void sendWorld(const WorldConfig& w);

  // Replace the daemon's obstacle set: a CLEAR frame then one ADD per shape
  // (VSIM_CTL_CLEAR_OBSTACLES / VSIM_CTL_ADD_OBSTACLE).
  void sendObstacles(const QVector<Obstacle>& obs);

  // Set sim loop rates (VSIM_CTL_SET_RATES). imuHz = firmware loop rate;
  // physicsHz/imuHz RK4 substeps run per sample; poseHz = render rate.
  void sendRates(int imuHz, int physicsHz, int poseHz);

  // Point the daemon at a serialized BVH world-mesh file to mmap, or clear it
  // (VSIM_CTL_SET_WORLD_MESH / VSIM_CTL_CLEAR_WORLD_MESH).
  void sendWorldMesh(const QString& path, quint32 verts, quint32 tris,
                     quint32 nodes, float restitution, bool doubleSided);
  void clearWorldMesh();

  // Inject/clear failures (VSIM_CTL_SET_FAULTS): per-rotor kill + IMU dropout.
  void sendFaults(const std::array<bool, 4>& motorKill, bool imuDropout);

  // Push per-sensor noise model + enable (VSIM_CTL_SET_NOISE). enable=false
  // drops that sensor's feed (channel zeroed).
  void sendNoise(float accSigma, float accBiasClip, bool accEn,
                 float gyrSigma, float gyrBiasClip, bool gyrEn,
                 float magSigma, float magBiasClip, bool magEn);

  // Push the world wind field (VSIM_CTL_SET_WIND): steady + gust + turbulence.
  void sendWind(const WindConfig& w);

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
  bool    attach_only_ = false;   // read an existing pose FIFO, don't spawn
  QString attach_pose_path_;      // pose FIFO to attach to in attach-only mode

  // pid of the spawned vsim_d process; -1 if not running. Guarded by
  // daemon_mtx_ because killDaemon() runs from both the GUI thread
  // (requestStop) and the worker thread (end of run() / dtor).
  int daemon_pid_ = -1;
  std::mutex daemon_mtx_;

  int pose_fd_ = -1;
  int ctl_fd_  = -1;

  std::atomic<bool> stop_flag_{false};

  // Pose latch for snapshot() pull-style readers.
  mutable std::mutex snap_mtx_;
  SimSnapshot last_snap_;
};

}  // namespace vsim

Q_DECLARE_METATYPE(vsim::SimSnapshot)
