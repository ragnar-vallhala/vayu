#pragma once

#include "VsimTypes.h"

#include <QObject>
#include <QString>
#include <QThread>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

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
  Vec3  wind_w  = {0, 0, 0};  // instantaneous world wind [m/s] NED
  float airspeed     = 0.0f;  // ‖v_rel‖ [m/s]
  float ge_factor    = 1.0f;  // ground-effect multiplier
  float batt_voltage = 0.0f;  // [V]
  float batt_current = 0.0f;  // [A]
  float batt_mah_used= 0.0f;  // [mAh]
  float batt_soc     = 0.0f;  // [0,1]
};

// SimWorker -- in-process RTOS SITL engine driver (+ legacy FIFO attach).
//
// Default (engine) lifecycle:
//   1. construct on the GUI thread; setIface(&iface) so firmware telemetry
//      reaches the GUI via the in-process UART2 callback.
//   2. start() -- worker thread runs rtos_engine_boot(iface) (boots the REAL
//      vaios firmware + in-process vsim physics, no vsim_d, no FIFO), enables
//      the serial RC feeder (RcBridge pty / remote transmitter), then loops:
//      drain queued config -> rtos_engine_run_step (1 ms, wall-clock paced) ->
//      vsim_inproc_get_pose -> poseUpdated (~60 Hz), until requestStop().
//   3. requestStop() -- sets the stop flag; the worker exits the loop. The
//      firmware/scheduler stay booted (idempotent), so Re-Start just resumes.
//
// The engine runs firmware AND physics in one deterministic stepper. sendX()
// (GUI thread) enqueues a config op applied on the worker thread between steps
// via the vsim_inproc_set_* surface (the engine is single-threaded).
//
// startAttach() keeps the LEGACY behaviour: read an existing pose FIFO (e.g. the
// autotuner's own vsim_d) and emit poseUpdated, mirroring a sim we don't own.
// No engine, no ctl; sendX() are no-ops in attach mode.
class SimWorker : public QThread {
  Q_OBJECT
 public:
  explicit SimWorker(QObject* parent = nullptr);
  ~SimWorker() override;

  // Retained for source compatibility; the engine runs in-process, so there is
  // no daemon binary to resolve. No-op.
  void setVsimBinary(const QString& path) { vsim_bin_ = path; }

  // The vsim_iface_t* whose UART2 callback receives firmware telemetry in
  // process. Set before start() so rtos_engine_boot wires telemetry to the GUI.
  void setIface(void* iface) { iface_ = iface; }

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
  void runEngine();               // default: drive the in-process RTOS engine
  void runAttach();               // legacy: read an existing pose FIFO
  void emitFromFrame(const void* frame_bytes);
  // Queue a config op to run on the worker thread between steps (no-op in
  // attach mode). The closure typically calls a vsim_inproc_set_* function.
  void enqueue(std::function<void()> op);

  QString vsim_bin_;              // retained, unused (engine is in-process)
  void*   iface_ = nullptr;       // vsim_iface_t* for in-process telemetry
  bool    attach_only_ = false;   // read an existing pose FIFO, don't run engine
  QString attach_pose_path_;      // pose FIFO to attach to in attach-only mode

  int pose_fd_ = -1;              // attach-only: the pose FIFO read fd

  std::atomic<bool> stop_flag_{false};

  // Config ops from the GUI thread, drained + applied on the worker thread.
  std::mutex cmd_mtx_;
  std::vector<std::function<void()>> cmds_;

  // Pose latch for snapshot() pull-style readers.
  mutable std::mutex snap_mtx_;
  SimSnapshot last_snap_;
};

}  // namespace vsim

Q_DECLARE_METATYPE(vsim::SimSnapshot)
