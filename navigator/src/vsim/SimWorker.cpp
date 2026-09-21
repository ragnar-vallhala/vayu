#include "SimWorker.h"

#include "SitlModule.h" // runtime-loaded SITL engine (boot/run_step/get_pose/config)
#include "vsim_proto.h"

#include <QByteArray>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace {
// The loaded engine's vtable. Resolved per call rather than captured into each
// queued lambda: the module is loaded once and never unloaded, so this is a
// pointer read, and it keeps the enqueue sites free of loader plumbing.
// Only ever dereferenced on paths runEngine() has already gated on
// SitlModule::available().
inline const vayu_sitl_api_t *sitl() { return SitlModule::instance().api(); }
} // namespace

namespace vsim {

SimWorker::SimWorker(QObject *parent) : QThread(parent) {
  qRegisterMetaType<SimSnapshot>("vsim::SimSnapshot");
}

SimWorker::~SimWorker() {
  requestStop();
  if (isRunning())
    wait(2000);
  if (pose_fd_ >= 0) {
    ::close(pose_fd_);
    pose_fd_ = -1;
  }
}

void SimWorker::requestStop() {
  // The engine stays booted (idempotent); the worker just exits its loop.
  stop_flag_.store(true, std::memory_order_release);
}

void SimWorker::enqueue(std::function<void()> op) {
  if (attach_only_)
    return; // mirroring someone else's sim — no ctl channel
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmds_.push_back(std::move(op));
}

SimSnapshot SimWorker::snapshot() const {
  std::lock_guard<std::mutex> lk(snap_mtx_);
  return last_snap_;
}

void SimWorker::sendReset() { sendResetPose(0.0f, 0.0f, -0.05f); }

void SimWorker::sendResetPose(float x, float y, float z) {
  vsim_ctl_reset_t body{};
  body.pos_w[0] = x;
  body.pos_w[1] = y;
  body.pos_w[2] = z;        // NED: more negative = higher above ground
  body.quat_wxyz[0] = 1.0f; // identity (level)
  enqueue([body] { sitl()->reset_to(&body); });
}

void SimWorker::sendTestRig(bool on) {
  vsim_ctl_testrig_t body{};
  body.enable = on ? 1 : 0;
  body.pos[2] = -0.05f;
  enqueue([body] { sitl()->set_testrig(&body); });
  emit logLine(
      QStringLiteral("rtos engine: test-rig %1").arg(on ? "ON" : "off"));
}

void SimWorker::sendRigPose(float rollDeg, float pitchDeg, float yawDeg) {
  // NED ZYX (yaw-pitch-roll) Euler -> body->world quaternion: the exact
  // inverse of quatToEulerNED() in VsimTypes.h.
  const double h = 0.017453292519943295 * 0.5; // deg -> half-radians
  const double cr = std::cos(rollDeg * h), sr = std::sin(rollDeg * h);
  const double cp = std::cos(pitchDeg * h), sp = std::sin(pitchDeg * h);
  const double cy = std::cos(yawDeg * h), sy = std::sin(yawDeg * h);
  vsim_ctl_reset_t body{};
  body.pos_w[2] = -0.05f;
  body.quat_wxyz[0] = static_cast<float>(cr * cp * cy + sr * sp * sy);
  body.quat_wxyz[1] = static_cast<float>(sr * cp * cy - cr * sp * sy);
  body.quat_wxyz[2] = static_cast<float>(cr * sp * cy + sr * cp * sy);
  body.quat_wxyz[3] = static_cast<float>(cr * cp * sy - sr * sp * cy);
  enqueue([body] { sitl()->reset_to(&body); });
  emit logLine(QStringLiteral("rig pose sent: r=%1 p=%2 y=%3")
                   .arg(rollDeg, 0, 'f', 0)
                   .arg(pitchDeg, 0, 'f', 0)
                   .arg(yawDeg, 0, 'f', 0));
}

void SimWorker::sendGeometry(const GeometryConfig &g) {
  vsim_ctl_geometry_t body{};
  body.mass = g.mass;
  for (int i = 0; i < 9; ++i)
    body.inertia[i] = g.inertia[i];
  for (int i = 0; i < 4; ++i) {
    const auto &m = g.motors[i];
    body.motors[i].pos[0] = m.pos.x();
    body.motors[i].pos[1] = m.pos.y();
    body.motors[i].pos[2] = m.pos.z();
    body.motors[i].axis[0] = m.axis.x();
    body.motors[i].axis[1] = m.axis.y();
    body.motors[i].axis[2] = m.axis.z();
    body.motors[i].spin = static_cast<float>(m.spin);
    body.motors[i].k_thrust = m.k_thrust;
    body.motors[i].k_moment = m.k_moment;
    body.motors[i].max_omega = m.max_omega;
    body.motors[i].tau = m.tau;
  }
  enqueue([body] { sitl()->set_geometry(&body); });
  emit logLine("rtos engine: geometry pushed");
}

void SimWorker::sendFaults(const std::array<bool, 4> &motorKill,
                           bool imuDropout) {
  vsim_ctl_faults_t body{};
  for (int i = 0; i < 4; ++i)
    body.motor_kill[i] = motorKill[i] ? 1 : 0;
  body.imu_dropout = imuDropout ? 1 : 0;
  enqueue([body] { sitl()->set_faults(&body); });
  emit logLine("rtos engine: faults pushed");
}

void SimWorker::sendNoise(float accSigma, float accBiasClip, bool accEn,
                          float gyrSigma, float gyrBiasClip, bool gyrEn,
                          float magSigma, float magBiasClip, bool magEn) {
  vsim_ctl_noise_t body{};
  body.acc_sigma = accSigma;
  body.acc_bias_clip = accBiasClip;
  body.acc_enable = accEn;
  body.gyr_sigma = gyrSigma;
  body.gyr_bias_clip = gyrBiasClip;
  body.gyr_enable = gyrEn;
  body.mag_sigma = magSigma;
  body.mag_bias_clip = magBiasClip;
  body.mag_enable = magEn;
  enqueue([body] { sitl()->set_noise(&body); });
  emit logLine("rtos engine: noise pushed");
}

void SimWorker::sendWind(const WindConfig &w) {
  vsim_ctl_wind_t body{};
  body.steady[0] = w.steady.x();
  body.steady[1] = w.steady.y();
  body.steady[2] = w.steady.z();
  body.gust_amp = w.gustAmp;
  body.gust_period = w.gustPeriod;
  body.turb_sigma = w.turbSigma;
  body.turb_tau = w.turbTau;
  body.enable = w.enabled ? 1 : 0;
  enqueue([body] { sitl()->set_wind(&body); });
  emit logLine("rtos engine: wind pushed");
}

void SimWorker::sendWorld(const WorldConfig &w) {
  vsim_ctl_world_t body{};
  body.gravity = w.gravity;
  body.ground_z = w.ground_z;
  body.restitution = w.restitution;
  body.linear_drag = w.linear_drag;
  body.angular_drag = w.angular_drag;
  body.ground_right_gain = w.ground_right_gain;
  body.ground_right_damp = w.ground_right_damp;
  enqueue([body] { sitl()->set_world(&body); });
  emit logLine("rtos engine: world pushed");
}

void SimWorker::sendObstacles(const QVector<Obstacle> &obs) {
  // Clear, then append each shape — as separate queued ops so they apply in
  // order on the worker thread (CLEAR before each ADD).
  enqueue([] { sitl()->clear_obstacles(); });
  for (const Obstacle &o : obs) {
    vsim_ctl_obstacle_t b{};
    b.type = o.type;
    b.pos[0] = o.pos.x();
    b.pos[1] = o.pos.y();
    b.pos[2] = o.pos.z();
    b.size[0] = o.size.x();
    b.size[1] = o.size.y();
    b.size[2] = o.size.z();
    b.rot_deg[0] = o.rotate.x();
    b.rot_deg[1] = o.rotate.y();
    b.rot_deg[2] = o.rotate.z();
    b.restitution = o.restitution;
    enqueue([b] { sitl()->add_obstacle(&b); });
  }
  emit logLine(QString("rtos engine: %1 obstacles pushed").arg(obs.size()));
}

void SimWorker::sendRates(int imuHz, int physicsHz, int poseHz) {
  vsim_ctl_rates_t b{};
  b.imu_hz = static_cast<uint32_t>(imuHz);
  b.physics_hz = static_cast<uint32_t>(physicsHz);
  b.pose_hz = static_cast<uint32_t>(poseHz);
  enqueue([b] { sitl()->set_rates(&b); });
  emit logLine(QString("rtos engine: rates imu=%1 physics=%2 pose=%3 Hz")
                   .arg(imuHz)
                   .arg(physicsHz)
                   .arg(poseHz));
}

void SimWorker::sendWorldMesh(const QString &path, quint32 verts, quint32 tris,
                              quint32 nodes, float restitution,
                              bool doubleSided) {
  const QByteArray pb = path.toLocal8Bit();
  vsim_ctl_world_mesh_t b{};
  if (pb.size() >= int(sizeof(b.path))) {
    emit logLine("rtos engine: world-mesh path too long");
    return;
  }
  b.vertex_count = verts;
  b.triangle_count = tris;
  b.node_count = nodes;
  b.flags = doubleSided ? 1u : 0u;
  b.restitution = restitution;
  b.path_len = static_cast<uint32_t>(pb.size());
  std::memcpy(b.path, pb.constData(), pb.size());
  enqueue([b] { sitl()->set_world_mesh(&b); });
  emit logLine(
      QString("rtos engine: world mesh -> %1 (%2 tris)").arg(path).arg(tris));
}

void SimWorker::clearWorldMesh() {
  enqueue([] { sitl()->clear_world_mesh(); });
  emit logLine("rtos engine: world mesh cleared");
}

// Drive the in-process RTOS engine: boot the firmware + physics, enable the
// serial RC feeder, then loop applying queued config, stepping (1 ms paced),
// and publishing pose at ~60 Hz. The engine stays booted across Stop/Re-Start
// (boot is idempotent), so this just resumes the loop.
void SimWorker::runEngine() {
  // The module is loaded lazily, so this is where a missing or ABI-mismatched
  // firmware build surfaces. Report the loader's reason rather than a bare
  // "boot failed": missing file and version mismatch have different fixes.
  if (!SitlModule::instance().available()) {
    emit logLine("rtos engine: " + SitlModule::instance().error());
    emit stoppedCleanly();
    return;
  }
  if (sitl()->boot(telemetry_cb_, telemetry_user_) != 0) {
    emit logLine("rtos engine: boot failed");
    emit stoppedCleanly();
    return;
  }
  emit logLine("rtos engine: loaded " + SitlModule::instance().path() + " [" +
               SitlModule::instance().buildId() + "]");
  sitl()->enable_serial_rc(); // RC from RcBridge pty / remote transmitter
  sitl()->run_begin();
  emit logLine("rtos engine: online");
  emit online();

  vsim_pose_frame_t pose;
  int n = 0;
  while (!stop_flag_.load(std::memory_order_acquire)) {
    // Apply config queued from the GUI thread (engine is single-threaded).
    {
      std::vector<std::function<void()>> batch;
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        batch.swap(cmds_);
      }
      for (auto &op : batch)
        op();
    }
    sitl()->run_step();    // one 1 ms step, wall-clock paced
    if ((++n & 15) == 0) { // ~62.5 Hz to the renderer
      sitl()->get_pose(&pose);
      emitFromFrame(&pose);
    }
  }
  emit logLine("rtos engine: stopped");
  emit stoppedCleanly();
}

void SimWorker::emitFromFrame(const void *bytes) {
  const auto *frame = reinterpret_cast<const vsim_pose_frame_t *>(bytes);
  SimSnapshot snap;
  snap.pos_w = Vec3(frame->pos_w[0], frame->pos_w[1], frame->pos_w[2]);
  snap.att = Quat(frame->quat_wxyz[0], frame->quat_wxyz[1], frame->quat_wxyz[2],
                  frame->quat_wxyz[3]);
  snap.vel_w = Vec3(frame->vel_w[0], frame->vel_w[1], frame->vel_w[2]);
  snap.omega_b = Vec3(frame->omega_b[0], frame->omega_b[1], frame->omega_b[2]);
  for (int i = 0; i < 4; ++i) {
    snap.motor_omega[i] = frame->motor_omega[i];
    snap.motor_duty[i] = frame->motor_duty[i];
  }
  snap.tick_count =
      (static_cast<uint64_t>(frame->tick_hi) << 32) | frame->tick_lo;
  snap.wind_w = Vec3(frame->wind_w[0], frame->wind_w[1], frame->wind_w[2]);
  snap.airspeed = frame->airspeed;
  snap.ge_factor = frame->ge_factor;
  snap.batt_voltage = frame->batt_voltage;
  snap.batt_current = frame->batt_current;
  snap.batt_mah_used = frame->batt_mah_used;
  snap.batt_soc = frame->batt_soc;
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    last_snap_ = snap;
  }
  emit poseUpdated(snap);
}

void SimWorker::startAttach(const QString &posePath) {
  attach_only_ = true;
  attach_pose_path_ = posePath;
  start();
}

void SimWorker::run() {
  stop_flag_.store(false, std::memory_order_release);
  if (attach_only_)
    runAttach(); // mirror an external sim's pose FIFO
  else
    runEngine(); // drive the in-process RTOS engine
}

// Viewer-only mode: read an EXISTING pose FIFO published by an external sim
// (e.g. a headless vayu_sitl_rtos run) and emit poseUpdated.
void SimWorker::runAttach() {
  // Wait briefly for the file to appear (the external sim may publish it a
  // moment after launch).
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(20);
  const QByteArray p = attach_pose_path_.toLocal8Bit();
  while (!stop_flag_.load(std::memory_order_acquire)) {
    pose_fd_ = ::open(p.constData(), O_RDONLY | O_NONBLOCK);
    if (pose_fd_ >= 0)
      break;
    if (std::chrono::steady_clock::now() > deadline) {
      emit logLine("attach: external pose FIFO never appeared");
      emit stoppedCleanly();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (pose_fd_ < 0) {
    emit stoppedCleanly();
    return;
  }
  emit logLine("attached to external sim: " + attach_pose_path_);
  emit online();

  // Magic-resync buffer: walk to the next frame boundary on a mid-stream
  // connect / short read. Tolerate startup EOF (writer-less FIFO returns 0)
  // until the producer connects, bounded by a grace window.
  std::string buf;
  bool producer_seen = false;
  const auto startup_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!stop_flag_.load(std::memory_order_acquire)) {
    char chunk[2048];
    ssize_t r = ::read(pose_fd_, chunk, sizeof(chunk));
    if (r > 0) {
      producer_seen = true;
      buf.append(chunk, static_cast<size_t>(r));
    } else if (r == 0) {
      if (producer_seen)
        break; // producer connected, then went away
      if (std::chrono::steady_clock::now() > startup_deadline) {
        emit logLine("attach: no pose producer within startup grace");
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } else if (r < 0 && errno != EAGAIN) {
      break; // hard error
    } else {
      producer_seen = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }

    // Emit only the LATEST complete pose frame per iteration (latest-wins).
    size_t pos = 0;
    const void *last_good = nullptr;
    while (pos + sizeof(vsim_hdr_t) <= buf.size()) {
      const auto *hdr = reinterpret_cast<const vsim_hdr_t *>(buf.data() + pos);
      if (hdr->magic != VSIM_MAGIC) {
        ++pos;
        continue;
      }
      const size_t need = sizeof(vsim_hdr_t) + hdr->payload_bytes;
      if (pos + need > buf.size())
        break; // incomplete tail
      if (hdr->version == VSIM_PROTO_VERSION && hdr->type == VSIM_FRAME_POSE &&
          need == sizeof(vsim_pose_frame_t)) {
        last_good = buf.data() + pos;
      }
      pos += need;
    }
    if (last_good)
      emitFromFrame(last_good);
    if (pos > 0)
      buf.erase(0, pos);
  }

  if (pose_fd_ >= 0) {
    ::close(pose_fd_);
    pose_fd_ = -1;
  }
  emit logLine("detached from external sim");
  emit stoppedCleanly();
}

} // namespace vsim
