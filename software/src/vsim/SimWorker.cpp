#include "SimWorker.h"

#include "vsim_proto.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcessEnvironment>

#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace vsim {

namespace {

// Resolve the vsim_d binary path. Priority:
//   1. setVsimBinary("...") explicit override (handled in caller).
//   2. $VSIM_BIN_PATH env var.
//   3. <NavigatorDir>/../../build_vsim/vsim_d  (in-tree layout: a sibling
//      build_vsim/ at the repo root, while Navigator runs from
//      software/build/.)
//   4. "vsim_d" (relies on $PATH).
QString resolveBinary(const QString& override_path) {
    if (!override_path.isEmpty()) return override_path;

    const auto env = QProcessEnvironment::systemEnvironment();
    if (env.contains("VSIM_BIN_PATH")) {
        const QString p = env.value("VSIM_BIN_PATH");
        if (!p.isEmpty()) return p;
    }

    const QString app_dir = QCoreApplication::applicationDirPath();
    if (!app_dir.isEmpty()) {
        const QString guess =
            QDir(app_dir).absoluteFilePath("../../build_vsim/vsim_d");
        if (QFileInfo(guess).isExecutable()) {
            return QFileInfo(guess).canonicalFilePath();
        }
    }
    return QStringLiteral("vsim_d");
}

// Per-instance FIFO isolation (roadmap sim-integration #1): append
// $VSIM_FIFO_SUFFIX to the shared /tmp base path. SimulatorWidget publishes
// the suffix in the environment before vayu_sitl_start + spawning vsim_d, so
// the in-process firmware shim, the spawned daemon, and this reader all agree
// on /tmp/vsim_*<suffix>. Unset/empty == legacy shared paths.
std::string fifoSuffixed(const char* base) {
    const char* s = ::getenv("VSIM_FIFO_SUFFIX");
    return std::string(base) + ((s && *s) ? s : "");
}

}  // namespace

SimWorker::SimWorker(QObject* parent) : QThread(parent) {
    qRegisterMetaType<SimSnapshot>("vsim::SimSnapshot");
}

SimWorker::~SimWorker() {
    requestStop();
    if (isRunning()) wait(2000);
    closeFifos();
}

void SimWorker::requestStop() {
    stop_flag_.store(true, std::memory_order_release);
    killDaemon();
}

SimSnapshot SimWorker::snapshot() const {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    return last_snap_;
}

void SimWorker::sendReset() {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.hdr.seq_no        = 0;  // ctl is one-shot; the daemon doesn't dedupe by seq
    f.subtype           = VSIM_CTL_RESET;
    vsim_ctl_reset_t body{};
    body.pos_w[2]       = -0.05f;  // a few cm above ground
    body.quat_wxyz[0]   = 1.0f;    // identity
    std::memcpy(f.body, &body, sizeof(body));
    ::write(ctl_fd_, &f, sizeof(f));
}

void SimWorker::sendTestRig(bool on) {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype           = VSIM_CTL_SET_TESTRIG;
    vsim_ctl_testrig_t body{};
    body.enable = on ? 1 : 0;
    body.pos[2] = -0.05f;
    std::memcpy(f.body, &body, sizeof(body));
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine(QStringLiteral("vsim_d: test-rig %1").arg(on ? "ON" : "off"));
}

void SimWorker::sendRigPose(float rollDeg, float pitchDeg, float yawDeg) {
    if (ctl_fd_ < 0) {
        emit logLine(QStringLiteral("rig pose ignored — ctl channel closed "
                                    "(is the sim running?)"));
        return;
    }
    // NED ZYX (yaw-pitch-roll) Euler -> body->world quaternion: the exact
    // inverse of quatToEulerNED() in VsimTypes.h.
    const double h = 0.017453292519943295 * 0.5;  // deg -> half-radians
    const double cr = std::cos(rollDeg * h),  sr = std::sin(rollDeg * h);
    const double cp = std::cos(pitchDeg * h), sp = std::sin(pitchDeg * h);
    const double cy = std::cos(yawDeg * h),   sy = std::sin(yawDeg * h);
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype           = VSIM_CTL_RESET;
    vsim_ctl_reset_t body{};
    body.pos_w[2]     = -0.05f;
    body.quat_wxyz[0] = static_cast<float>(cr * cp * cy + sr * sp * sy);
    body.quat_wxyz[1] = static_cast<float>(sr * cp * cy - cr * sp * sy);
    body.quat_wxyz[2] = static_cast<float>(cr * sp * cy + sr * cp * sy);
    body.quat_wxyz[3] = static_cast<float>(cr * cp * sy - sr * sp * cy);
    std::memcpy(f.body, &body, sizeof(body));
    const ssize_t n = ::write(ctl_fd_, &f, sizeof(f));
    emit logLine(QStringLiteral("rig pose sent: r=%1 p=%2 y=%3 (%4 B)")
                     .arg(rollDeg, 0, 'f', 0).arg(pitchDeg, 0, 'f', 0)
                     .arg(yawDeg, 0, 'f', 0).arg(static_cast<int>(n)));
}

void SimWorker::sendGeometry(const GeometryConfig& g) {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.hdr.seq_no        = 0;
    f.subtype           = VSIM_CTL_SET_GEOMETRY;

    vsim_ctl_geometry_t body{};
    body.mass = g.mass;
    for (int i = 0; i < 9; ++i) body.inertia[i] = g.inertia[i];
    for (int i = 0; i < 4; ++i) {
        const auto& m = g.motors[i];
        body.motors[i].pos[0]   = m.pos.x();
        body.motors[i].pos[1]   = m.pos.y();
        body.motors[i].pos[2]   = m.pos.z();
        body.motors[i].axis[0]  = m.axis.x();
        body.motors[i].axis[1]  = m.axis.y();
        body.motors[i].axis[2]  = m.axis.z();
        body.motors[i].spin      = static_cast<float>(m.spin);
        body.motors[i].k_thrust  = m.k_thrust;
        body.motors[i].k_moment  = m.k_moment;
        body.motors[i].max_omega = m.max_omega;
        body.motors[i].tau       = m.tau;
    }
    std::memcpy(f.body, &body, sizeof(body));  // 200 B into the 256 B body
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine("vsim_d: geometry pushed");
}

void SimWorker::sendWorld(const WorldConfig& w) {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.hdr.seq_no        = 0;
    f.subtype           = VSIM_CTL_SET_WORLD;

    vsim_ctl_world_t body{};
    body.gravity      = w.gravity;
    body.ground_z     = w.ground_z;
    body.restitution  = w.restitution;
    body.linear_drag  = w.linear_drag;
    body.angular_drag = w.angular_drag;
    body.ground_right_gain = w.ground_right_gain;
    body.ground_right_damp = w.ground_right_damp;
    std::memcpy(f.body, &body, sizeof(body));
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine("vsim_d: world pushed");
}

void SimWorker::sendObstacles(const QVector<Obstacle>& obs) {
    if (ctl_fd_ < 0) return;
    auto frame = [&](uint32_t subtype) {
        vsim_ctl_frame_t f{};
        f.hdr.magic         = VSIM_MAGIC;
        f.hdr.version       = VSIM_PROTO_VERSION;
        f.hdr.type          = VSIM_FRAME_CTL;
        f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
        f.subtype           = subtype;
        return f;
    };
    // Clear, then append each shape.
    { vsim_ctl_frame_t f = frame(VSIM_CTL_CLEAR_OBSTACLES); ::write(ctl_fd_, &f, sizeof(f)); }
    for (const Obstacle& o : obs) {
        vsim_ctl_frame_t f = frame(VSIM_CTL_ADD_OBSTACLE);
        vsim_ctl_obstacle_t b{};
        b.type = o.type;
        b.pos[0] = o.pos.x();    b.pos[1] = o.pos.y();    b.pos[2] = o.pos.z();
        b.size[0] = o.size.x();  b.size[1] = o.size.y();  b.size[2] = o.size.z();
        b.rot_deg[0] = o.rotate.x(); b.rot_deg[1] = o.rotate.y(); b.rot_deg[2] = o.rotate.z();
        b.restitution = o.restitution;
        std::memcpy(f.body, &b, sizeof(b));
        ::write(ctl_fd_, &f, sizeof(f));
    }
    emit logLine(QString("vsim_d: %1 obstacles pushed").arg(obs.size()));
}

void SimWorker::sendRates(int imuHz, int physicsHz, int poseHz) {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype           = VSIM_CTL_SET_RATES;
    vsim_ctl_rates_t b{};
    b.imu_hz     = static_cast<uint32_t>(imuHz);
    b.physics_hz = static_cast<uint32_t>(physicsHz);
    b.pose_hz    = static_cast<uint32_t>(poseHz);
    std::memcpy(f.body, &b, sizeof(b));
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine(QString("vsim_d: rates imu=%1 physics=%2 pose=%3 Hz")
                     .arg(imuHz).arg(physicsHz).arg(poseHz));
}

void SimWorker::sendWorldMesh(const QString& path, quint32 verts, quint32 tris,
                              quint32 nodes, float restitution, bool doubleSided) {
    if (ctl_fd_ < 0) return;
    const QByteArray pb = path.toLocal8Bit();
    vsim_ctl_world_mesh_t b{};
    if (pb.size() >= int(sizeof(b.path))) {
        emit logLine("vsim_d: world-mesh path too long");
        return;
    }
    vsim_ctl_frame_t f{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype           = VSIM_CTL_SET_WORLD_MESH;
    b.vertex_count = verts; b.triangle_count = tris; b.node_count = nodes;
    b.flags = doubleSided ? 1u : 0u;
    b.restitution = restitution;
    b.path_len = static_cast<uint32_t>(pb.size());
    std::memcpy(b.path, pb.constData(), pb.size());
    std::memcpy(f.body, &b, sizeof(b));
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine(QString("vsim_d: world mesh -> %1 (%2 tris)").arg(path).arg(tris));
}

void SimWorker::clearWorldMesh() {
    if (ctl_fd_ < 0) return;
    vsim_ctl_frame_t f{};
    f.hdr.magic = VSIM_MAGIC;
    f.hdr.version = VSIM_PROTO_VERSION;
    f.hdr.type = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype = VSIM_CTL_CLEAR_WORLD_MESH;
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine("vsim_d: world mesh cleared");
}

bool SimWorker::spawnDaemon() {
    const QString bin_q = resolveBinary(vsim_bin_);
    const QByteArray bin_b = bin_q.toLocal8Bit();
    char* const argv[] = { const_cast<char*>(bin_b.constData()), nullptr };

    pid_t pid = -1;
    int rc = posix_spawnp(&pid, argv[0], nullptr, nullptr, argv, environ);
    if (rc != 0) {
        emit logLine(QString("vsim_d spawn failed (%1): %2")
                         .arg(bin_q, ::strerror(rc)));
        return false;
    }
    {
        // Publish under the lock: a requestStop() racing with start-up must
        // either see -1 (and the worker tears down right after spawn) or this
        // pid (and kills it) — never miss it and leak the daemon.
        std::lock_guard<std::mutex> lk(daemon_mtx_);
        daemon_pid_ = pid;
    }
    emit logLine(QString("vsim_d spawned pid=%1 (%2)").arg(pid).arg(bin_q));
    return true;
}

void SimWorker::killDaemon() {
    // killDaemon() is called from BOTH the GUI thread (requestStop) and the
    // worker thread (end of run() / the dtor). Atomically *claim* the pid
    // under the lock — set daemon_pid_ = -1 before releasing — so exactly one
    // caller runs the SIGTERM->reap->SIGKILL sequence and the other sees -1
    // and returns. Without this the two raced: after one waitpid() reaped the
    // pid, the loser could waitpid()-miss for 1.5 s and then SIGKILL a pid the
    // OS had already recycled onto an unrelated process.
    pid_t pid;
    {
        std::lock_guard<std::mutex> lk(daemon_mtx_);
        pid = daemon_pid_;
        daemon_pid_ = -1;
    }
    if (pid <= 0) return;

    ::kill(pid, SIGTERM);
    // Bounded wait: poll for up to ~1.5 s before SIGKILL. The lock is NOT held
    // across the wait — only this caller owns `pid` now, so there's no race.
    int status = 0;
    for (int i = 0; i < 30; ++i) {
        pid_t r = ::waitpid(pid, &status, WNOHANG);
        if (r == pid || (r < 0 && errno == ECHILD)) return;  // reaped / gone
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ::kill(pid, SIGKILL);
    ::waitpid(pid, &status, 0);
}

bool SimWorker::openFifos() {
    // Wait briefly for vsim_d to mkfifo the paths. It does so very
    // quickly (millisecond range) after startup.
    auto wait_for = [](const char* path) {
        for (int i = 0; i < 50; ++i) {
            struct stat st;
            if (::stat(path, &st) == 0) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    };
    const std::string pose_path = fifoSuffixed(VSIM_FIFO_POSE);
    const std::string ctl_path  = fifoSuffixed(VSIM_FIFO_CTL);
    if (!wait_for(pose_path.c_str()) || !wait_for(ctl_path.c_str())) {
        emit logLine("vsim_d: timed out waiting for FIFOs");
        return false;
    }
    pose_fd_ = ::open(pose_path.c_str(), O_RDONLY | O_NONBLOCK);
    ctl_fd_  = ::open(ctl_path.c_str(),  O_RDWR   | O_NONBLOCK);
    if (pose_fd_ < 0 || ctl_fd_ < 0) {
        emit logLine(QString("vsim_d: FIFO open failed: %1").arg(::strerror(errno)));
        return false;
    }
    return true;
}

void SimWorker::closeFifos() {
    if (pose_fd_ >= 0) { ::close(pose_fd_); pose_fd_ = -1; }
    if (ctl_fd_  >= 0) { ::close(ctl_fd_);  ctl_fd_  = -1; }
}

void SimWorker::emitFromFrame(const void* bytes) {
    const auto* frame = reinterpret_cast<const vsim_pose_frame_t*>(bytes);
    SimSnapshot snap;
    snap.pos_w   = Vec3(frame->pos_w[0], frame->pos_w[1], frame->pos_w[2]);
    snap.att     = Quat(frame->quat_wxyz[0], frame->quat_wxyz[1],
                        frame->quat_wxyz[2], frame->quat_wxyz[3]);
    snap.vel_w   = Vec3(frame->vel_w[0],   frame->vel_w[1],   frame->vel_w[2]);
    snap.omega_b = Vec3(frame->omega_b[0], frame->omega_b[1], frame->omega_b[2]);
    for (int i = 0; i < 4; ++i) {
        snap.motor_omega[i] = frame->motor_omega[i];
        snap.motor_duty[i]  = frame->motor_duty[i];
    }
    snap.tick_count = (static_cast<uint64_t>(frame->tick_hi) << 32) | frame->tick_lo;
    {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        last_snap_ = snap;
    }
    emit poseUpdated(snap);
}

void SimWorker::startAttach(const QString& posePath) {
    attach_only_ = true;
    attach_pose_path_ = posePath;
    start();
}

void SimWorker::run() {
    stop_flag_.store(false, std::memory_order_release);

    if (attach_only_) {
        // Don't own a daemon: just read the existing pose FIFO. Wait briefly for
        // the file to appear (the tuner spawns its vsim_d a moment after launch).
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(20);
        const QByteArray p = attach_pose_path_.toLocal8Bit();
        while (!stop_flag_.load(std::memory_order_acquire)) {
            pose_fd_ = ::open(p.constData(), O_RDONLY | O_NONBLOCK);
            if (pose_fd_ >= 0) break;
            if (std::chrono::steady_clock::now() > deadline) {
                emit logLine("attach: tuner pose FIFO never appeared");
                emit stoppedCleanly();
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (pose_fd_ < 0) { emit stoppedCleanly(); return; }
        emit logLine("attached to tuner sim: " + attach_pose_path_);
        emit online();
    } else {
        if (!spawnDaemon())  { emit stoppedCleanly(); return; }
        if (!openFifos())    { killDaemon(); emit stoppedCleanly(); return; }

        emit logLine("vsim_d: pose reader online");
        emit online();
    }

    // Magic-resync buffer. vsim_d writes complete frames per write(),
    // but if we connect mid-stream we may need to walk to the next
    // boundary. Also handles short reads.
    //
    // Startup EOF tolerance: openFifos() only waits for the pose FIFO *file*
    // to exist. On a restart the file is a leftover from the previous run, so
    // it exists immediately — before the freshly spawned vsim_d has opened it
    // for writing. A read() on a writer-less FIFO returns 0 (EOF); treating
    // that as "daemon died" here would break the loop and killDaemon() the
    // daemon we just spawned (the restart-freeze bug). So tolerate EOF until
    // the producer connects — bounded by a grace window — and only treat a
    // *later* EOF (after we've seen the producer) as a real disconnect.
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
            // No writer on the FIFO right now.
            if (producer_seen) break;  // producer connected, then went away
            if (std::chrono::steady_clock::now() > startup_deadline) {
                emit logLine("vsim_d: no pose producer within startup grace");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } else if (r < 0 && errno != EAGAIN) {
            break;  // hard error
        } else {
            // EAGAIN: a writer is present but no data yet.
            producer_seen = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }

        // Dispatch as many complete pose frames as we have. Only emit
        // the LATEST one per loop iteration (latest-wins; the renderer
        // doesn't need every 60 Hz frame if we're behind).
        size_t pos = 0;
        const void* last_good = nullptr;
        while (pos + sizeof(vsim_hdr_t) <= buf.size()) {
            const auto* hdr = reinterpret_cast<const vsim_hdr_t*>(buf.data() + pos);
            if (hdr->magic != VSIM_MAGIC) {
                // Walk forward one byte and try again.
                ++pos;
                continue;
            }
            const size_t need = sizeof(vsim_hdr_t) + hdr->payload_bytes;
            if (pos + need > buf.size()) break;  // incomplete tail
            if (hdr->version == VSIM_PROTO_VERSION &&
                hdr->type    == VSIM_FRAME_POSE   &&
                need         == sizeof(vsim_pose_frame_t)) {
                last_good = buf.data() + pos;
            }
            pos += need;
        }
        if (last_good) emitFromFrame(last_good);
        if (pos > 0)   buf.erase(0, pos);
    }

    if (attach_only_) {
        if (pose_fd_ >= 0) { ::close(pose_fd_); pose_fd_ = -1; }
        emit logLine("detached from tuner sim");
    } else {
        closeFifos();
        killDaemon();
        emit logLine("vsim_d: stopped");
    }
    emit stoppedCleanly();
}

}  // namespace vsim
