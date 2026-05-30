#include "SimWorker.h"

#include "vsim_proto.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcessEnvironment>

#include <cerrno>
#include <csignal>
#include <cstring>
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
    }
    std::memcpy(f.body, &body, sizeof(body));  // 200 B into the 256 B body
    ::write(ctl_fd_, &f, sizeof(f));
    emit logLine("vsim_d: geometry pushed");
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
    daemon_pid_ = pid;
    emit logLine(QString("vsim_d spawned pid=%1 (%2)").arg(pid).arg(bin_q));
    return true;
}

void SimWorker::killDaemon() {
    if (daemon_pid_ <= 0) return;
    ::kill(daemon_pid_, SIGTERM);
    // Reap. waitpid is harmless if vsim_d already died.
    int status = 0;
    // Bounded wait: poll for up to ~1.5 s before SIGKILL.
    for (int i = 0; i < 30; ++i) {
        pid_t r = ::waitpid(daemon_pid_, &status, WNOHANG);
        if (r == daemon_pid_) {
            daemon_pid_ = -1;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ::kill(daemon_pid_, SIGKILL);
    ::waitpid(daemon_pid_, &status, 0);
    daemon_pid_ = -1;
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
    if (!wait_for(VSIM_FIFO_POSE) || !wait_for(VSIM_FIFO_CTL)) {
        emit logLine("vsim_d: timed out waiting for FIFOs");
        return false;
    }
    pose_fd_ = ::open(VSIM_FIFO_POSE, O_RDONLY | O_NONBLOCK);
    ctl_fd_  = ::open(VSIM_FIFO_CTL,  O_RDWR   | O_NONBLOCK);
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

void SimWorker::run() {
    stop_flag_.store(false, std::memory_order_release);

    if (!spawnDaemon())  { emit stoppedCleanly(); return; }
    if (!openFifos())    { killDaemon(); emit stoppedCleanly(); return; }

    emit logLine("vsim_d: pose reader online");
    emit online();

    // Magic-resync buffer. vsim_d writes complete frames per write(),
    // but if we connect mid-stream we may need to walk to the next
    // boundary. Also handles short reads.
    std::string buf;
    while (!stop_flag_.load(std::memory_order_acquire)) {
        char chunk[2048];
        ssize_t r = ::read(pose_fd_, chunk, sizeof(chunk));
        if (r > 0) {
            buf.append(chunk, static_cast<size_t>(r));
        } else if (r == 0 || (r < 0 && errno != EAGAIN)) {
            // Daemon closed or hard error.
            break;
        } else {
            // EAGAIN: sleep for ~1/poseHz to avoid spinning.
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

    closeFifos();
    killDaemon();
    emit logLine("vsim_d: stopped");
    emit stoppedCleanly();
}

}  // namespace vsim
