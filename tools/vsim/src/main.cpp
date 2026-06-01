// vsim_d — standalone physics daemon. Replaces the in-process
// SimWorker/SimController/PhysicsCore/MotorModel/SensorModels stack
// with a separate process that talks to the Navigator GCS (and the
// firmware living inside it) via four FIFOs.
//
// Wire protocol: tools/vsim/include/vsim_proto.h.
//
// One thread, three rates:
//   1 kHz : physics tick + drain pwm FIFO + drain ctl FIFO
//   200 Hz: emit IMU frame on the imu FIFO
//   60 Hz : emit pose frame on the pose FIFO
//
// Real-time pacing via clock_nanosleep. Falls back to "best effort"
// scheduling if the host can't keep up; never busy-spins.

#include "fifo_transport.h"
#include "sim_controller.h"
#include "vsim_proto.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <string>
#include <sys/file.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kPhysicsHz = 1000;
constexpr int kImuHz     = 200;
constexpr int kPoseHz    = 60;
constexpr int kImuDiv    = kPhysicsHz / kImuHz;     // 5
constexpr int kPoseDiv   = kPhysicsHz / kPoseHz;    // ~16

constexpr float kRad2Deg = 57.29577951308232f;

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true, std::memory_order_release); }

// Per-instance FIFO isolation (roadmap sim-integration #1 / HANDOFF §5.1):
// append $VSIM_FIFO_SUFFIX to every shared /tmp base path so each
// Navigator+firmware+vsim_d triple owns private FIFOs (and a private
// singleton lock) instead of colliding on the globals. Must match the
// firmware shim (host_navhal / host_imu_feeder path_suffixed) and the suffix
// SimWorker publishes in the environment. Unset/empty == legacy shared paths.
std::string suffixed(const char* base) {
    const char* s = std::getenv("VSIM_FIFO_SUFFIX");
    return std::string(base) + ((s && *s) ? s : "");
}

// Ensure we are the ONLY vsim_d producing on the shared /tmp/vsim_* FIFOs.
// A stale daemon (orphaned by a force-killed/crashed GCS) would otherwise
// keep writing /tmp/vsim_imu alongside us; the firmware then reads torn,
// interleaved frames → NaN IMU → estimator degraded → can't arm. We take
// an flock on a pidfile and, if a previous daemon holds it, terminate it
// and take over. The lock fd is intentionally held open for our lifetime.
void ensureSingleton() {
    const std::string lock = suffixed("/tmp/vsim_d.lock");
    int fd = ::open(lock.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd < 0) return;  // best-effort
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (::flock(fd, LOCK_EX | LOCK_NB) == 0) {
            ::ftruncate(fd, 0);
            ::lseek(fd, 0, SEEK_SET);
            char buf[32];
            int n = std::snprintf(buf, sizeof(buf), "%d\n", (int)::getpid());
            ssize_t w = ::write(fd, buf, (size_t)n);
            (void)w;
            return;  // hold fd (and the lock) for the process lifetime
        }
        // Held by another daemon: read its pid and end it, then retry.
        char buf[32] = {0};
        ::lseek(fd, 0, SEEK_SET);
        ssize_t r = ::read(fd, buf, sizeof(buf) - 1);
        (void)r;
        const int pid = std::atoi(buf);
        if (pid > 1 && pid != (int)::getpid()) {
            ::kill(pid, attempt < 30 ? SIGTERM : SIGKILL);
        }
        ::usleep(30000);
    }
    std::fprintf(stderr, "vsim_d: warning — could not claim singleton lock\n");
}

// Pack one ImuSample into the 76-byte firmware-side layout. Mirrors
// SimWorker's writeImuSample. Gyro converted from rad/s to deg/s.
void packImu(const vsim::ImuSample& s, uint8_t out[76]) {
    float f[19];
    f[0] = s.acc.x();  f[1] = s.acc.y();  f[2] = s.acc.z();
    f[3] = s.gyr.x() * kRad2Deg;
    f[4] = s.gyr.y() * kRad2Deg;
    f[5] = s.gyr.z() * kRad2Deg;
    f[6] = s.mag.x();  f[7] = s.mag.y();  f[8] = s.mag.z();
    // Mirror the calibrated triplets into the "raw" slots. Firmware
    // doesn't differentiate when samples arrive via the bridge.
    f[9]  = f[0]; f[10] = f[1]; f[11] = f[2];
    f[12] = f[3]; f[13] = f[4]; f[14] = f[5];
    f[15] = f[6]; f[16] = f[7]; f[17] = f[8];
    f[18] = s.temp;
    std::memcpy(out, f, 76);
}

}  // namespace

int main(int /*argc*/, char** /*argv*/) {
    // Catch SIGINT/SIGTERM to flush stderr and tear down cleanly.
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    // Drop SIGPIPE: a closed peer would otherwise kill us mid-write.
    std::signal(SIGPIPE, SIG_IGN);

    // Be the sole producer on the shared FIFOs (kills any stale daemon).
    ensureSingleton();

    vsim::FifoIn  pwm_in (suffixed(VSIM_FIFO_PWM));
    vsim::FifoOut imu_out(suffixed(VSIM_FIFO_IMU));
    vsim::FifoOut pose_out(suffixed(VSIM_FIFO_POSE));
    vsim::FifoIn  ctl_in (suffixed(VSIM_FIFO_CTL));

    if (!pwm_in.open() || !imu_out.open() || !pose_out.open() || !ctl_in.open()) {
        std::fprintf(stderr, "vsim_d: FIFO setup failed\n");
        return 1;
    }
    std::fprintf(stderr, "vsim_d: running at %d Hz physics, %d Hz IMU, %d Hz pose\n",
                 kPhysicsHz, kImuHz, kPoseHz);

    vsim::SimController ctl;
    // Persistent running config. SET_GEOMETRY and SET_WORLD each modify a
    // disjoint set of fields on these, so they compose rather than clobber.
    vsim::DroneParams drone;
    vsim::MotorParams motor;
    std::vector<vsim::SimObstacle> obstacles;

    // Initial pose: a few cm above ground, level. NED: z = -0.05.
    vsim::RigidBodyState s0;
    s0.pos_w = vsim::Vec3(0.0f, 0.0f, -0.05f);
    ctl.resetState(s0);

    std::array<float, 4> duty{0.0f, 0.0f, 0.0f, 0.0f};
    uint64_t tick = 0;
    uint32_t imu_seq = 0, pose_seq = 0;
    bool paused = false;

    using clock = std::chrono::steady_clock;
    const auto dt_dur = std::chrono::microseconds(1000000 / kPhysicsHz);
    const float dt = 1.0f / static_cast<float>(kPhysicsHz);
    auto next = clock::now();

    while (!g_stop.load(std::memory_order_acquire)) {
        // 1) Drain incoming pwm. Latest-wins.
        vsim_pwm_frame_t pwm;
        if (pwm_in.poll(VSIM_FRAME_PWM, &pwm, sizeof(pwm))) {
            for (int i = 0; i < 4; ++i) {
                // NOTE: std::clamp(NaN, …) returns NaN, so a NaN motor
                // command from a diverged firmware controller would poison
                // the physics. Reject non-finite duty explicitly.
                const float d = std::isfinite(pwm.duty[i]) ? pwm.duty[i] : 0.0f;
                duty[i] = std::clamp(d, 0.0f, 1.0f);
            }
        }

        // 2) Drain control messages (reset / pause / ...).
        vsim_ctl_frame_t cmd;
        if (ctl_in.poll(VSIM_FRAME_CTL, &cmd, sizeof(cmd))) {
            switch (cmd.subtype) {
                case VSIM_CTL_RESET: {
                    vsim_ctl_reset_t body;
                    std::memcpy(&body, cmd.body, sizeof(body));
                    vsim::RigidBodyState s;
                    s.pos_w   = vsim::Vec3(body.pos_w[0], body.pos_w[1], body.pos_w[2]);
                    s.att     = vsim::Quat(body.quat_wxyz[0], body.quat_wxyz[1],
                                           body.quat_wxyz[2], body.quat_wxyz[3]);
                    s.vel_w   = vsim::Vec3(body.vel_w[0],   body.vel_w[1],   body.vel_w[2]);
                    s.omega_b = vsim::Vec3(body.omega_b[0], body.omega_b[1], body.omega_b[2]);
                    ctl.resetState(s);
                    tick = 0;
                    std::fprintf(stderr, "vsim_d: reset\n");
                    break;
                }
                case VSIM_CTL_PAUSE: {
                    int32_t p;
                    std::memcpy(&p, cmd.body, sizeof(p));
                    paused = (p != 0);
                    std::fprintf(stderr, "vsim_d: %s\n", paused ? "paused" : "resumed");
                    break;
                }
                case VSIM_CTL_PING:
                    std::fprintf(stderr, "vsim_d: ping ok, tick=%llu\n",
                                 static_cast<unsigned long long>(tick));
                    break;
                case VSIM_CTL_SET_GEOMETRY: {
                    vsim_ctl_geometry_t g;
                    std::memcpy(&g, cmd.body, sizeof(g));
                    // Mass + full inertia tensor (row-major into Mat3); leaves
                    // drone's drag/ground/gravity (owned by SET_WORLD) intact.
                    drone.mass = g.mass;
                    for (int k = 0; k < 9; ++k) drone.inertia.m[k] = g.inertia[k];
                    // Per-rotor layout.
                    for (int i = 0; i < 4; ++i) {
                        motor.pos_b[i]     = vsim::Vec3(g.motors[i].pos[0], g.motors[i].pos[1], g.motors[i].pos[2]);
                        motor.axis_b[i]    = vsim::Vec3(g.motors[i].axis[0], g.motors[i].axis[1], g.motors[i].axis[2]);
                        motor.spin[i]      = (g.motors[i].spin >= 0.0f) ? +1 : -1;
                        motor.k_thrust[i]  = g.motors[i].k_thrust;
                        motor.k_moment[i]  = g.motors[i].k_moment;
                        motor.max_omega[i] = g.motors[i].max_omega;
                    }
                    ctl.setDroneParams(drone);
                    ctl.setMotorParams(motor);
                    std::fprintf(stderr,
                                 "vsim_d: geometry set (m=%.3f kg, Idiag=%.4g/%.4g/%.4g)\n",
                                 drone.mass, drone.inertia.at(0,0), drone.inertia.at(1,1), drone.inertia.at(2,2));
                    break;
                }
                case VSIM_CTL_SET_WORLD: {
                    vsim_ctl_world_t w;
                    std::memcpy(&w, cmd.body, sizeof(w));
                    drone.gravity            = w.gravity;
                    drone.ground_z           = w.ground_z;
                    drone.ground_restitution = w.restitution;
                    drone.linear_drag        = w.linear_drag;
                    drone.angular_drag       = w.angular_drag;
                    ctl.setDroneParams(drone);
                    std::fprintf(stderr,
                                 "vsim_d: world set (g=%.2f ground_z=%.2f rest=%.2f drag=%.3f/%.4f)\n",
                                 w.gravity, w.ground_z, w.restitution, w.linear_drag, w.angular_drag);
                    break;
                }
                case VSIM_CTL_CLEAR_OBSTACLES: {
                    obstacles.clear();
                    ctl.setObstacles(obstacles);
                    break;
                }
                case VSIM_CTL_ADD_OBSTACLE: {
                    vsim_ctl_obstacle_t b;
                    std::memcpy(&b, cmd.body, sizeof(b));
                    vsim::SimObstacle ob;
                    ob.type = b.type;
                    ob.pos = vsim::Vec3(b.pos[0], b.pos[1], b.pos[2]);
                    ob.size = vsim::Vec3(b.size[0], b.size[1], b.size[2]);
                    ob.rot_deg = vsim::Vec3(b.rot_deg[0], b.rot_deg[1], b.rot_deg[2]);
                    ob.restitution = b.restitution;
                    obstacles.push_back(ob);
                    ctl.setObstacles(obstacles);
                    std::fprintf(stderr, "vsim_d: obstacle added (type=%d, total=%zu)\n",
                                 ob.type, obstacles.size());
                    break;
                }
                default:
                    break;
            }
        }

        // 3) Advance physics.
        vsim::ImuSample s;
        if (!paused) {
            s = ctl.tick(duty, dt);
        }

        // 4) Emit IMU at kImuHz.
        if (!paused && (tick % kImuDiv) == 0) {
            vsim_imu_frame_t frame{};
            frame.hdr.magic         = VSIM_MAGIC;
            frame.hdr.version       = VSIM_PROTO_VERSION;
            frame.hdr.type          = VSIM_FRAME_IMU;
            frame.hdr.payload_bytes = VSIM_IMU_PAYLOAD_BYTES;
            frame.hdr.seq_no        = ++imu_seq;
            packImu(s, frame.imu_payload);
            imu_out.write(&frame, sizeof(frame));
        }

        // 5) Emit pose at kPoseHz.
        if ((tick % kPoseDiv) == 0) {
            const auto& st = ctl.state();
            const auto& wm = ctl.motorOmegas();
            vsim_pose_frame_t frame{};
            frame.hdr.magic         = VSIM_MAGIC;
            frame.hdr.version       = VSIM_PROTO_VERSION;
            frame.hdr.type          = VSIM_FRAME_POSE;
            frame.hdr.payload_bytes = sizeof(vsim_pose_frame_t) - sizeof(vsim_hdr_t);
            frame.hdr.seq_no        = ++pose_seq;
            frame.tick_lo           = static_cast<uint32_t>(tick & 0xFFFFFFFFu);
            frame.tick_hi           = static_cast<uint32_t>(tick >> 32);
            frame.pos_w[0]     = st.pos_w.x();  frame.pos_w[1] = st.pos_w.y();  frame.pos_w[2] = st.pos_w.z();
            frame.quat_wxyz[0] = st.att.scalar();
            frame.quat_wxyz[1] = st.att.x();
            frame.quat_wxyz[2] = st.att.y();
            frame.quat_wxyz[3] = st.att.z();
            frame.vel_w[0]     = st.vel_w.x();   frame.vel_w[1] = st.vel_w.y();   frame.vel_w[2] = st.vel_w.z();
            frame.omega_b[0]   = st.omega_b.x(); frame.omega_b[1] = st.omega_b.y(); frame.omega_b[2] = st.omega_b.z();
            for (int i = 0; i < 4; ++i) {
                frame.motor_omega[i] = wm[i];
                frame.motor_duty[i]  = duty[i];
            }
            pose_out.write(&frame, sizeof(frame));
        }

        ++tick;
        next += dt_dur;
        auto now = clock::now();
        if (next > now) {
            std::this_thread::sleep_until(next);
        } else {
            // Fell behind. Resync without trying to catch up.
            next = now;
        }
    }

    std::fprintf(stderr, "vsim_d: stopping\n");
    return 0;
}
