// vsim_d — standalone physics daemon. Replaces the in-process
// SimWorker/SimController/PhysicsCore/MotorModel/SensorModels stack
// with a separate process that talks to the Navigator GCS (and the
// firmware living inside it) via four FIFOs.
//
// Wire protocol: sim/vsim/include/vsim_proto.h.
//
// One thread, three rates:
//   8 kHz : physics tick + drain pwm FIFO + drain ctl FIFO
//   1 kHz : emit IMU frame on the imu FIFO
//   60 Hz : emit pose frame on the pose FIFO
//
// Real-time pacing via clock_nanosleep. Falls back to "best effort"
// scheduling if the host can't keep up; never busy-spins.

#include "fifo_transport.h"
#include "sim_controller.h"
#include "trimesh_bvh.h"
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
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kPhysicsHz = 8000;
constexpr int kImuHz     = 1000;
constexpr int kPoseHz    = 60;
constexpr int kImuDiv    = kPhysicsHz / kImuHz;     // 8 substeps/sample
constexpr int kPoseDiv   = kPhysicsHz / kPoseHz;    // ~133

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

// Pack one ImuSample into the 88-byte firmware-side bmx160_all_converted_reading_t
// layout (22 floats: acc, gyr, mag, acc_raw, gyr_raw, mag_compensated,
// mag_fusion, temp). Gyro converted from rad/s to deg/s. NOTE: the layout grew
// from 19→22 floats when bmx160 added mag_fusion[3] (the estimator's heading
// input) — see VSIM_IMU_PAYLOAD_BYTES. The producer MUST fill mag_fusion or the
// firmware reads {temp,0,0} there and yaw becomes unobservable in SITL.
void packImu(const vsim::ImuSample& s, uint8_t out[88]) {
    float f[22];
    f[0] = s.acc.x();  f[1] = s.acc.y();  f[2] = s.acc.z();
    f[3] = s.gyr.x() * kRad2Deg;
    f[4] = s.gyr.y() * kRad2Deg;
    f[5] = s.gyr.z() * kRad2Deg;
    f[6] = s.mag.x();  f[7] = s.mag.y();  f[8] = s.mag.z();
    // Mirror the calibrated triplets into the "raw" / "compensated" slots.
    // Firmware doesn't differentiate when samples arrive via the bridge.
    f[9]  = f[0]; f[10] = f[1]; f[11] = f[2];   // acc_raw
    f[12] = f[3]; f[13] = f[4]; f[14] = f[5];   // gyr_raw
    f[15] = f[6]; f[16] = f[7]; f[17] = f[8];   // mag_compensated
    // mag_fusion[3]: unit-normalized mag — the estimator's absolute heading
    // reference. On real hardware bmx160 computes this; in SITL we must too.
    const float mn = std::sqrt(f[6] * f[6] + f[7] * f[7] + f[8] * f[8]);
    if (mn > 1e-6f) { f[18] = f[6] / mn; f[19] = f[7] / mn; f[20] = f[8] / mn; }
    else            { f[18] = f[19] = f[20] = 0.0f; }
    f[21] = s.temp;
    std::memcpy(out, f, 88);
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
    vsim::FifoOut baro_out(suffixed(VSIM_FIFO_BARO));
    vsim::FifoIn  ctl_in (suffixed(VSIM_FIFO_CTL));

    if (!pwm_in.open() || !imu_out.open() || !pose_out.open() ||
        !baro_out.open() || !ctl_in.open()) {
        std::fprintf(stderr, "vsim_d: FIFO setup failed\n");
        return 1;
    }
    std::fprintf(stderr,
                 "vsim_d: boot default %d Hz physics, %d Hz IMU, %d Hz pose "
                 "(reconfigured live by SET_RATES — watch for 'vsim_d: rates ...')\n",
                 kPhysicsHz, kImuHz, kPoseHz);

    vsim::SimController ctl;
    // Persistent running config. SET_GEOMETRY and SET_WORLD each modify a
    // disjoint set of fields on these, so they compose rather than clobber.
    vsim::DroneParams drone;
    vsim::MotorParams motor;
    std::vector<vsim::SimObstacle> obstacles;

    // Higher-fidelity actuator imperfections, opt-in via env so default runs and
    // CI are byte-identical. These persist across SET_GEOMETRY (the geometry
    // frame does not carry them). Identified from real logs (plant_id): a
    // ~0.10 s transport delay + idle-stall reproduce the real ~2 Hz pitch cycle.
    if (const char* e = std::getenv("VSIM_MOTOR_DELAY_MS"))
        motor.transport_delay = std::atof(e) * 1e-3f;
    if (const char* e = std::getenv("VSIM_STALL_DUTY"))
        motor.stall_duty = std::atof(e);
    if (const char* e = std::getenv("VSIM_RESPIN_TAU"))
        motor.respin_tau = std::atof(e);
    if (motor.transport_delay > 0.0f || motor.stall_duty > 0.0f)
        std::fprintf(stderr, "vsim_d: actuator imperfections ON "
                     "(delay=%.0fms stall_duty=%.3f respin_tau=%.0fms)\n",
                     motor.transport_delay * 1e3f, motor.stall_duty,
                     motor.respin_tau * 1e3f);
    if (const char* e = std::getenv("VSIM_VIBE_G")) {
        float vg = std::atof(e);
        ctl.setVibeGain(vg);
        std::fprintf(stderr, "vsim_d: thrust-scaled vibration ON (%.2f g/full)\n", vg);
    }
    // Sim-cadence state log (env-gated): writes one row per IMU sample at the
    // TRUE sim clock — immune to the harness's wall-clock polling/aliasing,
    // which is what makes high-frequency dynamics unmeasurable in lockstep
    // (sim runs faster than realtime). Columns: t_ms, wx,wy,wz [deg/s], m0..m3.
    FILE* state_log = nullptr;
    if (const char* e = std::getenv("VSIM_LOG_PATH")) {
        state_log = std::fopen(e, "w");
        if (state_log) {
            std::fprintf(state_log, "t_ms,wx,wy,wz,m0,m1,m2,m3\n");
            std::fprintf(stderr, "vsim_d: sim-cadence state log -> %s\n", e);
        }
    }

    // World mesh: SET_WORLD_MESH names a serialized BVH file the GCS wrote; we
    // mmap it read-only and hand SimController a non-owning trimesh::Bvh view,
    // so the mapping must outlive every physics step. munmap on replace/exit.
    void*  world_map      = nullptr;
    size_t world_map_size = 0;
    auto drop_world_mesh = [&] {
        ctl.clearWorldMesh();
        if (world_map) { ::munmap(world_map, world_map_size); world_map = nullptr; world_map_size = 0; }
    };

    // Initial pose: a few cm above ground, level. NED: z = -0.05.
    vsim::RigidBodyState s0;
    s0.pos_w = vsim::Vec3(0.0f, 0.0f, -0.05f);
    ctl.resetState(s0);

    std::array<float, 4> duty{0.0f, 0.0f, 0.0f, 0.0f};
    uint64_t tick = 0;          // physics step count (pose-frame display)
    uint64_t outer = 0;         // sample/pace iterations
    uint32_t imu_seq = 0, pose_seq = 0, baro_seq = 0;
    bool paused = false;

    // Fault injection (VSIM_CTL_SET_FAULTS): latched failures for failsafe tests.
    std::array<bool, 4> motor_kill{false, false, false, false};
    bool imu_dropout = false;
    vsim::ImuSample imu_held{};  // last good sample, replayed while imu_dropout

    // Per-sensor enable (VSIM_CTL_SET_NOISE). enable==false zeroes that channel
    // in the emitted sample — the "dropout" the sensor-enable toggle injects.
    bool acc_enable = true, gyr_enable = true, mag_enable = true;

    // Runtime-tunable rates (VSIM_CTL_SET_RATES). imu_hz is the wall-clock pace
    // AND the firmware loop rate (its inner loop runs once per IMU sample);
    // physics_hz/imu_hz RK4 substeps run per sample so we can integrate fast
    // (8-10 kHz) while pacing + sampling stay at a sane, jitter-free rate.
    int imu_hz = kImuHz, physics_hz = kPhysicsHz, pose_hz = kPoseHz;
    int substeps = 1, pose_div = 1;
    float dt_sub = 1.0f / static_cast<float>(kPhysicsHz);
    using clock = std::chrono::steady_clock;
    std::chrono::microseconds outer_dur{1000000 / kImuHz};
    auto recompute_rates = [&] {
        if (imu_hz < 1) imu_hz = 1;
        if (physics_hz < imu_hz) physics_hz = imu_hz;
        if (pose_hz < 1) pose_hz = 1;
        substeps = physics_hz / imu_hz;
        if (substeps < 1) substeps = 1;
        physics_hz = imu_hz * substeps;                 // snap to exact multiple
        dt_sub = 1.0f / static_cast<float>(physics_hz);
        pose_div = imu_hz / pose_hz;
        if (pose_div < 1) pose_div = 1;
        outer_dur = std::chrono::microseconds(1000000 / imu_hz);
    };
    recompute_rates();
    auto next = clock::now();

    // Phase 2 lockstep (firmware/docs/plans/sitl-lockstep-sim.md): when VSIM_LOCKSTEP is
    // set, pace the loop on the firmware's PWM round-trip instead of the wall
    // clock, so the sim runs as fast as the firmware can consume it (faster than
    // realtime). A credit window lets vsim emit up to N IMU samples ahead of the
    // last PWM it has seen — enough to prime the firmware's estimator->control->
    // motor pipeline (depth ~2-3) and absorb thread jitter, while bounding the
    // IMU FIFO occupancy so frames are never dropped (FifoOut drops on EAGAIN,
    // and a dropped IMU sample = lost sim time = desync). Beyond the window vsim
    // blocks for fresh PWM, with a real-time watchdog so a non-producing
    // firmware can never wedge the sim. Default off → unchanged realtime pacing.
    const bool lockstep = [] {
        const char* e = std::getenv("VSIM_LOCKSTEP");
        return e && *e && std::strcmp(e, "0") != 0;
    }();
    const int ls_credit = [] {
        const char* e = std::getenv("VSIM_LOCKSTEP_CREDIT");
        const int v = e ? std::atoi(e) : 0;
        // Default 2: the credit window is also added control LATENCY in sim-time
        // (vsim applies PWM up to `credit` samples stale), and the determinism
        // validation (tools/autotune/validate_lockstep_determinism.py) showed
        // that a marginal roll/pitch loop tumbles at credit>=4 while credit=2
        // tracks realtime to within its run-to-run jitter. Raise it only for a
        // well-damped plant that tolerates the extra latency (more speed); see
        // firmware/docs/plans/sitl-lockstep-sim.md.
        return v > 0 ? v : 2;           // IMU samples vsim may run ahead of PWM
    }();
    const auto ls_watchdog = std::chrono::milliseconds(100);
    uint32_t last_pwm_seq = 0;
    int      ahead = 0;                 // IMU samples since the last PWM advance
    if (lockstep)
        std::fprintf(stderr,
                     "vsim_d: LOCKSTEP mode (credit=%d) — paced by PWM round-trip, "
                     "not the wall clock\n", ls_credit);

    // Drain the pwm FIFO (latest-wins), apply duty, and reset the credit window
    // when the firmware acknowledges progress via a new PWM sequence number.
    // Returns true iff the sequence advanced. NOTE: std::clamp(NaN,…) returns
    // NaN, so a NaN duty from a diverged controller would poison physics —
    // reject non-finite duty explicitly.
    auto ingest_pwm = [&]() -> bool {
        vsim_pwm_frame_t pwm;
        if (!pwm_in.poll(VSIM_FRAME_PWM, &pwm, sizeof(pwm)))
            return false;
        for (int i = 0; i < 4; ++i) {
            const float d = std::isfinite(pwm.duty[i]) ? pwm.duty[i] : 0.0f;
            duty[i] = std::clamp(d, 0.0f, 1.0f);
        }
        if (pwm.hdr.seq_no != last_pwm_seq) {
            last_pwm_seq = pwm.hdr.seq_no;
            ahead = 0;
            return true;
        }
        return false;
    };

    while (!g_stop.load(std::memory_order_acquire)) {
        // 1) Drain incoming pwm. Latest-wins (+ credit-window bookkeeping).
        ingest_pwm();

        // 2) Drain control messages (reset / pause / ...). Lossless: a startup
        // burst (rates + geometry + world) must ALL apply, so process every
        // queued frame, not just the latest (poll() would drop geometry).
        vsim_ctl_frame_t cmd;
        while (ctl_in.pollNext(VSIM_FRAME_CTL, &cmd, sizeof(cmd))) {
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
                    // Deterministic sensor reset: a non-zero seed makes an
                    // identical reset reproduce an identical noise trajectory,
                    // so the autotuner's cost is repeatable for fixed gains.
                    if (body.seed != 0) { ctl.seedSensors(body.seed); ctl.seedWind(body.seed); }
                    tick = 0;
                    outer = 0;
                    std::fprintf(stderr, "vsim_d: reset%s\n",
                                 body.seed ? " (seeded)" : "");
                    break;
                }
                case VSIM_CTL_PAUSE: {
                    int32_t p;
                    std::memcpy(&p, cmd.body, sizeof(p));
                    paused = (p != 0);
                    std::fprintf(stderr, "vsim_d: %s\n", paused ? "paused" : "resumed");
                    break;
                }
                case VSIM_CTL_SET_TESTRIG: {
                    vsim_ctl_testrig_t t;
                    std::memcpy(&t, cmd.body, sizeof(t));
                    ctl.setTestRig(t.enable != 0,
                                   vsim::Vec3(t.pos[0], t.pos[1], t.pos[2]),
                                   t.tether_k);
                    std::fprintf(stderr,
                                 "vsim_d: test-rig %s @ (%.2f,%.2f,%.2f) tether_k=%.1f\n",
                                 t.enable ? "ON" : "off",
                                 t.pos[0], t.pos[1], t.pos[2], t.tether_k);
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
                        // tau <= 0 (e.g. a zero-padding older sender) keeps the default.
                        motor.tau[i]       = (g.motors[i].tau > 1e-6f) ? g.motors[i].tau : 0.0125f;
                    }
                    ctl.setDroneParams(drone);
                    ctl.setMotorParams(motor);
                    std::fprintf(stderr,
                                 "vsim_d: geometry set (m=%.3f kg, Idiag=%.4g/%.4g/%.4g)\n",
                                 drone.mass, drone.inertia.at(0,0), drone.inertia.at(1,1), drone.inertia.at(2,2));
                    // Echo the per-rotor positions actually applied so a headless
                    // harness can assert it is flying the loaded frame's geometry,
                    // not the compiled-in defaults (see fidelity verify_frame).
                    for (int i = 0; i < 4; ++i)
                        std::fprintf(stderr,
                                     "vsim_d:   motor%d pos=(%.5f, %.5f, %.5f) spin=%d kt=%.4g\n",
                                     i, motor.pos_b[i].x(), motor.pos_b[i].y(),
                                     motor.pos_b[i].z(), motor.spin[i], motor.k_thrust[i]);
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
                    drone.ground_right_gain  = w.ground_right_gain;
                    drone.ground_right_damp  = w.ground_right_damp;
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
                case VSIM_CTL_SET_WORLD_MESH: {
                    vsim_ctl_world_mesh_t m;
                    std::memcpy(&m, cmd.body, sizeof(m));
                    m.path[sizeof(m.path) - 1] = '\0';
                    drop_world_mesh();   // release any previous mapping first
                    int fd = ::open(m.path, O_RDONLY);
                    if (fd < 0) {
                        std::fprintf(stderr, "vsim_d: world mesh open(%s) failed\n", m.path);
                        break;
                    }
                    struct stat st{};
                    if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
                        std::fprintf(stderr, "vsim_d: world mesh fstat failed\n");
                        ::close(fd);
                        break;
                    }
                    void* base = ::mmap(nullptr, static_cast<size_t>(st.st_size),
                                        PROT_READ, MAP_PRIVATE, fd, 0);
                    ::close(fd);   // mapping survives the fd
                    if (base == MAP_FAILED) {
                        std::fprintf(stderr, "vsim_d: world mesh mmap failed\n");
                        break;
                    }
                    vsim::trimesh::Bvh bvh = vsim::trimesh::Bvh::fromBytes(
                        static_cast<const uint8_t*>(base), static_cast<size_t>(st.st_size));
                    if (!bvh.valid() ||
                        bvh.h->triangle_count != m.triangle_count ||
                        bvh.h->vertex_count   != m.vertex_count) {
                        std::fprintf(stderr,
                            "vsim_d: world mesh invalid/mismatch (frame %u/%u tris/verts)\n",
                            m.triangle_count, m.vertex_count);
                        ::munmap(base, static_cast<size_t>(st.st_size));
                        break;
                    }
                    world_map      = base;
                    world_map_size = static_cast<size_t>(st.st_size);
                    ctl.setWorldMesh(bvh, m.restitution);
                    std::fprintf(stderr,
                                 "vsim_d: world mesh mapped (%u tris, %u nodes, rest=%.2f) from %s\n",
                                 bvh.h->triangle_count, bvh.h->node_count, m.restitution, m.path);
                    break;
                }
                case VSIM_CTL_CLEAR_WORLD_MESH: {
                    drop_world_mesh();
                    std::fprintf(stderr, "vsim_d: world mesh cleared\n");
                    break;
                }
                case VSIM_CTL_SET_RATES: {
                    vsim_ctl_rates_t r;
                    std::memcpy(&r, cmd.body, sizeof(r));
                    imu_hz     = static_cast<int>(r.imu_hz);
                    physics_hz = static_cast<int>(r.physics_hz);
                    pose_hz    = static_cast<int>(r.pose_hz);
                    recompute_rates();
                    next = clock::now();   // re-anchor the pace clock
                    std::fprintf(stderr,
                                 "vsim_d: rates imu=%d Hz, physics=%d Hz (%d substeps), pose=%d Hz\n",
                                 imu_hz, physics_hz, substeps, pose_hz);
                    break;
                }
                case VSIM_CTL_SET_NOISE: {
                    vsim_ctl_noise_t n;
                    std::memcpy(&n, cmd.body, sizeof(n));
                    vsim::SensorNoise sn;  // start from defaults, override σ/clip
                    sn.acc_noise_std = n.acc_sigma;
                    sn.acc_bias_clip = n.acc_bias_clip;
                    sn.gyr_noise_std = n.gyr_sigma;
                    sn.gyr_bias_clip = n.gyr_bias_clip;
                    sn.mag_noise_std = n.mag_sigma;
                    sn.mag_bias_clip = n.mag_bias_clip;
                    ctl.setNoise(sn);
                    acc_enable = (n.acc_enable != 0);
                    gyr_enable = (n.gyr_enable != 0);
                    mag_enable = (n.mag_enable != 0);
                    std::fprintf(stderr,
                                 "vsim_d: noise set (acc σ=%.4g en=%d, gyr σ=%.4g en=%d, mag σ=%.4g en=%d)\n",
                                 sn.acc_noise_std, acc_enable, sn.gyr_noise_std,
                                 gyr_enable, sn.mag_noise_std, mag_enable);
                    break;
                }
                case VSIM_CTL_SET_FAULTS: {
                    vsim_ctl_faults_t fl;
                    std::memcpy(&fl, cmd.body, sizeof(fl));
                    for (int i = 0; i < 4; ++i) motor_kill[i] = (fl.motor_kill[i] != 0);
                    imu_dropout = (fl.imu_dropout != 0);
                    std::fprintf(stderr,
                                 "vsim_d: faults kill=%d%d%d%d imu_dropout=%d\n",
                                 motor_kill[0], motor_kill[1], motor_kill[2],
                                 motor_kill[3], imu_dropout);
                    break;
                }
                case VSIM_CTL_SET_WIND: {
                    vsim_ctl_wind_t w;
                    std::memcpy(&w, cmd.body, sizeof(w));
                    vsim::WindConfig wc;
                    wc.steady      = vsim::Vec3(w.steady[0], w.steady[1], w.steady[2]);
                    wc.gust_amp    = w.gust_amp;
                    wc.gust_period = w.gust_period;
                    wc.turb_sigma  = w.turb_sigma;
                    wc.turb_tau    = w.turb_tau;
                    wc.enable      = (w.enable != 0);
                    ctl.setWind(wc);
                    std::fprintf(stderr,
                                 "vsim_d: wind %s steady=(%.1f,%.1f,%.1f) gust=%.1f/%.1fs turb=%.2f\n",
                                 wc.enable ? "ON" : "off",
                                 w.steady[0], w.steady[1], w.steady[2],
                                 w.gust_amp, w.gust_period, w.turb_sigma);
                    break;
                }
                default:
                    break;
            }
        }

        // 2b) Apply motor-kill faults: a dead ESC produces no thrust regardless
        // of the commanded duty (also reflected in the pose frame's duty).
        for (int i = 0; i < 4; ++i)
            if (motor_kill[i]) duty[i] = 0.0f;

        // 3) Advance physics: `substeps` RK4 steps per sample, IMU sampled once.
        vsim::ImuSample s;
        if (!paused) {
            for (int sub = 0; sub < substeps; ++sub) ctl.stepOnce(duty, dt_sub);
            s = ctl.sampleImu(dt_sub);
            tick += static_cast<uint64_t>(substeps);
            // IMU dropout: freeze on the last good sample (stuck sensor) so the
            // frame keeps pacing the firmware but the reading no longer tracks.
            if (imu_dropout) s = imu_held;
            else imu_held = s;
            // Per-sensor dropout: a disabled sensor reports zero on its channel.
            if (!acc_enable) s.acc = vsim::Vec3(0.0f, 0.0f, 0.0f);
            if (!gyr_enable) s.gyr = vsim::Vec3(0.0f, 0.0f, 0.0f);
            if (!mag_enable) s.mag = vsim::Vec3(0.0f, 0.0f, 0.0f);
            if (state_log) {
                const vsim::Vec3& w = ctl.state().omega_b;   // true body rate
                std::fprintf(state_log,
                             "%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f\n",
                             tick / 8.0,                       // tick @8kHz -> ms
                             w.x() * 57.2958f, w.y() * 57.2958f, w.z() * 57.2958f,
                             duty[0], duty[1], duty[2], duty[3]);
                std::fflush(state_log);   // survive the harness SIGTERM/kill
            }
        }

        // 4) Emit IMU once per sample (imu_hz == the firmware loop rate).
        if (!paused) {
            vsim_imu_frame_t frame{};
            frame.hdr.magic         = VSIM_MAGIC;
            frame.hdr.version       = VSIM_PROTO_VERSION;
            frame.hdr.type          = VSIM_FRAME_IMU;
            frame.hdr.payload_bytes = VSIM_IMU_PAYLOAD_BYTES;
            frame.hdr.seq_no        = ++imu_seq;
            packImu(s, frame.imu_payload);
            imu_out.write(&frame, sizeof(frame));
        }

        // 5) Emit pose at pose_hz (every pose_div samples).
        if ((outer % pose_div) == 0) {
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
            const vsim::Vec3& vw = ctl.windWorld();   // sim-fidelity telemetry (Phase 1)
            frame.wind_w[0] = vw.x(); frame.wind_w[1] = vw.y(); frame.wind_w[2] = vw.z();
            pose_out.write(&frame, sizeof(frame));

            // 5b) Modelled barometer (BME280 analog). Derive static pressure from
            // the true altitude (NED down -> up) via the ISA formula — the exact
            // inverse of the firmware's altitude derivation, so the FC's own
            // bme280 path recovers this altitude. We emit PHYSICAL pressure only
            // (no altitude): the firmware does the altitude + telemetry, just as
            // the IMU feeder hands over physical IMU and lets the FC estimate.
            {
                constexpr double kSeaLevelPa = 101325.0;
                const double altitude_up = -static_cast<double>(st.pos_w.z());
                double ratio = 1.0 - altitude_up / 44330.0;
                if (ratio < 0.0) ratio = 0.0;  // guard absurd altitudes
                vsim_baro_frame_t bframe{};
                bframe.hdr.magic         = VSIM_MAGIC;
                bframe.hdr.version       = VSIM_PROTO_VERSION;
                bframe.hdr.type          = VSIM_FRAME_BARO;
                bframe.hdr.payload_bytes = sizeof(vsim_baro_frame_t) - sizeof(vsim_hdr_t);
                bframe.hdr.seq_no        = ++baro_seq;
                bframe.pressure_pa   = static_cast<float>(kSeaLevelPa * std::pow(ratio, 5.255));
                bframe.temperature_c = 25.0f;  // modelled cabin/air temperature
                bframe.humidity_rh   = 50.0f;  // modelled relative humidity
                baro_out.write(&bframe, sizeof(bframe));
            }
        }

        ++outer;
        if (!paused) ++ahead;   // one more IMU sample emitted this iteration

        if (lockstep) {
            // Backpressure: once we're `credit` samples ahead of the firmware's
            // last acknowledged PWM, wait for it to catch up before stepping on.
            // This bounds the IMU FIFO occupancy (no dropped samples) and keeps
            // vsim from outrunning the firmware. Watchdog-bounded so a stalled
            // (non-PWM-producing) firmware can never deadlock the sim.
            if (ahead >= ls_credit) {
                const auto deadline = clock::now() + ls_watchdog;
                while (!g_stop.load(std::memory_order_acquire)) {
                    if (ingest_pwm())
                        break;                       // firmware acknowledged → go
                    if (clock::now() > deadline) {
                        ahead = 0;                   // stalled → proceed, don't wedge
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                }
            }
        } else {
            next += outer_dur;
            auto now = clock::now();
            if (next > now) {
                std::this_thread::sleep_until(next);
            } else {
                // Fell behind. Resync without trying to catch up.
                next = now;
            }
        }
    }

    drop_world_mesh();
    std::fprintf(stderr, "vsim_d: stopping\n");
    return 0;
}
