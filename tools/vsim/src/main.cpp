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
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

constexpr int kPhysicsHz = 1000;
constexpr int kImuHz     = 200;
constexpr int kPoseHz    = 60;
constexpr int kImuDiv    = kPhysicsHz / kImuHz;     // 5
constexpr int kPoseDiv   = kPhysicsHz / kPoseHz;    // ~16

constexpr float kRad2Deg = 57.29577951308232f;

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true, std::memory_order_release); }

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

    vsim::FifoIn  pwm_in (VSIM_FIFO_PWM);
    vsim::FifoOut imu_out(VSIM_FIFO_IMU);
    vsim::FifoOut pose_out(VSIM_FIFO_POSE);
    vsim::FifoIn  ctl_in (VSIM_FIFO_CTL);

    if (!pwm_in.open() || !imu_out.open() || !pose_out.open() || !ctl_in.open()) {
        std::fprintf(stderr, "vsim_d: FIFO setup failed\n");
        return 1;
    }
    std::fprintf(stderr, "vsim_d: running at %d Hz physics, %d Hz IMU, %d Hz pose\n",
                 kPhysicsHz, kImuHz, kPoseHz);

    vsim::SimController ctl;
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
                duty[i] = std::clamp(pwm.duty[i], 0.0f, 1.0f);
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
