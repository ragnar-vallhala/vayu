// world_mesh_transport_test.cpp — end-to-end Phase-2 transport check.
//
// Spawns the REAL vsim_d binary ($VSIM_BIN_PATH, default ./build_vsim/vsim_d),
// builds a floor BVH the same way the GCS does (WorldMeshBuilder), publishes it
// to the mmap file, ships VSIM_CTL_SET_WORLD_MESH over the ctl FIFO, then lets a
// thrustless drone fall from 2 m. If the mmap blob transport + daemon
// validation work, the drone rests on the imported floor (z≈0) instead of
// falling through to the far-below ground plane (ground_z=50). This is the only
// test that exercises the actual ctl frame + open/fstat/mmap/Bvh::fromBytes path
// against the shipped daemon; the collision math itself is covered by
// world_collision_test.cpp.
//
// No Qt: speaks the FIFO protocol directly (vsim_proto.h + fifo_transport.h) and
// builds the blob via the Qt-free WorldMeshBuilder bridge.
//
// build:
//   g++ -std=c++17 -I sim/vsim/include -I navigator/src/vsim \
//       sim/vsim/tests/world_mesh_transport_test.cpp \
//       sim/vsim/src/fifo_transport.cpp navigator/src/vsim/WorldMeshBuilder.cpp \
//       -o /tmp/wmtx && VSIM_BIN_PATH=build_vsim/vsim_d /tmp/wmtx
#include "WorldMeshBuilder.h"
#include "fifo_transport.h"
#include "vsim_proto.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <csignal>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

extern char** environ;

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        ++g_checks;                                                             \
        if (cond) std::printf("  ok   %s\n", (msg));                            \
        else { ++g_fails; std::printf("  FAIL %s\n", (msg)); }                  \
    } while (0)

static void msleep(int ms) {
    struct timespec ts{ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&ts, nullptr);
}

static std::string suffixed(const char* base) {
    const char* s = std::getenv("VSIM_FIFO_SUFFIX");
    return s ? std::string(base) + s : std::string(base);
}

static void fillHdr(vsim_ctl_frame_t& f, uint32_t subtype) {
    f = vsim_ctl_frame_t{};
    f.hdr.magic         = VSIM_MAGIC;
    f.hdr.version       = VSIM_PROTO_VERSION;
    f.hdr.type          = VSIM_FRAME_CTL;
    f.hdr.payload_bytes = sizeof(f) - sizeof(vsim_hdr_t);
    f.subtype           = subtype;
}

int main() {
    std::printf("== world-mesh transport test ==\n");

    // Per-run FIFO isolation so we never collide with a real Navigator.
    char suf[64];
    std::snprintf(suf, sizeof(suf), "_wmtx%d", (int)getpid());
    setenv("VSIM_FIFO_SUFFIX", suf, 1);

    // 1) Build a floor BVH (a big quad at z=0 = two triangles, soup of 6 verts)
    //    exactly as the GCS would, and publish it to the mmap file atomically.
    const float H = 50.0f;
    std::vector<float> v = {
        -H, -H, 0,   H, -H, 0,   H,  H, 0,   // tri 0
        -H, -H, 0,   H,  H, 0,  -H,  H, 0,   // tri 1
    };
    const uint32_t nverts = (uint32_t)(v.size() / 3);  // 6
    uint32_t nodes = 0;
    std::vector<uint8_t> blob = vsim::buildWorldBvh(v.data(), nverts, true, nodes);
    CHECK(!blob.empty() && nodes > 0, "GCS-side BVH blob built");

    const std::string path = suffixed("/tmp/vsim_world") + ".bin";
    const std::string tmp  = path + ".tmp";
    {
        int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        bool ok = fd >= 0 &&
                  ::write(fd, blob.data(), blob.size()) == (ssize_t)blob.size();
        if (fd >= 0) ::close(fd);
        ::rename(tmp.c_str(), path.c_str());
        CHECK(ok, "blob written + renamed into place");
    }

    // 2) Spawn the real daemon (inherits VSIM_FIFO_SUFFIX via environ).
    const char* bin = std::getenv("VSIM_BIN_PATH");
    if (!bin) bin = "build_vsim/vsim_d";
    pid_t pid = -1;
    char* argv[] = {const_cast<char*>(bin), nullptr};
    if (posix_spawn(&pid, bin, nullptr, nullptr, argv, environ) != 0) {
        std::printf("  FAIL could not spawn %s\n", bin);
        return 1;
    }
    msleep(300);  // let it create/open its FIFOs

    vsim::FifoOut ctl(suffixed(VSIM_FIFO_CTL));
    vsim::FifoIn  pose(suffixed(VSIM_FIFO_POSE));
    ctl.open();
    pose.open();

    // 3) World: earth gravity, ground plane 50 m BELOW the mesh floor.
    {
        vsim_ctl_frame_t f; fillHdr(f, VSIM_CTL_SET_WORLD);
        vsim_ctl_world_t w{};
        w.gravity = 9.81f; w.ground_z = 50.0f; w.restitution = 0.0f;
        std::memcpy(f.body, &w, sizeof(w));
        ctl.write(&f, sizeof(f));
    }
    // 4) Drop the drone in from 2 m above the floor.
    {
        vsim_ctl_frame_t f; fillHdr(f, VSIM_CTL_RESET);
        vsim_ctl_reset_t r{};
        r.pos_w[2] = -2.0f; r.quat_wxyz[0] = 1.0f;
        std::memcpy(f.body, &r, sizeof(r));
        ctl.write(&f, sizeof(f));
    }
    // 5) Ship the world mesh.
    {
        vsim_ctl_frame_t f; fillHdr(f, VSIM_CTL_SET_WORLD_MESH);
        vsim_ctl_world_mesh_t m{};
        m.vertex_count = nverts; m.triangle_count = nverts / 3;
        m.node_count = nodes; m.flags = 1u; m.restitution = 0.0f;
        m.path_len = (uint32_t)path.size();
        std::strncpy(m.path, path.c_str(), sizeof(m.path) - 1);
        std::memcpy(f.body, &m, sizeof(m));
        bool wrote = ctl.write(&f, sizeof(f));
        CHECK(wrote, "SET_WORLD_MESH ctl frame sent");
    }

    // 6) Read pose for ~3 s; track the last z. Drone falls then rests on mesh.
    int frames = 0;
    float last_z = -2.0f;
    for (int i = 0; i < 300; ++i) {
        vsim_pose_frame_t pf;
        if (pose.poll(VSIM_FRAME_POSE, &pf, sizeof(pf))) {
            ++frames;
            last_z = pf.pos_w[2];
        }
        msleep(10);
    }

    std::printf("  pose frames=%d, final z=%.3f (mesh floor 0, ground 50)\n",
                frames, last_z);
    CHECK(frames > 50, "pose stream healthy");
    CHECK(last_z > -0.5f && last_z < 1.0f, "drone rested on the imported mesh floor");
    CHECK(last_z < 10.0f, "drone did NOT fall through to the ground plane (50)");

    // 7) Tear down.
    if (pid > 0) { ::kill(pid, SIGTERM); int st; ::waitpid(pid, &st, 0); }
    ::unlink(path.c_str());

    std::printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
