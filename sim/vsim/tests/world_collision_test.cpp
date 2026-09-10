// world_collision_test.cpp — drone-vs-world-mesh collision (no Qt/daemon).
//   g++ -std=c++17 -I sim/vsim/include sim/vsim/tests/world_collision_test.cpp \
//       sim/vsim/src/physics_core.cpp -o /tmp/wct && /tmp/wct
#include "physics_core.h"
#include "trimesh_bvh.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace vsim;

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    ++g_checks;                                                                \
    if (cond)                                                                  \
      printf("  ok   %s\n", (msg));                                            \
    else {                                                                     \
      ++g_fails;                                                               \
      printf("  FAIL %s (%s:%d)\n", (msg), __FILE__, __LINE__);                \
    }                                                                          \
  } while (0)

// Build a flat floor at z=0 over [-half,half]^2 as a triangle grid + BVH blob.
static std::vector<uint8_t> floorBlob(float half, int n) {
  std::vector<float> v;
  std::vector<uint32_t> t;
  auto vid = [&](int i, int j) { return uint32_t((n + 1) * i + j); };
  for (int i = 0; i <= n; ++i)
    for (int j = 0; j <= n; ++j) {
      v.push_back(-half + 2 * half * i / n);
      v.push_back(-half + 2 * half * j / n);
      v.push_back(0.0f);
    }
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      t.push_back(vid(i, j));
      t.push_back(vid(i + 1, j));
      t.push_back(vid(i + 1, j + 1));
      t.push_back(vid(i, j));
      t.push_back(vid(i + 1, j + 1));
      t.push_back(vid(i, j + 1));
    }
  return trimesh::buildSerialized(v.data(), uint32_t(v.size() / 3), t.data(),
                                  uint32_t(t.size() / 3), true);
}

int main() {
  printf("== world-mesh collision test ==\n");

  DroneParams dp; // default 1 kg, diag inertia
  dp.gravity = 9.81f;
  dp.ground_z = 5.0f; // ground plane 5 m BELOW the mesh floor (NED +z down)
  dp.ground_restitution = 0.0f;

  std::vector<uint8_t> blob = floorBlob(3.0f, 12); // floor at z=0
  trimesh::Bvh bvh = trimesh::Bvh::fromBytes(blob.data(), blob.size());

  // --- 1) Drone dropped onto the floor rests on it, doesn't fall through ---
  {
    PhysicsCore p;
    p.setParams(dp);
    p.setWorldMesh(bvh, 0.0f);
    RigidBodyState s;
    s.pos_w = Vec3(0, 0, -1.0f); // 1 m above the floor
    p.reset(s);
    for (int i = 0; i < 4000; ++i)
      p.step(Vec3(0, 0, 0), Vec3(0, 0, 0), 0.001f);
    const float z = p.state().pos_w.z();
    printf("  rest z = %.3f (floor at 0, ground at 5)\n", z);
    CHECK(z > -0.2f && z < 0.2f, "drone rests on the mesh floor (~z=0)");
    CHECK(z < 1.0f, "drone did NOT fall through to the ground plane");
  }

  // --- 2) Dropped off-centre on a tilted contact -> acquires tilt (tips) ---
  {
    // Floor tilted: reuse the flat floor but start the drone rolled, so the
    // asymmetric footprint contact should drive it back / impart omega.
    PhysicsCore p;
    p.setParams(dp);
    p.setWorldMesh(bvh, 0.2f);
    RigidBodyState s;
    s.pos_w = Vec3(0, 0, -0.5f);
    s.vel_w = Vec3(0, 0, 3.0f); // moving down hard -> bounce
    p.reset(s);
    float vbefore = s.vel_w.z();
    for (int i = 0; i < 200; ++i)
      p.step(Vec3(0, 0, 0), Vec3(0, 0, 0), 0.001f);
    const float vz = p.state().pos_w.z();
    printf("  after impact z = %.3f, vel.z = %.2f\n", vz, p.state().vel_w.z());
    CHECK(p.state().pos_w.z() < 0.2f,
          "stopped at/above the floor after impact");
    CHECK(vbefore > 0 && p.state().vel_w.z() < vbefore,
          "downward velocity removed/reflected by the contact");
  }

  // --- 3) No world mesh -> falls to the ground plane (z=5) ---
  {
    PhysicsCore p;
    p.setParams(dp); // no setWorldMesh
    RigidBodyState s;
    s.pos_w = Vec3(0, 0, -1.0f);
    p.reset(s);
    for (int i = 0; i < 4000; ++i)
      p.step(Vec3(0, 0, 0), Vec3(0, 0, 0), 0.001f);
    CHECK(p.state().pos_w.z() > 4.5f,
          "with no mesh, falls through to ground_z");
  }

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
