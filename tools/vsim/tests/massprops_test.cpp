// Unit test for vsim::computeMassProperties + vsim::Mat3.
//
// Validates the polyhedral mass-properties integral against closed-form
// inertia of a solid box, plus the Mat3 inverse, with no GUI/daemon.
//
// Build (see harness in this dir / the build commands): links Qt6::Gui
// for QVector3D and compiles MassProperties.cpp directly.
//
// Exit 0 on pass.

#include "MassProperties.h"
#include "MeshLoader.h"
#include "vsim_math.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using vsim::Mat3;
using vsim::computeMassProperties;

static int g_fail = 0;
static void check(bool ok, const char* what, double got, double want) {
  if (!ok) {
    std::printf("  FAIL %-22s got %.6g want %.6g\n", what, got, want);
    ++g_fail;
  } else {
    std::printf("  ok   %-22s %.6g\n", what, got);
  }
}
static bool close(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Build a solid box (triangle soup, outward CCW winding) of dims
// lx,ly,lz centered at `c`.
static std::vector<QVector3D> box(float lx, float ly, float lz, QVector3D c) {
  const float hx = lx / 2, hy = ly / 2, hz = lz / 2;
  QVector3D v[8] = {
      {c.x()-hx, c.y()-hy, c.z()-hz}, {c.x()+hx, c.y()-hy, c.z()-hz},
      {c.x()+hx, c.y()+hy, c.z()-hz}, {c.x()-hx, c.y()+hy, c.z()-hz},
      {c.x()-hx, c.y()-hy, c.z()+hz}, {c.x()+hx, c.y()-hy, c.z()+hz},
      {c.x()+hx, c.y()+hy, c.z()+hz}, {c.x()-hx, c.y()+hy, c.z()+hz},
  };
  // 12 triangles, outward-facing.
  int idx[36] = {
      0,3,2, 0,2,1,   // -z
      4,5,6, 4,6,7,   // +z
      0,1,5, 0,5,4,   // -y
      2,3,7, 2,7,6,   // +y
      1,2,6, 1,6,5,   // +x
      0,4,7, 0,7,3,   // -x
  };
  std::vector<QVector3D> out;
  for (int i = 0; i < 36; ++i) out.push_back(v[idx[i]]);
  return out;
}

int main(int argc, char** argv) {
  int rc = 0;

  // --- Cube: side a, mass m -> I = (1/6) m a^2 on diagonal, products 0.
  {
    std::printf("[cube centered at origin]\n");
    const float a = 0.4f, m = 2.0f;
    auto mp = computeMassProperties(box(a, a, a, {0, 0, 0}), m);
    const double I = (1.0 / 6.0) * m * a * a;
    check(mp.valid, "valid", mp.valid, 1);
    check(close(mp.volume, a*a*a, 1e-6), "volume", mp.volume, a*a*a);
    check(close(mp.com.length(), 0, 1e-5), "com~0", mp.com.length(), 0);
    check(close(mp.ixx, I, 1e-5), "Ixx", mp.ixx, I);
    check(close(mp.iyy, I, 1e-5), "Iyy", mp.iyy, I);
    check(close(mp.izz, I, 1e-5), "Izz", mp.izz, I);
    check(close(mp.ixy, 0, 1e-5) && close(mp.ixz, 0, 1e-5) && close(mp.iyz, 0, 1e-5),
          "products~0", mp.ixy, 0);
  }

  // --- Offset box: CoM should land at the offset; inertia about CoM
  //     matches the centered case (parallel-axis handled internally).
  {
    std::printf("[box 0.2x0.6x0.4 offset by (1,-2,0.5)]\n");
    const float lx=0.2f, ly=0.6f, lz=0.4f, m=3.0f;
    QVector3D c(1.0f, -2.0f, 0.5f);
    auto mp = computeMassProperties(box(lx, ly, lz, c), m);
    const double Ixx = (1.0/12.0)*m*(ly*ly + lz*lz);
    const double Iyy = (1.0/12.0)*m*(lx*lx + lz*lz);
    const double Izz = (1.0/12.0)*m*(lx*lx + ly*ly);
    check(close((mp.com - c).length(), 0, 1e-4), "com at offset", (mp.com-c).length(), 0);
    check(close(mp.ixx, Ixx, 1e-5), "Ixx", mp.ixx, Ixx);
    check(close(mp.iyy, Iyy, 1e-5), "Iyy", mp.iyy, Iyy);
    check(close(mp.izz, Izz, 1e-5), "Izz", mp.izz, Izz);
    check(close(mp.ixy, 0, 1e-5) && close(mp.ixz, 0, 1e-5) && close(mp.iyz, 0, 1e-5),
          "products~0 (axis-aligned)", mp.ixy, 0);
  }

  // --- Mat3 inverse: I * I^-1 == identity for a non-diagonal tensor.
  {
    std::printf("[Mat3 inverse]\n");
    Mat3 A = Mat3::symmetric(0.02f, 0.03f, 0.04f, 0.005f, -0.003f, 0.002f);
    Mat3 P = A * A.inverse();
    bool ok = true;
    for (int r = 0; r < 3; ++r)
      for (int col = 0; col < 3; ++col) {
        const float want = (r == col) ? 1.0f : 0.0f;
        if (std::fabs(P.at(r, col) - want) > 1e-4f) ok = false;
      }
    check(ok, "A*inv(A)=I", ok, 1);
  }

  // --- Real mesh import: load an STL via MeshLoader (assimp) and check
  //     its computed volume against the known box volume.
  if (argc > 1) {
    std::printf("[mesh import: %s]\n", argv[1]);
    QString err;
    vsim::LoadedMesh mesh = vsim::loadMesh(QString::fromUtf8(argv[1]), 1.0f, &err);
    check(mesh.valid, "loaded", mesh.valid, 1);
    if (mesh.valid) {
      auto mp = computeMassProperties(mesh.positions, 1.0f);
      // box.stl is 0.35 x 0.35 x 0.06 -> 0.00735 m^3.
      check(close(mp.volume, 0.00735, 1e-4), "stl volume", mp.volume, 0.00735);
      check(close(mp.com.length(), 0, 1e-4), "stl com~0", mp.com.length(), 0);
    }
  }

  rc = (g_fail == 0) ? 0 : 1;
  std::printf("%s (%d failures)\n", rc == 0 ? "PASS" : "FAIL", g_fail);
  return rc;
}
