// Heightfield.h — a square, regularly-sampled 2D height grid in NED.
//
// Stores terrain elevation `h` (metres above the z=0 ground plane) on an n x n
// lattice spanning [-sizeM/2, +sizeM/2] in both X (north) and Y (east). Terrain
// up is -Z, so a sample of height h sits at world z = -h. Header-only: it's a
// plain data holder with bilinear sampling and an analytic surface normal, all
// cheap and worth inlining.
#pragma once

#include "ProcgenTypes.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace vsim::procgen {

struct Heightfield {
  int n = 0;            // samples per side (n x n)
  float sizeM = 0.0f;   // world extent of one side [m], centred on the origin
  std::vector<float> h; // row-major heights [m], length n*n; index = iy*n + ix

  bool valid() const {
    return n >= 2 && static_cast<std::size_t>(n) * n == h.size();
  }

  // World metres covered by one grid step.
  float spacing() const { return n > 1 ? sizeM / static_cast<float>(n - 1) : 0.0f; }

  // Height at clamped integer lattice coords.
  float at(int ix, int iy) const {
    if (ix < 0) ix = 0; else if (ix >= n) ix = n - 1;
    if (iy < 0) iy = 0; else if (iy >= n) iy = n - 1;
    return h[static_cast<std::size_t>(iy) * n + ix];
  }

  // World position (NED metres) of lattice node (ix, iy). z = -height (up).
  PgVec3 worldAt(int ix, int iy) const {
    const float half = sizeM * 0.5f;
    const float s = spacing();
    return PgVec3{-half + s * static_cast<float>(ix),
                  -half + s * static_cast<float>(iy),
                  -at(ix, iy)};
  }

  // Bilinear height at continuous world (wx, wy). Outside the grid clamps to the
  // edge. Used by a future heightfield collider and by scatter placement.
  float sampleWorld(float wx, float wy) const {
    if (!valid()) return 0.0f;
    const float half = sizeM * 0.5f;
    const float s = spacing();
    const float gx = (wx + half) / s;
    const float gy = (wy + half) / s;
    int ix = static_cast<int>(std::floor(gx));
    int iy = static_cast<int>(std::floor(gy));
    const float fx = gx - static_cast<float>(ix);
    const float fy = gy - static_cast<float>(iy);
    const float h00 = at(ix, iy);
    const float h10 = at(ix + 1, iy);
    const float h01 = at(ix, iy + 1);
    const float h11 = at(ix + 1, iy + 1);
    const float a = h00 + (h10 - h00) * fx;
    const float b = h01 + (h11 - h01) * fx;
    return a + (b - a) * fy;
  }

  // Outward (up-ish, NED -Z) unit surface normal at lattice node (ix, iy), from
  // central differences of the height field. Surface is z = -h(x,y), so the
  // (unnormalised) outward normal is (dh/dx, dh/dy, -1).
  PgVec3 normalAt(int ix, int iy) const {
    const float s = spacing();
    const float inv2s = s > 0.0f ? 1.0f / (2.0f * s) : 0.0f;
    const float dhdx = (at(ix + 1, iy) - at(ix - 1, iy)) * inv2s;
    const float dhdy = (at(ix, iy + 1) - at(ix, iy - 1)) * inv2s;
    float nx = dhdx, ny = dhdy, nz = -1.0f;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 0.0f) { nx /= len; ny /= len; nz /= len; }
    return PgVec3{nx, ny, nz};
  }
};

}  // namespace vsim::procgen
