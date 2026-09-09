// ProcgenTypes.h — Qt-free POD types shared across the procedural-generation
// core (noise / heightfield / terrain).
//
// The whole procgen/ module is deliberately Qt-free: plain structs + std::vector
// only, so it unit-tests without a GL context or QApplication and could later be
// shared with sim/vsim/. The thin Qt adapter that turns a ProcMesh into a
// vsim::LoadedMesh lives outside this directory (ProceduralWorld.cpp).
#pragma once

#include <cstddef>
#include <cstdint> // ensure int32_t et al. are defined before <cmath>/<cstdlib>
#include <vector>

namespace vsim::procgen {

// Minimal 3-vector. Coordinates are NED world metres (X north, Y east, Z down)
// once a mesh is assembled; terrain "up" is therefore -Z.
struct PgVec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

// A triangle soup: positions.size() == 3 * triangleCount(), grouped per
// triangle (matches vsim::LoadedMesh and the trimesh BVH input). normals and
// colors are matching per-vertex arrays (colors are RGB in [0,1]).
struct ProcMesh {
  std::vector<PgVec3> positions;
  std::vector<PgVec3> normals;
  std::vector<PgVec3> colors;

  std::size_t triangleCount() const { return positions.size() / 3; }
  bool empty() const { return positions.empty(); }
};

} // namespace vsim::procgen
