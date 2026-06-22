// Flora.h — procedural grass scatter for the terrain (Qt-free).
//
// Places grass-blade instances on a terrain chunk: a jittered global grid
// (seamless across chunks), sampling the field for height/slope so blades sit on
// the surface and only grow on grassy, not-too-steep, not-too-high ground. The
// renderer draws one instanced blade mesh per chunk from these instances with
// wind + lighting + fog. No imported assets — blades are procedural geometry.
#pragma once

#include "ProcgenTypes.h"
#include "TerrainField.h"

#include <cstdint>
#include <vector>

namespace vsim::procgen {

// One grass blade. The renderer expands a shared blade mesh per instance:
// translate to pos, rotate by yaw about the vertical, scale height, tint.
struct FloraInstance {
  PgVec3 pos;    // base on the surface, NED world metres
  float yaw;     // blade orientation [rad]
  float height;  // blade height [m]
  PgVec3 tint;   // blade colour (green variants; brighter = flower accents)
  float flower;  // 0 = grass blade, 1 = flower (renderer widens + brightens top)
};

struct FloraParams {
  uint32_t seed = 1337u;
  float spacing = 1.6f;        // grid spacing between blades [m]
  float jitter = 0.85f;        // positional jitter (fraction of spacing)
  float maxSlope = 0.32f;      // skip where (1 - flatness) exceeds this
  float grassMaxFrac = 0.55f;  // grass only below this fraction of heightM
  float minHeight = 0.22f;     // blade height range [m]
  float maxHeight = 0.55f;
  float flowerFrac = 0.05f;    // fraction of blades that become flowers
};

// Scatter blades over chunk (cx, cy) spanning [cx*chunkM,(cx+1)*chunkM]^2.
// Deterministic from p.seed; the global grid makes neighbouring chunks tile
// without gaps or doubles.
std::vector<FloraInstance> scatterFlora(const TerrainField& f, int cx, int cy,
                                        float chunkM, const FloraParams& p);

}  // namespace vsim::procgen
