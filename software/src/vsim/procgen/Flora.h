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
  float spacing = 0.20f;       // grid spacing between blades [m] (dense meadow)
  float jitter = 0.9f;         // positional jitter (fraction of spacing)
  float grassMaxFrac = 0.5f;   // grass fades out by this fraction of heightM
  // Slope (flatness in [0,1], 1 = flat) where grass density ramps in: none below
  // slopeLo, full above slopeHi. Lower = grass tolerates steeper ground.
  float slopeLo = 0.80f;
  float slopeHi = 0.93f;
  // Extra blades scattered per grid cell — multiplies density much more cheaply
  // than shrinking the spacing (the field grid is sampled once per cell).
  float bladesPerCell = 2.0f;  // carpet density (camera-facing fraction fills gaps)
  float heightMean = 1.4f;     // blade height: normal distribution [m]
  float heightStdDev = 0.3f;
  float flowerFrac = 0.01f;    // sparse flowers (more reads as litter in a dense field)
};

// Live look/shading knobs for the GPU grass + terrain lighting. These drive
// shader uniforms (per-frame), so edits take effect immediately — no terrain
// regeneration needed. Defaults reproduce the tuned Ghost-of-Tsushima look.
struct GrassLook {
  // Lighting — shared by grass and terrain.
  float sunIntensity = 1.0f;     // multiplies the directional sun (key light)
  float ambientStrength = 1.0f;  // multiplies the sky/hemisphere ambient fill
  // Grass shading.
  float brightness = 1.0f;       // multiplies grass albedo (overall green level)
  float tipWarmth = 0.55f;       // yellow-green warming toward the lit tips
  float backlight = 1.0f;        // subsurface backlight (SSS) when sun is behind
  float sheen = 1.0f;            // waxy specular sheen along lit blades
  float veinStrength = 1.0f;     // midrib highlight + curled-edge shade (the vein)
  float rootDarkness = 0.14f;    // ambient-occlusion floor at the blade base (lower = darker)
  float faceCameraFrac = 0.35f;  // fraction of blades turned broadside to the camera
};

// Scatter blades over chunk (cx, cy) spanning [cx*chunkM,(cx+1)*chunkM]^2.
// Deterministic from p.seed; the global grid makes neighbouring chunks tile
// without gaps or doubles.
std::vector<FloraInstance> scatterFlora(const TerrainField& f, int cx, int cy,
                                        float chunkM, const FloraParams& p);

}  // namespace vsim::procgen
