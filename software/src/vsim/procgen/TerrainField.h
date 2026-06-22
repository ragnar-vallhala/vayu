// TerrainField.h — an infinite, translation-invariant terrain function.
//
// Where TerrainGen builds one finite origin-centred arena, TerrainField is a
// pure function height(worldX, worldY) defined over ALL of space, with no
// centre. It's the basis for endless streaming terrain: a chunk is just this
// field sampled over a square world-coordinate window. Because every chunk
// samples the SAME global field (and normals are finite differences of it),
// adjacent chunks meet seamlessly with no stitching — shared edge coordinates
// produce identical positions, normals, and colors.
//
// Macro composition: a slow noise field decides where mountain ranges sit vs.
// open meadow basins, so terrain stays varied forever instead of repeating one
// bowl. Qt-free and deterministic from one seed.
#pragma once

#include "Noise.h"
#include "ProcgenTypes.h"

#include <cstdint>

namespace vsim::procgen {

struct FieldParams {
  uint32_t seed = 1337u;
  float heightM = 80.0f;     // peak (mountain) elevation above ground [m]
  float featureM = 150.0f;   // metres of the largest detail feature
  float macroM = 1400.0f;    // metres of the macro range/basin placement field
  int octaves = 6;
  float lacunarity = 2.0f;
  float gain = 0.5f;
  float mountainMix = 0.6f;  // ridged-peak weight inside mountainous regions
};

class TerrainField {
 public:
  explicit TerrainField(const FieldParams& p);

  // Elevation [m] above the ground plane at world (wx, wy). Always >= 0.
  float height(float wx, float wy) const;

  // Outward unit surface normal (NED, up is -Z) from central differences of
  // height() with step `eps` (use the mesh vertex spacing so normals match the
  // facets). Sampling the global field means chunk edges agree automatically.
  PgVec3 normal(float wx, float wy, float eps) const;

  // Meadow color splat for elevation `h` (metres) and `flatness` in [0,1]
  // (1 = level ground, 0 = vertical). Green valley -> olive -> rock.
  PgVec3 color(float h, float flatness) const;

  const FieldParams& params() const { return p_; }

 private:
  FieldParams p_;
  Noise hills_;
  Noise mtn_;
  Noise macro_;
  Noise warp_;
  float baseFreq_;  // 1 / featureM
  float macroFreq_; // 1 / macroM
};

// Sample `f` over the square world window of chunk (cx, cy) and triangulate it
// into a triangle soup (NED metres, per-vertex normals + meadow colors).
// Chunk (cx, cy) spans [cx*chunkM, (cx+1)*chunkM] x [cy*chunkM, (cy+1)*chunkM].
// `res` is grid cells per side; vertices on shared edges land on identical world
// coordinates as the neighbour chunk, so meshes tile seamlessly.
ProcMesh meshFieldChunk(const TerrainField& f, int cx, int cy, float chunkM,
                        int res);

}  // namespace vsim::procgen
