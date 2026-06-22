// TerrainGen.h — procedural terrain: parameters -> heightfield -> colored mesh.
//
// Phase 0 of the procedural-world effort. Produces a triangle-soup ProcMesh in
// NED world metres with per-vertex normals and a meadow color splat, ready to
// feed the existing setWorldMesh (render) and buildWorldBvh (collision) paths
// unchanged. Pure data, no Qt, no GL — fully unit-testable.
#pragma once

#include "Heightfield.h"
#include "ProcgenTypes.h"

#include <cstdint>

namespace vsim::procgen {

// Knobs for the terrain field. Defaults give a ~256 m rolling meadow ringed by
// higher ground, peaks ~35 m. All generation is a pure function of these, so
// the same struct reproduces the same world.
struct TerrainParams {
  uint32_t seed = 1337u;
  float sizeM = 256.0f;     // square extent of the world [m], centred on origin
  int resolution = 256;     // grid samples per side (>=2)

  float featureM = 150.0f;  // world metres of the largest terrain feature
  float heightM = 80.0f;    // peak (mountain-rim) elevation above ground [m]
  int octaves = 6;          // fBm/ridged detail layers
  float lacunarity = 2.0f;  // frequency step per octave
  float gain = 0.5f;        // amplitude step per octave

  float mountainMix = 0.6f;   // weight of ridged peaks in the mountainous rim
  float valleyDepth = 0.5f;   // [0,1] larger = wider central meadow floor
};

// Build just the heightfield (elevation grid). Separated out so a future
// heightfield collider and the scatter system can share it with the mesher.
Heightfield generateHeightfield(const TerrainParams& p);

// Triangulate a heightfield into a triangle soup with per-vertex normals and a
// meadow color splat (green valley floor -> olive slopes -> rock on steep/high
// ground). Two triangles per grid cell.
ProcMesh meshFromHeightfield(const Heightfield& hf, const TerrainParams& p);

// Convenience: heightfield + mesh + meadow splat in one call.
ProcMesh generateMeadow(const TerrainParams& p);

}  // namespace vsim::procgen
