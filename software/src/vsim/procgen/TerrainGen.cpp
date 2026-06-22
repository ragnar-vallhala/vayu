#include "TerrainGen.h"

#include "Noise.h"

#include <algorithm>
#include <cmath>

namespace vsim::procgen {
namespace {

inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// Hermite smoothstep, edge0 < edge1.
inline float smoothstep(float e0, float e1, float x) {
  const float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}

inline PgVec3 mix(const PgVec3& a, const PgVec3& b, float t) {
  return PgVec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t};
}

// Meadow palette, blended by normalised elevation and slope. Kept as a small
// table here in Phase 0; when materials become data-driven (Phase 1) this moves
// into the biome recipe's MaterialLayers.
PgVec3 meadowColor(float normHeight, float flatness) {
  // Elevation ramp: grass -> olive/dry grass -> bare rock near the tops.
  const PgVec3 grass{0.27f, 0.44f, 0.16f};
  const PgVec3 olive{0.46f, 0.49f, 0.24f};
  const PgVec3 rock {0.42f, 0.38f, 0.33f};
  PgVec3 c = normHeight < 0.5f ? mix(grass, olive, normHeight * 2.0f)
                               : mix(olive, rock, (normHeight - 0.5f) * 2.0f);
  // Steep faces read as exposed rock/dirt regardless of altitude. flatness is
  // 1 on level ground, 0 on a vertical wall.
  const float rockiness = clamp01((0.78f - flatness) * 3.0f);
  return mix(c, PgVec3{0.36f, 0.31f, 0.26f}, rockiness);
}

}  // namespace

Heightfield generateHeightfield(const TerrainParams& p) {
  Heightfield hf;
  hf.n = std::max(2, p.resolution);
  hf.sizeM = p.sizeM;
  hf.h.assign(static_cast<std::size_t>(hf.n) * hf.n, 0.0f);

  // Three decorrelated noise fields from the one seed: rolling hills, ridged
  // mountains, and a low-frequency domain warp that bends features so nothing
  // looks grid-aligned.
  const Noise hills(p.seed);
  const Noise mtn(p.seed ^ 0x9e3779b9u);
  const Noise warp(p.seed ^ 0x68bc21ebu);

  // Lowest octave spans `featureM` metres per cycle.
  const float baseFreq = p.featureM > 0.0f ? 1.0f / p.featureM : 1.0f / 170.0f;
  const float half = p.sizeM * 0.5f;

  for (int iy = 0; iy < hf.n; ++iy) {
    for (int ix = 0; ix < hf.n; ++ix) {
      const PgVec3 w = hf.worldAt(ix, iy);  // z unused here (still 0)
      const float nx = w.x * baseFreq;
      const float ny = w.y * baseFreq;

      // Domain warp: offset the lookup by a slow noise field.
      const float wx = nx + 0.6f * warp.fbm(nx * 0.5f, ny * 0.5f, 3, 2.0f, 0.5f);
      const float wy = ny + 0.6f * warp.fbm(nx * 0.5f + 5.2f,
                                            ny * 0.5f + 1.3f, 3, 2.0f, 0.5f);

      const float roll = hills.fbm(wx, wy, p.octaves, p.lacunarity, p.gain);
      const float rollUnit = roll * 0.5f + 0.5f;            // [0,1] gentle hills
      const float ridge = mtn.ridged(wx, wy, p.octaves, p.lacunarity, p.gain);
      const float mtnUnit = ridge * ridge;                 // [0,1] sharp peaks

      // Compose an open meadow floor that rises into a mountainous rim. `rim`
      // is 0 across the central valley and ramps to 1 toward the edges; a wider
      // valleyDepth pushes the mountains further out (bigger meadow).
      const float r =
          std::sqrt(w.x * w.x + w.y * w.y) / (half > 0.0f ? half : 1.0f);
      const float rimStart = 0.15f + 0.45f * clamp01(p.valleyDepth);
      const float rim = smoothstep(rimStart, rimStart + 0.5f, r);

      // Meadow: only gentle undulation. Rim: ridged mountains plus some hill
      // mass so the slopes aren't bare. Normalised so the tallest peaks reach
      // ~heightM and the valley floor sits a few metres up.
      const float elev = 0.12f * rollUnit +
                         rim * (p.mountainMix * mtnUnit + 0.3f * rollUnit);

      hf.h[static_cast<std::size_t>(iy) * hf.n + ix] =
          clamp01(elev) * p.heightM;
    }
  }
  return hf;
}

ProcMesh meshFromHeightfield(const Heightfield& hf, const TerrainParams& p) {
  ProcMesh m;
  if (!hf.valid()) return m;

  // Precompute per-node position, normal, and color, then emit a triangle soup
  // (two tris per cell, vertices duplicated — matches LoadedMesh / BVH input).
  const int n = hf.n;
  const float invH = p.heightM > 0.0f ? 1.0f / p.heightM : 1.0f;

  std::vector<PgVec3> pos(static_cast<std::size_t>(n) * n);
  std::vector<PgVec3> nrm(static_cast<std::size_t>(n) * n);
  std::vector<PgVec3> col(static_cast<std::size_t>(n) * n);
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      const std::size_t i = static_cast<std::size_t>(iy) * n + ix;
      pos[i] = hf.worldAt(ix, iy);
      const PgVec3 nv = hf.normalAt(ix, iy);
      nrm[i] = nv;
      const float normH = clamp01(hf.at(ix, iy) * invH);
      const float flatness = clamp01(-nv.z);  // NED up is -Z; flat -> nz≈-1
      col[i] = meadowColor(normH, flatness);
    }
  }

  const std::size_t cells = static_cast<std::size_t>(n - 1) * (n - 1);
  m.positions.reserve(cells * 6);
  m.normals.reserve(cells * 6);
  m.colors.reserve(cells * 6);

  auto push = [&](std::size_t i) {
    m.positions.push_back(pos[i]);
    m.normals.push_back(nrm[i]);
    m.colors.push_back(col[i]);
  };

  for (int iy = 0; iy < n - 1; ++iy) {
    for (int ix = 0; ix < n - 1; ++ix) {
      const std::size_t i00 = static_cast<std::size_t>(iy) * n + ix;
      const std::size_t i10 = i00 + 1;
      const std::size_t i01 = i00 + n;
      const std::size_t i11 = i01 + 1;
      // Winding is CCW seen from above (-Z, the up side in NED).
      push(i00); push(i10); push(i11);
      push(i00); push(i11); push(i01);
    }
  }
  return m;
}

ProcMesh generateMeadow(const TerrainParams& p) {
  return meshFromHeightfield(generateHeightfield(p), p);
}

}  // namespace vsim::procgen
