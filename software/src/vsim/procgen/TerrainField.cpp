#include "TerrainField.h"

#include <algorithm>
#include <cmath>

namespace vsim::procgen {
namespace {

inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

inline float smoothstep(float e0, float e1, float x) {
  const float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}

inline PgVec3 mix(const PgVec3& a, const PgVec3& b, float t) {
  return PgVec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t};
}

}  // namespace

TerrainField::TerrainField(const FieldParams& p)
    : p_(p),
      hills_(p.seed),
      mtn_(p.seed ^ 0x9e3779b9u),
      macro_(p.seed ^ 0x68bc21ebu),
      warp_(p.seed ^ 0xb5297a4du),
      baseFreq_(p.featureM > 0.0f ? 1.0f / p.featureM : 1.0f / 150.0f),
      macroFreq_(p.macroM > 0.0f ? 1.0f / p.macroM : 1.0f / 1400.0f) {}

float TerrainField::height(float wx, float wy) const {
  // Macro field: where mountains live (high) vs. open meadow basins (low). Slow,
  // so ranges and valleys span hundreds of metres and never repeat near you.
  const float mx = wx * macroFreq_;
  const float my = wy * macroFreq_;
  const float macro = macro_.fbm(mx, my, 3, 2.0f, 0.5f) * 0.5f + 0.5f;  // [0,1]
  const float mountainAmount = smoothstep(0.40f, 0.68f, macro);

  // Detail bands. Domain-warp so nothing looks grid-aligned.
  const float nx = wx * baseFreq_;
  const float ny = wy * baseFreq_;
  // Strong domain warp so ridgelines meander instead of marching in a regular
  // row (the warp is the main thing that makes the mountains look natural).
  const float wxw = nx + 0.9f * warp_.fbm(nx * 0.5f, ny * 0.5f, 3, 2.0f, 0.5f);
  const float wyw = ny + 0.9f * warp_.fbm(nx * 0.5f + 5.2f, ny * 0.5f + 1.3f, 3,
                                          2.0f, 0.5f);

  const float roll = hills_.fbm(wxw, wyw, p_.octaves, p_.lacunarity, p_.gain);
  const float rollUnit = roll * 0.5f + 0.5f;                  // [0,1] gentle
  // Broad mountain ridges from only a few ridged octaves (more octaves added
  // thin high-frequency spikes). The fbm `roll` above supplies the fine surface
  // texture on the slopes, so the result is wide mountains, not cones.
  const int ridgeOctaves = std::min(p_.octaves, 4);
  const float ridge = mtn_.ridged(wxw, wyw, ridgeOctaves, p_.lacunarity, 0.55f);
  const float mtnUnit = std::pow(clamp01(ridge), 0.7f);  // round the crests

  // Meadow regions: gentle undulation only. Mountain regions: rounded ridges
  // plus some hill mass so slopes aren't bare. mountainAmount blends between.
  const float elev = 0.10f * rollUnit +
                     mountainAmount * (p_.mountainMix * mtnUnit + 0.35f * rollUnit);
  return clamp01(elev) * p_.heightM;
}

PgVec3 TerrainField::normal(float wx, float wy, float eps) const {
  if (eps <= 0.0f) eps = 1.0f;
  const float hl = height(wx - eps, wy);
  const float hr = height(wx + eps, wy);
  const float hd = height(wx, wy - eps);
  const float hu = height(wx, wy + eps);
  // Surface z = -h(x,y); outward (up) normal ∝ (dh/dx, dh/dy, -1).
  const float dhdx = (hr - hl) / (2.0f * eps);
  const float dhdy = (hu - hd) / (2.0f * eps);
  float nx = dhdx, ny = dhdy, nz = -1.0f;
  const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (len > 0.0f) { nx /= len; ny /= len; nz /= len; }
  return PgVec3{nx, ny, nz};
}

PgVec3 TerrainField::color(float h, float flatness) const {
  const float t = clamp01(p_.heightM > 0.0f ? h / p_.heightM : 0.0f);
  const float steep = clamp01(1.0f - flatness);   // 0 flat .. 1 vertical
  // Muted, overcast palette to match the moody grass: darker desaturated greens,
  // earthy soil, cool grey rock, and cool (not blown-out) snow.
  // Grassy ground matches the CANOPY-FLOOR colour at the base of the GPU blades
  // (GpuGrass root ~vec3(0.05,0.14,0.09)) so the ground showing between blades
  // reads as the same shaded floor, not a lighter gap.
  const PgVec3 green{0.06f, 0.15f, 0.09f};
  const PgVec3 brown{0.29f, 0.23f, 0.15f};
  const PgVec3 rock {0.33f, 0.33f, 0.32f};
  const PgVec3 snow {0.80f, 0.83f, 0.88f};

  // Altitude band: green valley -> brown mid -> bare rock high. The green holds
  // up to near the grass line so thinning grass blends into green ground (not a
  // bare-soil edge), then browns over a narrow band above it.
  PgVec3 c = mix(green, brown, smoothstep(p_.colBrownT - 0.12f, p_.colBrownT + 0.10f, t));
  c = mix(c, rock, smoothstep(p_.colRockT - 0.34f, p_.colRockT, t));

  // Slope exposes brown/rock regardless of altitude (steeper = rockier).
  const float steepMix =
      smoothstep(p_.colSlopeT - 0.17f, p_.colSlopeT + 0.17f, steep);
  c = mix(c, mix(brown, rock, t), steepMix);

  // Snow caps the high tops, and not on near-vertical faces (won't hold).
  const float snowAmt =
      smoothstep(p_.colSnowT, p_.colSnowT + 0.2f, t) * (1.0f - 0.7f * steepMix);
  c = mix(c, snow, snowAmt);
  return c;
}

ProcMesh meshFieldChunk(const TerrainField& f, int cx, int cy, float chunkM,
                        int res) {
  ProcMesh m;
  if (res < 1 || chunkM <= 0.0f) return m;

  const float step = chunkM / static_cast<float>(res);
  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;
  const int verts = res + 1;

  // Sample heights on an extended grid (chunk + apron). The apron lets the slope
  // cap be applied identically at chunk borders, so adjacent chunks stay
  // seamless. apron must exceed the number of erosion iterations.
  const float maxSlope = f.params().maxSlope;
  const bool limit = maxSlope > 0.0f && maxSlope < 8.0f;
  const int apron = limit ? 8 : 0;
  const int iters = 6;
  const int gn = verts + 2 * apron;

  std::vector<float> H(static_cast<std::size_t>(gn) * gn);
  for (int j = 0; j < gn; ++j)
    for (int i = 0; i < gn; ++i)
      H[static_cast<std::size_t>(j) * gn + i] = f.height(
          x0 + step * static_cast<float>(i - apron),
          y0 + step * static_cast<float>(j - apron));

  // Slope cap: iteratively pull any cell down to at most maxStep above its
  // lowest 4-neighbour (grayscale erosion). Removes spikes/near-vertical faces
  // while leaving slopes below the cap untouched. Buffered for order-independence.
  if (limit) {
    const float maxStep = maxSlope * step;
    std::vector<float> B = H;
    for (int k = 0; k < iters; ++k) {
      for (int j = 1; j < gn - 1; ++j)
        for (int i = 1; i < gn - 1; ++i) {
          const std::size_t c = static_cast<std::size_t>(j) * gn + i;
          const float lo = std::min(std::min(H[c - 1], H[c + 1]),
                                    std::min(H[c - gn], H[c + gn]));
          B[c] = std::min(H[c], lo + maxStep);
        }
      H = B;
    }
  }

  // Limited-grid accessor for chunk node (i,j); the apron covers i,j in [-1..verts].
  auto gh = [&](int i, int j) -> float {
    int gi = i + apron, gj = j + apron;
    gi = gi < 0 ? 0 : (gi >= gn ? gn - 1 : gi);
    gj = gj < 0 ? 0 : (gj >= gn ? gn - 1 : gj);
    return H[static_cast<std::size_t>(gj) * gn + gi];
  };
  const float inv2s = 1.0f / (2.0f * step);

  // Node position / normal (from the limited grid) / color, then triangle soup.
  std::vector<PgVec3> pos(static_cast<std::size_t>(verts) * verts);
  std::vector<PgVec3> nrm(static_cast<std::size_t>(verts) * verts);
  std::vector<PgVec3> col(static_cast<std::size_t>(verts) * verts);
  for (int j = 0; j < verts; ++j) {
    for (int i = 0; i < verts; ++i) {
      const float h = gh(i, j);
      // Surface z = -h; outward normal ∝ (dh/dx, dh/dy, -1) from the limited grid.
      const float dhdx = (gh(i + 1, j) - gh(i - 1, j)) * inv2s;
      const float dhdy = (gh(i, j + 1) - gh(i, j - 1)) * inv2s;
      float nx = dhdx, ny = dhdy, nz = -1.0f;
      const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (len > 0.0f) { nx /= len; ny /= len; nz /= len; }
      const std::size_t k = static_cast<std::size_t>(j) * verts + i;
      pos[k] = PgVec3{x0 + step * static_cast<float>(i),
                      y0 + step * static_cast<float>(j), -h};
      nrm[k] = PgVec3{nx, ny, nz};
      const float flatness = nz < 0.0f ? -nz : 0.0f;
      col[k] = f.color(h, flatness);
    }
  }

  m.positions.reserve(static_cast<std::size_t>(res) * res * 6);
  m.normals.reserve(static_cast<std::size_t>(res) * res * 6);
  m.colors.reserve(static_cast<std::size_t>(res) * res * 6);
  auto push = [&](std::size_t k) {
    m.positions.push_back(pos[k]);
    m.normals.push_back(nrm[k]);
    m.colors.push_back(col[k]);
  };
  for (int j = 0; j < res; ++j) {
    for (int i = 0; i < res; ++i) {
      const std::size_t k00 = static_cast<std::size_t>(j) * verts + i;
      const std::size_t k10 = k00 + 1;
      const std::size_t k01 = k00 + verts;
      const std::size_t k11 = k01 + 1;
      push(k00); push(k10); push(k11);
      push(k00); push(k11); push(k01);
    }
  }
  return m;
}

}  // namespace vsim::procgen
