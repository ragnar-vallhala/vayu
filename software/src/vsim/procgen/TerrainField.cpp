#include "TerrainField.h"

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
  const float wxw = nx + 0.6f * warp_.fbm(nx * 0.5f, ny * 0.5f, 3, 2.0f, 0.5f);
  const float wyw = ny + 0.6f * warp_.fbm(nx * 0.5f + 5.2f, ny * 0.5f + 1.3f, 3,
                                          2.0f, 0.5f);

  const float roll = hills_.fbm(wxw, wyw, p_.octaves, p_.lacunarity, p_.gain);
  const float rollUnit = roll * 0.5f + 0.5f;                  // [0,1] gentle
  const float ridge = mtn_.ridged(wxw, wyw, p_.octaves, p_.lacunarity, p_.gain);
  const float mtnUnit = ridge * ridge;                        // [0,1] sharp

  // Meadow regions: gentle undulation only. Mountain regions: ridged peaks plus
  // some hill mass so slopes aren't bare. mountainAmount blends between them.
  const float elev = 0.10f * rollUnit +
                     mountainAmount * (p_.mountainMix * mtnUnit + 0.3f * rollUnit);
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
  const PgVec3 grass{0.27f, 0.44f, 0.16f};
  const PgVec3 olive{0.46f, 0.49f, 0.24f};
  const PgVec3 rock {0.42f, 0.38f, 0.33f};
  PgVec3 c = t < 0.5f ? mix(grass, olive, t * 2.0f)
                      : mix(olive, rock, (t - 0.5f) * 2.0f);
  const float rockiness = clamp01((0.78f - flatness) * 3.0f);
  return mix(c, PgVec3{0.36f, 0.31f, 0.26f}, rockiness);
}

ProcMesh meshFieldChunk(const TerrainField& f, int cx, int cy, float chunkM,
                        int res) {
  ProcMesh m;
  if (res < 1 || chunkM <= 0.0f) return m;

  const float step = chunkM / static_cast<float>(res);
  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;
  const int verts = res + 1;

  // Precompute node position / normal / color, then emit a triangle soup.
  std::vector<PgVec3> pos(static_cast<std::size_t>(verts) * verts);
  std::vector<PgVec3> nrm(static_cast<std::size_t>(verts) * verts);
  std::vector<PgVec3> col(static_cast<std::size_t>(verts) * verts);
  for (int j = 0; j < verts; ++j) {
    for (int i = 0; i < verts; ++i) {
      const float wx = x0 + step * static_cast<float>(i);
      const float wy = y0 + step * static_cast<float>(j);
      const float h = f.height(wx, wy);
      const PgVec3 nv = f.normal(wx, wy, step);
      const std::size_t k = static_cast<std::size_t>(j) * verts + i;
      pos[k] = PgVec3{wx, wy, -h};  // up is -Z in NED
      nrm[k] = nv;
      const float flatness = nv.z < 0.0f ? -nv.z : 0.0f;
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
