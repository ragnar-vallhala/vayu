#include "Flora.h"

#include <cmath>

namespace vsim::procgen {
namespace {

inline uint32_t h32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}
// Hash a global grid cell + a salt into a 32-bit value (deterministic).
inline uint32_t hcell(int gx, int gy, uint32_t seed, uint32_t salt) {
  uint32_t h = static_cast<uint32_t>(gx) * 0x9e3779b1U;
  h ^= static_cast<uint32_t>(gy) * 0x85ebca77U;
  h ^= seed * 0xc2b2ae3dU;
  h ^= salt * 0x27d4eb2fU;
  return h32(h);
}
inline float u01(uint32_t h) { return (h & 0xffffffU) / 16777216.0f; }

inline float clamp01(float v) {
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}
inline float smoothstep(float e0, float e1, float x) {
  const float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}

} // namespace

std::vector<FloraInstance> scatterFlora(const TerrainField &f, int cx, int cy,
                                        float chunkM, const FloraParams &p) {
  std::vector<FloraInstance> out;
  if (p.spacing <= 0.0f || chunkM <= 0.0f)
    return out;

  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;

  // Sample the field's height onto a fixed-resolution COARSE grid ONCE, then
  // bilinear-sample that for every blade. The expensive noise field is evaluated
  // ~CR^2 times no matter how dense the grass is (instead of once per blade) —
  // this is the CPU analogue of Ghost of Tsushima's GPU height texture, and is
  // what makes tiny spacing affordable to scatter.
  const int CR = 128;
  const int cg = CR + 1;
  const float cstep = chunkM / static_cast<float>(CR);
  std::vector<float> CH(static_cast<std::size_t>(cg) * cg);
  for (int j = 0; j < cg; ++j)
    for (int i = 0; i < cg; ++i)
      CH[static_cast<std::size_t>(j) * cg + i] =
          f.height(x0 + static_cast<float>(i) * cstep,
                   y0 + static_cast<float>(j) * cstep);
  auto hAt = [&](float wx, float wy) -> float {
    float gx = (wx - x0) / cstep, gy = (wy - y0) / cstep;
    gx = gx < 0.0f ? 0.0f : (gx > CR ? float(CR) : gx);
    gy = gy < 0.0f ? 0.0f : (gy > CR ? float(CR) : gy);
    const int i0 = int(gx), j0 = int(gy);
    const int i1 = i0 < CR ? i0 + 1 : i0, j1 = j0 < CR ? j0 + 1 : j0;
    const float fx = gx - i0, fy = gy - j0;
    const float a = CH[j0 * cg + i0] * (1 - fx) + CH[j0 * cg + i1] * fx;
    const float b = CH[j1 * cg + i0] * (1 - fx) + CH[j1 * cg + i1] * fx;
    return a * (1 - fy) + b * fy;
  };

  // Global blade grid (cell ownership keeps neighbours seamless).
  const int gx0 = static_cast<int>(std::floor(x0 / p.spacing));
  const int gx1 = static_cast<int>(std::floor((x0 + chunkM) / p.spacing));
  const int gy0 = static_cast<int>(std::floor(y0 / p.spacing));
  const int gy1 = static_cast<int>(std::floor((y0 + chunkM) / p.spacing));
  for (int gx = gx0; gx <= gx1; ++gx) {
    const float baseX = (static_cast<float>(gx) + 0.5f) * p.spacing;
    if (baseX < x0 || baseX >= x0 + chunkM)
      continue; // owned by another chunk
    for (int gy = gy0; gy <= gy1; ++gy) {
      const float baseY = (static_cast<float>(gy) + 0.5f) * p.spacing;
      if (baseY < y0 || baseY >= y0 + chunkM)
        continue;

      const float h = hAt(baseX, baseY);
      // Slope from the coarse grid (cheap bilinear taps). flatness = 1/|normal|.
      const float dhdx =
          (hAt(baseX + cstep, baseY) - hAt(baseX - cstep, baseY)) /
          (2.0f * cstep);
      const float dhdy =
          (hAt(baseX, baseY + cstep) - hAt(baseX, baseY - cstep)) /
          (2.0f * cstep);
      const float flatness = 1.0f / std::sqrt(dhdx * dhdx + dhdy * dhdy + 1.0f);

      // Density as a smooth function of height and slope: full on the flat,
      // low valley floor; falls to zero on steep faces and up toward the rocky
      // tops, so grass hugs the green ground (matching the surface colour).
      const float t = h / (f.params().heightM + 1e-3f);
      const float altF =
          1.0f - smoothstep(p.grassMaxFrac * 0.55f, p.grassMaxFrac, t);
      const float slopeF = smoothstep(p.slopeLo, p.slopeHi, flatness);
      const float density = altF * slopeF;
      if (density <= 0.0f)
        continue;
      if (u01(hcell(gx, gy, p.seed, 3)) > density)
        continue;

      // Regional lean angle: a slow spatial variation so nearby blades lean
      // together (the meadow "flows") instead of pointing every which way.
      const float regAngle =
          2.0f * (std::sin(baseX * 0.035f) + std::cos(baseY * 0.028f));

      // Multiple blades per cell (each with its own jitter/yaw/height) to crank
      // density cheaply. Per-blade salts are spaced 20 apart so they don't
      // collide across blades or with the cell's density salt (3).
      const int nb = std::max(1, static_cast<int>(std::lround(p.bladesPerCell)));
      for (int bi = 0; bi < nb; ++bi) {
        const uint32_t s0 = 10u + static_cast<uint32_t>(bi) * 20u;
        const float jx = (u01(hcell(gx, gy, p.seed, s0)) - 0.5f) * p.jitter;
        const float jy = (u01(hcell(gx, gy, p.seed, s0 + 1)) - 0.5f) * p.jitter;
        const float wx = baseX + jx * p.spacing;
        const float wy = baseY + jy * p.spacing;

        FloraInstance b;
        // Surface height at the jittered position from the coarse grid (cheap).
        b.pos = PgVec3{wx, wy, -hAt(wx, wy)};
        // Yaw = regional lean + a moderate per-blade spread (flow, not chaos).
        b.yaw = regAngle + (u01(hcell(gx, gy, p.seed, s0 + 2)) - 0.5f) * 2.0f;
        // Normal-distributed height (Box-Muller) from mean + std deviation.
        float u1 = u01(hcell(gx, gy, p.seed, s0 + 3));
        const float u2 = u01(hcell(gx, gy, p.seed, s0 + 4));
        if (u1 < 1e-6f)
          u1 = 1e-6f;
        const float gauss =
            std::sqrt(-2.0f * std::log(u1)) * std::cos(6.2831853f * u2);
        b.height = std::max(0.03f, p.heightMean + p.heightStdDev * gauss);

        const bool isFlower = u01(hcell(gx, gy, p.seed, s0 + 5)) < p.flowerFrac;
        if (isFlower) {
          const float fh = u01(hcell(gx, gy, p.seed, s0 + 6));
          if (fh < 0.40f)
            b.tint = PgVec3{0.95f, 0.95f, 0.97f}; // white
          else if (fh < 0.72f)
            b.tint = PgVec3{0.93f, 0.84f, 0.28f}; // yellow
          else
            b.tint = PgVec3{0.86f, 0.34f, 0.30f}; // red
          b.flower = 1.0f; // renderer keeps a green stem, blooms only the top
        } else {
          const float v = 0.85f + 0.34f * u01(hcell(gx, gy, p.seed, s0 + 7));
          const float yellow = 0.12f * u01(hcell(gx, gy, p.seed, s0 + 8));
          b.tint = PgVec3{(0.30f + yellow) * v, 0.52f * v, 0.18f * v};
          b.flower = 0.0f;
        }
        out.push_back(b);
      }
    }
  }
  return out;
}

} // namespace vsim::procgen
