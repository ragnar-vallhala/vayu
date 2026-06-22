#include "Flora.h"

#include <cmath>

namespace vsim::procgen {
namespace {

inline uint32_t h32(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352dU;
  x ^= x >> 15; x *= 0x846ca68bU;
  x ^= x >> 16; return x;
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

inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
inline float smoothstep(float e0, float e1, float x) {
  const float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}

}  // namespace

std::vector<FloraInstance> scatterFlora(const TerrainField& f, int cx, int cy,
                                        float chunkM, const FloraParams& p) {
  std::vector<FloraInstance> out;
  if (p.spacing <= 0.0f || chunkM <= 0.0f) return out;

  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;

  // Iterate the global grid cells whose centre falls in this chunk, so the
  // scatter is identical regardless of which chunk emits a given cell.
  const int gx0 = static_cast<int>(std::floor(x0 / p.spacing));
  const int gx1 = static_cast<int>(std::floor((x0 + chunkM) / p.spacing));
  const int gy0 = static_cast<int>(std::floor(y0 / p.spacing));
  const int gy1 = static_cast<int>(std::floor((y0 + chunkM) / p.spacing));
  const int nx = gx1 - gx0 + 1, ny = gy1 - gy0 + 1;
  if (nx < 1 || ny < 1) return out;

  // Sample terrain height ONCE per cell centre (the dominant cost), then derive
  // slope from grid neighbours instead of a full field normal per blade — ~5x
  // cheaper than sampling height+normal at every blade.
  std::vector<float> H(static_cast<std::size_t>(nx) * ny);
  for (int gi = 0; gi < nx; ++gi)
    for (int gj = 0; gj < ny; ++gj)
      H[static_cast<std::size_t>(gj) * nx + gi] =
          f.height((static_cast<float>(gx0 + gi) + 0.5f) * p.spacing,
                   (static_cast<float>(gy0 + gj) + 0.5f) * p.spacing);
  auto at = [&](int gi, int gj) {
    gi = gi < 0 ? 0 : (gi >= nx ? nx - 1 : gi);
    gj = gj < 0 ? 0 : (gj >= ny ? ny - 1 : gj);
    return H[static_cast<std::size_t>(gj) * nx + gi];
  };

  out.reserve(static_cast<std::size_t>(nx) * ny / 2);
  for (int gi = 0; gi < nx; ++gi) {
    const int gx = gx0 + gi;
    const float baseX = (static_cast<float>(gx) + 0.5f) * p.spacing;
    if (baseX < x0 || baseX >= x0 + chunkM) continue;  // owned by another chunk
    for (int gj = 0; gj < ny; ++gj) {
      const int gy = gy0 + gj;
      const float baseY = (static_cast<float>(gy) + 0.5f) * p.spacing;
      if (baseY < y0 || baseY >= y0 + chunkM) continue;

      const float h = at(gi, gj);
      // Slope from grid neighbours: surface z = -h, normal ∝ (dh/dx, dh/dy, -1),
      // so flatness = 1/|normal|.
      const float dhdx = (at(gi + 1, gj) - at(gi - 1, gj)) / (2.0f * p.spacing);
      const float dhdy = (at(gi, gj + 1) - at(gi, gj - 1)) / (2.0f * p.spacing);
      const float flatness = 1.0f / std::sqrt(dhdx * dhdx + dhdy * dhdy + 1.0f);

      // Density as a smooth function of height and slope: full on the flat,
      // low valley floor; falls to zero on steep faces and up toward the rocky
      // tops, so grass hugs the green ground (matching the surface colour).
      const float t = h / (f.params().heightM + 1e-3f);
      const float altF = 1.0f - smoothstep(p.grassMaxFrac * 0.55f,
                                           p.grassMaxFrac, t);
      const float slopeF = smoothstep(p.slopeLo, p.slopeHi, flatness);
      const float density = altF * slopeF;
      if (density <= 0.0f) continue;
      if (u01(hcell(gx, gy, p.seed, 3)) > density) continue;

      // Multiple blades per cell (each with its own jitter/yaw/height) to crank
      // density cheaply. Per-blade salts are spaced 20 apart so they don't
      // collide across blades or with the cell's density salt (3).
      const int nb = std::max(1, static_cast<int>(p.bladesPerCell + 0.5f));
      for (int bi = 0; bi < nb; ++bi) {
        const uint32_t s0 = 10u + static_cast<uint32_t>(bi) * 20u;
        const float jx = (u01(hcell(gx, gy, p.seed, s0)) - 0.5f) * p.jitter;
        const float jy = (u01(hcell(gx, gy, p.seed, s0 + 1)) - 0.5f) * p.jitter;
        const float wx = baseX + jx * p.spacing;
        const float wy = baseY + jy * p.spacing;

        FloraInstance b;
        // Exact surface height at the jittered position (only for kept blades)
        // so blades sit ON the ground, not floating on a slope.
        b.pos = PgVec3{wx, wy, -f.height(wx, wy)};
        b.yaw = u01(hcell(gx, gy, p.seed, s0 + 2)) * 6.2831853f;
        // Normal-distributed height (Box-Muller) from mean + std deviation.
        float u1 = u01(hcell(gx, gy, p.seed, s0 + 3));
        const float u2 = u01(hcell(gx, gy, p.seed, s0 + 4));
        if (u1 < 1e-6f) u1 = 1e-6f;
        const float gauss =
            std::sqrt(-2.0f * std::log(u1)) * std::cos(6.2831853f * u2);
        b.height = std::max(0.03f, p.heightMean + p.heightStdDev * gauss);

        const bool isFlower = u01(hcell(gx, gy, p.seed, s0 + 5)) < p.flowerFrac;
        if (isFlower) {
          const float fh = u01(hcell(gx, gy, p.seed, s0 + 6));
          if (fh < 0.55f)      b.tint = PgVec3{0.92f, 0.82f, 0.20f};  // yellow
          else if (fh < 0.85f) b.tint = PgVec3{0.86f, 0.30f, 0.26f};  // red
          else                 b.tint = PgVec3{0.92f, 0.92f, 0.95f};  // white
          b.flower = 1.0f;
          b.height *= 1.15f;
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

}  // namespace vsim::procgen
