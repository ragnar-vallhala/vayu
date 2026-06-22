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

}  // namespace

std::vector<FloraInstance> scatterFlora(const TerrainField& f, int cx, int cy,
                                        float chunkM, const FloraParams& p) {
  std::vector<FloraInstance> out;
  if (p.spacing <= 0.0f || chunkM <= 0.0f) return out;

  const float x0 = static_cast<float>(cx) * chunkM;
  const float y0 = static_cast<float>(cy) * chunkM;
  const float grassMaxH = p.grassMaxFrac * f.params().heightM;

  // Iterate the global grid cells whose centre falls in this chunk, so the
  // scatter is identical regardless of which chunk emits a given cell.
  const int gx0 = static_cast<int>(std::floor(x0 / p.spacing));
  const int gx1 = static_cast<int>(std::floor((x0 + chunkM) / p.spacing));
  const int gy0 = static_cast<int>(std::floor(y0 / p.spacing));
  const int gy1 = static_cast<int>(std::floor((y0 + chunkM) / p.spacing));

  out.reserve(static_cast<std::size_t>(gx1 - gx0 + 1) * (gy1 - gy0 + 1));
  for (int gx = gx0; gx <= gx1; ++gx) {
    const float baseX = (static_cast<float>(gx) + 0.5f) * p.spacing;
    if (baseX < x0 || baseX >= x0 + chunkM) continue;  // owned by another chunk
    for (int gy = gy0; gy <= gy1; ++gy) {
      const float baseY = (static_cast<float>(gy) + 0.5f) * p.spacing;
      if (baseY < y0 || baseY >= y0 + chunkM) continue;

      const float jx = (u01(hcell(gx, gy, p.seed, 1)) - 0.5f) * p.jitter;
      const float jy = (u01(hcell(gx, gy, p.seed, 2)) - 0.5f) * p.jitter;
      const float wx = baseX + jx * p.spacing;
      const float wy = baseY + jy * p.spacing;

      const float h = f.height(wx, wy);
      if (h > grassMaxH) continue;                 // bare rock up high
      const PgVec3 n = f.normal(wx, wy, p.spacing);
      const float flatness = n.z < 0.0f ? -n.z : 0.0f;
      if ((1.0f - flatness) > p.maxSlope) continue;  // too steep

      // Thin out toward the slope/altitude limits so edges fade, not cut hard.
      const float r = u01(hcell(gx, gy, p.seed, 3));
      const float density = clamp01(flatness * 1.2f) *
                            clamp01(1.0f - h / (grassMaxH + 1e-3f) * 0.6f);
      if (r > density) continue;

      FloraInstance b;
      b.pos = PgVec3{wx, wy, -h};
      b.yaw = u01(hcell(gx, gy, p.seed, 4)) * 6.2831853f;
      b.height = p.minHeight +
                 (p.maxHeight - p.minHeight) * u01(hcell(gx, gy, p.seed, 5));

      const bool isFlower = u01(hcell(gx, gy, p.seed, 6)) < p.flowerFrac;
      if (isFlower) {
        // A few bright accents: yellow / red / white from the hash.
        const float fh = u01(hcell(gx, gy, p.seed, 7));
        if (fh < 0.55f)      b.tint = PgVec3{0.92f, 0.82f, 0.20f};  // yellow
        else if (fh < 0.85f) b.tint = PgVec3{0.86f, 0.30f, 0.26f};  // red
        else                 b.tint = PgVec3{0.92f, 0.92f, 0.95f};  // white
        b.flower = 1.0f;
        b.height *= 1.15f;
      } else {
        // Grass green with per-blade lightness variation.
        const float v = 0.82f + 0.36f * u01(hcell(gx, gy, p.seed, 8));
        b.tint = PgVec3{0.26f * v, 0.42f * v, 0.17f * v};
        b.flower = 0.0f;
      }
      out.push_back(b);
    }
  }
  return out;
}

}  // namespace vsim::procgen
