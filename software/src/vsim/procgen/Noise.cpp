#include "Noise.h"

#include <cmath>

namespace vsim::procgen {
namespace {

// 32-bit integer avalanche hash (variant of the Wang/MurmurHash finalizer).
// Deterministic; good bit-mixing so adjacent lattice cells decorrelate.
inline uint32_t hash32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

// Mix lattice coords + seed into a single 32-bit hash.
inline uint32_t hash2(int ix, int iy, uint32_t seed) {
  uint32_t h = static_cast<uint32_t>(ix) * 0x9e3779b1U;
  h ^= static_cast<uint32_t>(iy) * 0x85ebca77U;
  h ^= seed * 0xc2b2ae3dU;
  return hash32(h);
}

// Quintic smootherstep (Perlin's improved interpolant): C2-continuous, so the
// terrain has no second-derivative creases at lattice lines.
inline float fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

inline float lerp(float a, float b, float t) { return a + t * (b - a); }

}  // namespace

void Noise::cornerGradient(int ix, int iy, float& gx, float& gy) const {
  // Map the lattice hash to an angle on the unit circle -> unit gradient. Using
  // a full continuum of directions (not 8 fixed ones) avoids axis-aligned
  // artefacts that show up as grid-shaped ridges on terrain.
  const uint32_t h = hash2(ix, iy, seed_);
  const float angle = (static_cast<float>(h) / 4294967296.0f) * 6.2831853f;
  gx = std::cos(angle);
  gy = std::sin(angle);
}

float Noise::gradient(float x, float y) const {
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float fx = x - static_cast<float>(x0);
  const float fy = y - static_cast<float>(y0);

  // Dot each corner's gradient with the offset vector from that corner.
  float g00x, g00y, g10x, g10y, g01x, g01y, g11x, g11y;
  cornerGradient(x0, y0, g00x, g00y);
  cornerGradient(x1, y0, g10x, g10y);
  cornerGradient(x0, y1, g01x, g01y);
  cornerGradient(x1, y1, g11x, g11y);

  const float n00 = g00x * fx + g00y * fy;
  const float n10 = g10x * (fx - 1.0f) + g10y * fy;
  const float n01 = g01x * fx + g01y * (fy - 1.0f);
  const float n11 = g11x * (fx - 1.0f) + g11y * (fy - 1.0f);

  const float u = fade(fx);
  const float v = fade(fy);
  const float nx0 = lerp(n00, n10, u);
  const float nx1 = lerp(n01, n11, u);
  // Perlin gradient noise lands in ~[-0.707, 0.707]; scale to ~[-1, 1].
  return lerp(nx0, nx1, v) * 1.4142136f;
}

float Noise::fbm(float x, float y, int octaves, float lacunarity,
                 float gain) const {
  float sum = 0.0f;
  float amp = 1.0f;
  float freq = 1.0f;
  float norm = 0.0f;
  if (octaves < 1) octaves = 1;
  for (int o = 0; o < octaves; ++o) {
    sum += amp * gradient(x * freq, y * freq);
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }
  return norm > 0.0f ? sum / norm : 0.0f;
}

float Noise::ridged(float x, float y, int octaves, float lacunarity,
                    float gain) const {
  float sum = 0.0f;
  float amp = 1.0f;
  float freq = 1.0f;
  float norm = 0.0f;
  if (octaves < 1) octaves = 1;
  for (int o = 0; o < octaves; ++o) {
    // 1 - |noise| peaks (=1) where the noise crosses zero -> sharp ridge lines;
    // squaring sharpens the ridge and rounds the valley floor.
    float r = 1.0f - std::fabs(gradient(x * freq, y * freq));
    r *= r;
    sum += amp * r;
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }
  return norm > 0.0f ? sum / norm : 0.0f;
}

}  // namespace vsim::procgen
