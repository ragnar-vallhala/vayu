// Noise.h — deterministic, dependency-free 2D coherent noise.
//
// Everything seeds from one 32-bit integer, so a given (seed, params) pair
// reproduces the same field byte-for-byte — the same world must come out on the
// render side and the physics/BVH side. No global RNG, no wall-clock.
#pragma once

#include <cstdint>

namespace vsim::procgen {

// Gradient (Perlin-style) value noise. Cheap, branch-light, good enough for
// terrain. fbm() and ridged() are the two compositions terrain actually uses;
// gradient() is exposed for tests and for callers that want a single octave.
class Noise {
 public:
  explicit Noise(uint32_t seed) : seed_(seed) {}

  // Single-octave gradient noise at continuous (x, y). Range ~[-1, 1], smooth
  // (C2) and zero at integer lattice points.
  float gradient(float x, float y) const;

  // Fractal Brownian motion: `octaves` summed gradient layers, each octave at
  // `lacunarity`x the previous frequency and `gain`x the amplitude. Normalised
  // back to ~[-1, 1] regardless of octave count.
  float fbm(float x, float y, int octaves, float lacunarity, float gain) const;

  // Ridged multifractal: per-octave (1 - |gradient|)^2, weighted by the running
  // amplitude. Produces sharp ridges and rounded valleys — the mountain look.
  // Range ~[0, 1].
  float ridged(float x, float y, int octaves, float lacunarity, float gain) const;

 private:
  // 2D hash -> pseudo-random unit gradient at lattice corner (ix, iy).
  void cornerGradient(int ix, int iy, float& gx, float& gy) const;

  uint32_t seed_;
};

}  // namespace vsim::procgen
