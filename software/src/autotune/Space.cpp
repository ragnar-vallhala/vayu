#include "Space.h"

namespace autotune {

// (name, lo, hi, seed) — mirrors autotune.py _BASE / _YAW exactly.
static const Param kBase[] = {
    {"rate_kp", 0.0005, 0.012, 0.007},  // capped below the buzz knee (~0.01)
    {"rate_ki", 0.0, 0.01, 0.002},
    {"rate_kd", 0.0, 0.001, 0.0005},  // D on the noisy gyro buzzes -> capped
    {"angle_kp", 0.05, 4.0, 2.0},     // responsiveness lever (safe, wide)
    {"gyro_lpf", 0.0, 0.012, 0.0},    // gyro LPF [s]; lag here destabilises
};
static const Param kYaw[] = {
    {"yaw_rate_kp", 0.002, 0.05, 0.01},
    {"yaw_rate_ki", 0.0, 0.012, 0.002},
    {"yaw_rate_kd", 0.0, 0.002, 0.0005},
    {"yaw_gyro_lpf", 0.0, 0.012, 0.0},
};

Space::Space(bool tuneYaw) : m_tuneYaw(tuneYaw) {
  for (const Param &p : kBase)
    m_params.push_back(p);
  if (tuneYaw)
    for (const Param &p : kYaw)
      m_params.push_back(p);
}

std::vector<std::string> Space::names() const {
  std::vector<std::string> n;
  n.reserve(m_params.size());
  for (const Param &p : m_params)
    n.push_back(p.name);
  return n;
}

Bounds Space::bounds() const {
  Bounds b;
  b.reserve(m_params.size());
  for (const Param &p : m_params)
    b.emplace_back(p.lo, p.hi);
  return b;
}

Vec Space::seed() const {
  Vec s;
  s.reserve(m_params.size());
  for (const Param &p : m_params)
    s.push_back(p.seed);
  return s;
}

}  // namespace autotune
