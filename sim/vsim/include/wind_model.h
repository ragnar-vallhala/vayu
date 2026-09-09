// wind_model.h — world-frame wind field: steady + deterministic gust +
// Dryden-style band-limited turbulence. The airframe feels it as relative-
// velocity drag in PhysicsCore::derive(). See
// docs/sim-fidelity/01-wind-turbulence.md.
//
// Header-only: it's a small filter and keeps the daemon's CMake source list
// untouched. Owns its OWN seeded RNG (independent of the sensor-noise RNG) so
// enabling wind never perturbs the existing accelerometer/gyro/mag noise
// sequence — both streams are reproducible from the same reset master seed.
#ifndef VSIM_WIND_MODEL_H
#define VSIM_WIND_MODEL_H

#include "vsim_types.h"

#include <cmath>
#include <cstdint>
#include <random>

namespace vsim {

struct WindConfig {
  Vec3 steady{0.0f, 0.0f, 0.0f}; // NED [m/s]
  float gust_amp = 0.0f;         // peak gust [m/s]
  float gust_period = 0.0f;      // [s], <=0 disables the gust
  float turb_sigma = 0.0f;       // turbulence RMS [m/s]
  float turb_tau = 1.0f;         // correlation time [s]
  bool enable = false;           // false = fast bypass (no wind, no draw)
};

class WindModel {
public:
  void setConfig(const WindConfig &c) {
    cfg_ = c;
    if (cfg_.turb_tau <= 0.0f)
      cfg_.turb_tau = 1.0f; // guard div-by-zero
  }

  // Re-seed the turbulence RNG and zero the filter state so an identical
  // reset reproduces an identical wind trajectory. Salted off the master seed
  // so wind and sensor noise are decorrelated yet both deterministic.
  void seed(uint64_t s) {
    rng_.seed(s ^ 0x9E3779B97F4A7C15ULL); // golden-ratio salt
    turb_ = Vec3(0.0f, 0.0f, 0.0f);
  }

  // Clear the running state (gust phase + turbulence) without touching config.
  void reset() {
    t_ = 0.0f;
    turb_ = Vec3(0.0f, 0.0f, 0.0f);
    last_ = Vec3(0.0f, 0.0f, 0.0f);
  }

  // Advance by dt and return the instantaneous world wind [m/s] NED.
  const Vec3 &step(float dt) {
    if (!cfg_.enable) {
      last_ = Vec3(0.0f, 0.0f, 0.0f);
      return last_;
    }
    t_ += dt;

    // Gust: a sinusoid along the horizontal steady direction (N if steady
    // has no horizontal component). Deterministic, so A/B tuning runs match.
    Vec3 gust(0.0f, 0.0f, 0.0f);
    if (cfg_.gust_amp != 0.0f && cfg_.gust_period > 0.0f) {
      const float hn = std::sqrt(cfg_.steady.x() * cfg_.steady.x() +
                                 cfg_.steady.y() * cfg_.steady.y());
      const Vec3 dir =
          (hn > 1e-6f) ? Vec3(cfg_.steady.x() / hn, cfg_.steady.y() / hn, 0.0f)
                       : Vec3(1.0f, 0.0f, 0.0f);
      const float amp =
          cfg_.gust_amp * std::sin(2.0f * kPi * t_ / cfg_.gust_period);
      gust = dir * amp;
    }

    // Turbulence: discrete first-order (Dryden-ish) shaping filter per axis.
    // The sqrt(1-a^2) factor makes the steady-state RMS == turb_sigma for
    // any dt, so the intensity knob is dt-independent.
    if (cfg_.turb_sigma > 0.0f) {
      const float a = std::exp(-dt / cfg_.turb_tau);
      const float k = cfg_.turb_sigma * std::sqrt(1.0f - a * a);
      turb_ =
          Vec3(a * turb_.x() + k * norm_(rng_), a * turb_.y() + k * norm_(rng_),
               a * turb_.z() + k * norm_(rng_));
    } else {
      turb_ = Vec3(0.0f, 0.0f, 0.0f);
    }

    last_ = cfg_.steady + gust + turb_;
    return last_;
  }

  const Vec3 &world() const { return last_; }
  bool enabled() const { return cfg_.enable; }

private:
  static constexpr float kPi = 3.14159265358979323846f;

  WindConfig cfg_;
  std::mt19937_64 rng_{0xC0FFEEu};
  std::normal_distribution<float> norm_{0.0f, 1.0f};
  Vec3 turb_{0.0f, 0.0f, 0.0f};
  Vec3 last_{0.0f, 0.0f, 0.0f};
  float t_ = 0.0f;
};

} // namespace vsim

#endif // VSIM_WIND_MODEL_H
