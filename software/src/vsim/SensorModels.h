#pragma once

#include "VsimTypes.h"

#include <random>

namespace vsim {

// Sensor noise + bias parameters. Defaults are loosely BMX160-ish:
//   - Accel noise density: ~150 ug/sqrt(Hz) -> at 200 Hz this is
//     about 21 mg = 0.21 m/s^2 RMS. We use 0.1 m/s^2 stddev as a
//     reasonable lump-sum value for the synthetic samples.
//   - Gyro noise: ~0.014 deg/s/sqrt(Hz) -> at 200 Hz about
//     0.2 deg/s RMS; we use 0.5 deg/s for headroom.
//   - Mag noise: typically dominated by environmental disturbance.
//     0.3 uT is reasonable.
//
// Biases drift via a slow random walk so the mahony estimator has
// something realistic to chase: each tick the bias gets a small
// Gaussian increment with stddev `bias_walk_*`.
struct SensorNoise {
  float acc_noise_std    = 0.10f;   // m/s^2
  float acc_bias_walk    = 0.0005f; // m/s^2 per tick
  float acc_bias_clip    = 0.20f;   // m/s^2 max bias magnitude

  float gyr_noise_std    = 0.0087f; // rad/s  (= 0.5 deg/s)
  float gyr_bias_walk    = 1e-5f;   // rad/s per tick
  float gyr_bias_clip    = 0.035f;  // rad/s  (= 2 deg/s)

  float mag_noise_std    = 0.3f;    // uT
  float mag_bias_walk    = 0.001f;  // uT per tick
  float mag_bias_clip    = 5.0f;    // uT

  float temp_mean        = 25.0f;   // degC
  float temp_noise_std   = 0.05f;
};

// Stateless-from-outside sensor synthesizer. Given physics state +
// motor wrench info, produces an ImuSample matching the BMX160's
// data layout that the firmware would otherwise read off the I2C bus.
class SensorModels {
 public:
  SensorModels();

  void setNoise(const SensorNoise& n) { noise_ = n; }
  const SensorNoise& noise() const { return noise_; }

  // Seed the RNGs deterministically (helpful for reproducible runs).
  void seed(uint64_t s);

  // Build one IMU sample from current rigid-body state + the world
  // acceleration we just integrated (a_world includes gravity and
  // body-frame forces). The accelerometer measures SPECIFIC FORCE
  // (= a_world - gravity_world) rotated into body frame -- so when
  // the airframe is at rest with no thrust, acc_body = (0,0,-G).
  ImuSample sample(const RigidBodyState& state,
                   const Vec3& a_world,
                   float dt);

 private:
  Vec3 walk(Vec3& bias, float walk_std, float clip);
  float randn(float std);

  SensorNoise noise_;
  std::mt19937_64                    rng_;
  std::normal_distribution<float>    norm_;

  Vec3 acc_bias_{0.0f, 0.0f, 0.0f};
  Vec3 gyr_bias_{0.0f, 0.0f, 0.0f};
  Vec3 mag_bias_{0.0f, 0.0f, 0.0f};
};

}  // namespace vsim
