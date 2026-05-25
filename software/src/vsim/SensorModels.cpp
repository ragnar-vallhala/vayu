#include "SensorModels.h"

#include <algorithm>
#include <cmath>

namespace vsim {

SensorModels::SensorModels()
    : rng_(0xC0FFEE), norm_(0.0f, 1.0f) {}

void SensorModels::seed(uint64_t s) {
  rng_.seed(s);
}

float SensorModels::randn(float std) {
  return norm_(rng_) * std;
}

// Drives one component of a bias vector through a clipped random
// walk: bias += N(0, walk_std) every tick, then clamp |bias_i| <=
// clip. Returns the new bias (mostly for ergonomics; bias is also
// mutated in-place).
Vec3 SensorModels::walk(Vec3& bias, float walk_std, float clip) {
  bias += Vec3(randn(walk_std), randn(walk_std), randn(walk_std));
  bias.setX(std::clamp(bias.x(), -clip, clip));
  bias.setY(std::clamp(bias.y(), -clip, clip));
  bias.setZ(std::clamp(bias.z(), -clip, clip));
  return bias;
}

ImuSample SensorModels::sample(const RigidBodyState& state,
                               const Vec3& a_world,
                               float dt) {
  // Random-walk biases. dt unused for the walk step itself; the
  // walk is "per call" rather than "per second" since the firmware
  // samples at a fixed cadence and the walk stddev should be tuned
  // for that cadence.
  (void)dt;
  walk(acc_bias_, noise_.acc_bias_walk, noise_.acc_bias_clip);
  walk(gyr_bias_, noise_.gyr_bias_walk, noise_.gyr_bias_clip);
  walk(mag_bias_, noise_.mag_bias_walk, noise_.mag_bias_clip);

  // Specific force in WORLD frame = a_world - gravity_world.
  // Gravity in NED is (0,0,+G). The accelerometer feels everything
  // except gravity, i.e. the support/thrust forces required to
  // produce the world acceleration. At rest a_world = (0,0,0) so
  // specific force = -(0,0,G) = (0,0,-G); rotate into body and a
  // level airframe sees (0,0,-G). That's the standard BMX160 "1g
  // upward in body frame at rest" reading the firmware expects.
  Vec3 spec_w = a_world - Vec3(0.0f, 0.0f, kG);

  // Rotate world->body: q.conjugated() applied to a world vector
  // gives the same vector expressed in body frame.
  Quat q_inv = state.att.conjugated();
  Vec3 acc_b = q_inv.rotatedVector(spec_w);
  Vec3 mag_b = q_inv.rotatedVector(magWorldNed());

  ImuSample s;
  s.acc = acc_b + acc_bias_ + Vec3(randn(noise_.acc_noise_std),
                                   randn(noise_.acc_noise_std),
                                   randn(noise_.acc_noise_std));
  s.gyr = state.omega_b + gyr_bias_ + Vec3(randn(noise_.gyr_noise_std),
                                           randn(noise_.gyr_noise_std),
                                           randn(noise_.gyr_noise_std));
  s.mag = mag_b + mag_bias_ + Vec3(randn(noise_.mag_noise_std),
                                   randn(noise_.mag_noise_std),
                                   randn(noise_.mag_noise_std));
  s.temp = noise_.temp_mean + randn(noise_.temp_noise_std);
  return s;
}

}  // namespace vsim
