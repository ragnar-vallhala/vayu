/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sensor_models.h"

#include <algorithm>

namespace vsim {

// A fixed seed is the point: runs must be reproducible. seed() overrides it.
// NOLINTNEXTLINE(cert-msc32-c,cert-msc51-cpp)
SensorModels::SensorModels() : rng_(0xC0FFEE), norm_(0.0f, 1.0f) {}

void SensorModels::seed(uint64_t s) {
  rng_.seed(s);
  // normal_distribution caches the second Box-Muller value between calls;
  // clear it so the stream is fully determined by the rng seed alone.
  norm_.reset();
  // Reset the random-walk bias accumulators too -- otherwise two resets
  // with the same seed still diverge, starting from whatever bias the
  // previous rollout drifted to. A reseed is a full sensor-state reset.
  acc_bias_ = Vec3(0.0f, 0.0f, 0.0f);
  gyr_bias_ = Vec3(0.0f, 0.0f, 0.0f);
  mag_bias_ = Vec3(0.0f, 0.0f, 0.0f);
}

float SensorModels::randn(float std) { return norm_(rng_) * std; }

Vec3 SensorModels::walk(Vec3 &bias, float walk_std, float clip) {
  bias += Vec3(randn(walk_std), randn(walk_std), randn(walk_std));
  bias.setX(std::clamp(bias.x(), -clip, clip));
  bias.setY(std::clamp(bias.y(), -clip, clip));
  bias.setZ(std::clamp(bias.z(), -clip, clip));
  return bias;
}

ImuSample SensorModels::sample(const RigidBodyState &state, const Vec3 &a_world,
                               float gravity, float dt) {
  (void)dt; // walk is per-call, not per-second
  walk(acc_bias_, noise_.acc_bias_walk, noise_.acc_bias_clip);
  walk(gyr_bias_, noise_.gyr_bias_walk, noise_.gyr_bias_clip);
  walk(mag_bias_, noise_.mag_bias_walk, noise_.mag_bias_clip);

  // Specific force in WORLD frame = a_world - gravity_world.
  // At rest a_world = (0,0,0) so spec = -(0,0,G) = (0,0,-G); a level
  // airframe sees (0,0,-G) in body frame -- the standard BMX160
  // reading firmware expects.
  Vec3 spec_w = a_world - Vec3(0.0f, 0.0f, gravity);

  Quat q_inv = state.att.conjugated();
  Vec3 acc_b = q_inv.rotatedVector(spec_w);
  Vec3 mag_b = q_inv.rotatedVector(magWorldNed());

  // Thrust-scaled vibration: prop/motor imbalance the telemetry IMU aliases.
  // vibe_*_std_ is set per-step from the motor command (0 = off, default).
  const float av = noise_.acc_noise_std + vibe_acc_std_;
  const float gv = noise_.gyr_noise_std + vibe_gyr_std_;
  ImuSample s;
  s.acc = acc_b + acc_bias_ + Vec3(randn(av), randn(av), randn(av));
  s.gyr = state.omega_b + gyr_bias_ + Vec3(randn(gv), randn(gv), randn(gv));
  s.mag = mag_b + mag_bias_ +
          Vec3(randn(noise_.mag_noise_std), randn(noise_.mag_noise_std),
               randn(noise_.mag_noise_std));
  s.temp = noise_.temp_mean + randn(noise_.temp_noise_std);
  return s;
}

} // namespace vsim
