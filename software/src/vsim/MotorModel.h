#pragma once

#include "VsimTypes.h"

namespace vsim {

// Per-rotor first-order dynamics + thrust/torque aggregation.
//
// Each motor commanded duty (0..1) maps linearly to a target omega
// (max_omega * duty). Actual omega filters toward target with
// asymmetric tau_up / tau_down (props spin up slower than they coast
// down). Per-rotor thrust = k_thrust * omega^2 along body +Z (NED:
// body +Z is _down_ in world when level, but thrust on a quad points
// _away from the ground_ relative to the body, i.e. body -Z in NED).
// Reaction torque is k_moment * omega^2 about body Z, with sign
// dictated by spin direction (and again NED-signed so that +Z is
// down).
class MotorModel {
 public:
  MotorModel();

  void setParams(const MotorParams& p) { params_ = p; }
  const MotorParams& params() const { return params_; }

  // duty[4] in [0,1]; dt in seconds. Updates internal omegas and
  // returns the aggregated body-frame wrench.
  void update(const std::array<float, 4>& duty, float dt,
              Vec3* force_b, Vec3* torque_b);

  void reset();

  // Latest per-rotor angular speeds (rad/s), for rendering / telemetry.
  const std::array<float, 4>& omegas() const { return omega_; }

 private:
  MotorParams params_;
  std::array<float, 4> omega_ = {0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace vsim
