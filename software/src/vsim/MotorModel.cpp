#include "MotorModel.h"

#include <algorithm>
#include <cmath>

namespace vsim {

MotorModel::MotorModel() = default;

void MotorModel::reset() {
  omega_ = {0.0f, 0.0f, 0.0f, 0.0f};
}

void MotorModel::update(const std::array<float, 4>& duty, float dt,
                        Vec3* force_b, Vec3* torque_b) {
  Vec3 F(0.0f, 0.0f, 0.0f);
  Vec3 T(0.0f, 0.0f, 0.0f);

  for (int i = 0; i < 4; ++i) {
    float d = std::clamp(duty[i], 0.0f, 1.0f);
    float target = d * params_.max_omega;

    // Asymmetric first-order tracking. Pick the time constant by
    // direction: spinning up uses tau_up, slowing uses tau_down.
    float tau = (target > omega_[i]) ? params_.tau_up : params_.tau_down;
    if (tau < 1e-6f) tau = 1e-6f;
    float alpha = dt / (tau + dt);            // exact discrete pole
    omega_[i] += (target - omega_[i]) * alpha;

    float w2 = omega_[i] * omega_[i];
    float thrust = params_.k_thrust * w2;     // N, magnitude

    // Body +Z is DOWN in NED. Quad thrust points up out of the rotors,
    // i.e. body -Z. Hence F_i_z = -thrust.
    Vec3 F_i(0.0f, 0.0f, -thrust);
    F += F_i;

    // Moment from offset rotor: r x F.
    Vec3 r = params_.pos_b[i];                // body XY, z=0
    T += Vec3::crossProduct(r, F_i);

    // Reaction torque about body Z. CCW rotor (spin=+1) viewed from
    // ABOVE (i.e. looking down body +Z, which is into the ground)
    // produces a reaction torque on the body in the opposite direction
    // of the rotor's angular velocity vector. NED body +Z = down, so
    // a rotor that looks CCW from above (the conventional view from
    // outside the airframe) is actually CW about the NED body +Z
    // axis. Net effect: spin=+1 -> reaction torque about body +Z is
    // positive. Pre-existing vayu motor-mixing assumes M1+M3 CCW;
    // their yaw command is achieved by speeding those two motors,
    // which produces the desired +yaw torque -- so the sign here is
    // already consistent.
    float tau_react = params_.k_moment * w2 * static_cast<float>(params_.spin[i]);
    T += Vec3(0.0f, 0.0f, tau_react);
  }

  if (force_b)  *force_b  = F;
  if (torque_b) *torque_b = T;
}

}  // namespace vsim
