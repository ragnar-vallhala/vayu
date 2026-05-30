#include "motor_model.h"

#include <algorithm>

namespace vsim {

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

        // Asymmetric first-order tracking; pick tau by direction.
        // Note: the alpha formula is backward-Euler discretization, not
        // "exact" as the earlier comment claimed -- carrying behavior
        // forward verbatim, but the label is fixed.
        float tau = (target > omega_[i]) ? params_.tau_up : params_.tau_down;
        if (tau < 1e-6f) tau = 1e-6f;
        float alpha = dt / (tau + dt);
        omega_[i] += (target - omega_[i]) * alpha;

        float w2 = omega_[i] * omega_[i];
        float thrust = params_.k_thrust * w2;

        // Body +Z is DOWN in NED; thrust points body -Z.
        Vec3 F_i(0.0f, 0.0f, -thrust);
        F += F_i;

        // Moment from rotor offset: r x F.
        Vec3 r = params_.pos_b[i];
        T += Vec3::crossProduct(r, F_i);

        // Reaction torque about body Z, signed by spin direction.
        float tau_react = params_.k_moment * w2 * static_cast<float>(params_.spin[i]);
        T += Vec3(0.0f, 0.0f, tau_react);
    }

    if (force_b)  *force_b  = F;
    if (torque_b) *torque_b = T;
}

}  // namespace vsim
