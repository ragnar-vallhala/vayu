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
        float target = d * params_.max_omega[i];

        // Asymmetric first-order tracking; pick tau by direction.
        // Note: the alpha formula is backward-Euler discretization, not
        // "exact" as the earlier comment claimed -- carrying behavior
        // forward verbatim, but the label is fixed.
        // Per-rotor spin-up constant; spin-down is 2x (legacy asymmetry).
        const float tau_up = params_.tau[i];
        float tau = (target > omega_[i]) ? tau_up : (tau_up * 2.0f);
        if (tau < 1e-6f) tau = 1e-6f;
        float alpha = dt / (tau + dt);
        omega_[i] += (target - omega_[i]) * alpha;

        float w2 = omega_[i] * omega_[i];
        float thrust = params_.k_thrust[i] * w2;

        // Thrust along the per-rotor axis (unit, body frame). Default
        // axis is body -Z (lift = up in NED). Normalize defensively in
        // case the editor pushed a non-unit vector.
        Vec3 axis = params_.axis_b[i];
        float an = std::sqrt(axis.x()*axis.x() + axis.y()*axis.y() + axis.z()*axis.z());
        if (an > 1e-6f) axis = axis / an;
        Vec3 F_i = axis * thrust;
        F += F_i;

        // Moment from rotor offset: r x F.
        Vec3 r = params_.pos_b[i];
        T += Vec3::crossProduct(r, F_i);

        // Reaction torque about the spin (= thrust) axis, signed by spin
        // direction. The -axis keeps parity with the previous body-Z
        // convention for the default -Z axis (reaction was +Z * spin).
        float tau_react = params_.k_moment[i] * w2 * static_cast<float>(params_.spin[i]);
        T -= axis * tau_react;
    }

    if (force_b)  *force_b  = F;
    if (torque_b) *torque_b = T;
}

}  // namespace vsim
