// physics_core.h — single-rigid-body RK4 integrator. Ported from
// software/src/vsim/PhysicsCore.h with Qt math types swapped for the
// Qt-free Vec3 / Quat in vsim_math.h. Behavior is bit-equivalent.
#ifndef VSIM_PHYSICS_CORE_H
#define VSIM_PHYSICS_CORE_H

#include "vsim_types.h"

namespace vsim {

// RK4-integrated rigid body, with a hard ground clamp at
// state.pos_w.z == params.ground_z (NED z grows downward).
//
// The wrench (force_b, torque_b) handed to step() is in body frame.
// Internally the integrator rotates force into world frame for the
// linear ODE and keeps angular in body frame.
class PhysicsCore {
public:
    PhysicsCore() = default;

    void reset(const RigidBodyState& initial = {});
    void setParams(const DroneParams& p) { params_ = p; }

    // One RK4 step. force_b and torque_b are body-frame.
    void step(const Vec3& force_b, const Vec3& torque_b, float dt);

    const RigidBodyState& state()  const { return state_; }
    const DroneParams&    params() const { return params_; }

private:
    struct Deriv {
        Vec3 d_pos;    // = vel_w
        Vec3 d_vel;    // = a_world
        Quat d_quat;   // = 0.5 * q * Quat(0, omega_b)
        Vec3 d_omega;  // = I^-1 * (tau_b - omega_b x I*omega_b)
    };

    Deriv          derive(const RigidBodyState& s,
                          const Vec3& force_b, const Vec3& torque_b) const;
    RigidBodyState advance(const RigidBodyState& s, const Deriv& k,
                           float dt) const;
    void           groundClamp();

    RigidBodyState state_;
    DroneParams    params_;
};

}  // namespace vsim

#endif  // VSIM_PHYSICS_CORE_H
