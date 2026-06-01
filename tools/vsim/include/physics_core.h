// physics_core.h — single-rigid-body RK4 integrator. Ported from
// software/src/vsim/PhysicsCore.h with Qt math types swapped for the
// Qt-free Vec3 / Quat in vsim_math.h. Behavior is bit-equivalent.
#ifndef VSIM_PHYSICS_CORE_H
#define VSIM_PHYSICS_CORE_H

#include "vsim_types.h"

#include <vector>

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
    // Caches the inverse inertia tensor so the RK4 derive() (4x per
    // 1 kHz step) doesn't re-invert a 3x3 every call.
    void setParams(const DroneParams& p) { params_ = p; I_inv_ = params_.inertia.inverse(); }

    // Static world obstacles the drone collides with (push-out + restitution).
    void setObstacles(const std::vector<SimObstacle>& o) { obstacles_ = o; }

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
    void           resolveObstacles();   // push the CoM out of any obstacle
    // Reset on non-finite state and clamp runaway rates so a control
    // divergence can't permanently poison the sim with NaN/inf.
    void           sanitize();

    RigidBodyState state_;
    DroneParams    params_;
    std::vector<SimObstacle> obstacles_;
    // Cached inverse of params_.inertia; kept in sync by setParams().
    // Default matches the default DroneParams inertia.
    Mat3           I_inv_ = DroneParams{}.inertia.inverse();
};

}  // namespace vsim

#endif  // VSIM_PHYSICS_CORE_H
