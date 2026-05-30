#include "physics_core.h"

namespace vsim {

void PhysicsCore::reset(const RigidBodyState& initial) {
    state_ = initial;
    state_.att.normalize();
}

// d/dt of the state, given a body-frame wrench.
//
//   pos_dot   = vel_w
//   vel_dot   = (R * F_b - drag*vel_w) / m + (0,0,kG)         NED gravity +Z
//   quat_dot  = 0.5 * q * Quat(0, omega_b)
//   omega_dot = I^-1 * (tau_b - omega_b x I*omega_b - c_w * omega_b)
//
// Inertia is diagonal (typical for a symmetric quad) so I^-1 is just
// element-wise reciprocal.
PhysicsCore::Deriv PhysicsCore::derive(const RigidBodyState& s,
                                       const Vec3& force_b,
                                       const Vec3& torque_b) const {
    Deriv d;
    d.d_pos = s.vel_w;

    Vec3 F_world = s.att.rotatedVector(force_b);
    Vec3 a = F_world / params_.mass;
    a -= s.vel_w * (params_.linear_drag / params_.mass);
    a += Vec3(0.0f, 0.0f, kG);   // gravity is +Z in NED
    d.d_vel = a;

    // q_dot = 0.5 * q * (0, omega_b)
    Quat omega_q(0.0f, s.omega_b.x(), s.omega_b.y(), s.omega_b.z());
    Quat q_dot = s.att * omega_q;
    d.d_quat = Quat(0.5f * q_dot.scalar(),
                    0.5f * q_dot.x(),
                    0.5f * q_dot.y(),
                    0.5f * q_dot.z());

    // Euler's equations with diagonal inertia.
    const Vec3& I = params_.inertia_diag;
    Vec3 Iw(I.x() * s.omega_b.x(),
            I.y() * s.omega_b.y(),
            I.z() * s.omega_b.z());
    Vec3 wxIw = Vec3::crossProduct(s.omega_b, Iw);
    Vec3 net_tau = torque_b - wxIw - s.omega_b * params_.angular_drag;
    d.d_omega = Vec3(net_tau.x() / I.x(),
                     net_tau.y() / I.y(),
                     net_tau.z() / I.z());

    return d;
}

RigidBodyState PhysicsCore::advance(const RigidBodyState& s, const Deriv& k,
                                    float dt) const {
    RigidBodyState n;
    n.pos_w   = s.pos_w   + k.d_pos   * dt;
    n.vel_w   = s.vel_w   + k.d_vel   * dt;
    n.omega_b = s.omega_b + k.d_omega * dt;
    n.att     = Quat(s.att.scalar() + k.d_quat.scalar() * dt,
                     s.att.x()      + k.d_quat.x()      * dt,
                     s.att.y()      + k.d_quat.y()      * dt,
                     s.att.z()      + k.d_quat.z()      * dt);
    return n;
}

void PhysicsCore::step(const Vec3& force_b, const Vec3& torque_b, float dt) {
    // Classic RK4. Wrench is zero-order-held across the step.
    Deriv k1 = derive(state_,                              force_b, torque_b);
    Deriv k2 = derive(advance(state_, k1, dt * 0.5f),      force_b, torque_b);
    Deriv k3 = derive(advance(state_, k2, dt * 0.5f),      force_b, torque_b);
    Deriv k4 = derive(advance(state_, k3, dt),             force_b, torque_b);

    Deriv k;
    k.d_pos   = (k1.d_pos   + k2.d_pos   * 2.0f + k3.d_pos   * 2.0f + k4.d_pos)   * (1.0f/6.0f);
    k.d_vel   = (k1.d_vel   + k2.d_vel   * 2.0f + k3.d_vel   * 2.0f + k4.d_vel)   * (1.0f/6.0f);
    k.d_omega = (k1.d_omega + k2.d_omega * 2.0f + k3.d_omega * 2.0f + k4.d_omega) * (1.0f/6.0f);
    k.d_quat  = Quat(
        (k1.d_quat.scalar() + 2*k2.d_quat.scalar() + 2*k3.d_quat.scalar() + k4.d_quat.scalar()) * (1.0f/6.0f),
        (k1.d_quat.x()      + 2*k2.d_quat.x()      + 2*k3.d_quat.x()      + k4.d_quat.x())      * (1.0f/6.0f),
        (k1.d_quat.y()      + 2*k2.d_quat.y()      + 2*k3.d_quat.y()      + k4.d_quat.y())      * (1.0f/6.0f),
        (k1.d_quat.z()      + 2*k2.d_quat.z()      + 2*k3.d_quat.z()      + k4.d_quat.z())      * (1.0f/6.0f));

    state_ = advance(state_, k, dt);
    state_.att.normalize();

    groundClamp();
}

// Hard floor at z == ground_z (NED, so larger z is lower). When the
// body's CoM would dip below the ground, snap z to ground_z and zero
// out any downward (positive-z) velocity. Attitude is left alone --
// the contract is "simple landing clamp, no tipping".
void PhysicsCore::groundClamp() {
    if (state_.pos_w.z() > params_.ground_z) {
        state_.pos_w.setZ(params_.ground_z);
        if (state_.vel_w.z() > 0.0f) {
            state_.vel_w.setZ(-state_.vel_w.z() * params_.ground_restitution);
        }
        // Light tangential friction. Note: still per-tick, not per-second --
        // the original software/src/vsim/PhysicsCore.cpp had the same bug.
        // Carrying it across unchanged for now; the daemon refactor isn't
        // the place to alter physics behavior. See analysis notes.
        state_.vel_w.setX(state_.vel_w.x() * 0.99f);
        state_.vel_w.setY(state_.vel_w.y() * 0.99f);
    }
}

}  // namespace vsim
