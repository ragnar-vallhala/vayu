#include "physics_core.h"

#include <algorithm>
#include <cmath>

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
// Inertia is a full symmetric 3x3 tensor (mesh-derived airframes have
// products of inertia), so I^-1 is the cached matrix inverse I_inv_.
PhysicsCore::Deriv PhysicsCore::derive(const RigidBodyState& s,
                                       const Vec3& force_b,
                                       const Vec3& torque_b) const {
    Deriv d;
    d.d_pos = s.vel_w;

    Vec3 F_world = s.att.rotatedVector(force_b);
    Vec3 a = F_world / params_.mass;
    a -= s.vel_w * (params_.linear_drag / params_.mass);
    a += Vec3(0.0f, 0.0f, params_.gravity);   // gravity is +Z in NED
    d.d_vel = a;

    // q_dot = 0.5 * q * (0, omega_b)
    Quat omega_q(0.0f, s.omega_b.x(), s.omega_b.y(), s.omega_b.z());
    Quat q_dot = s.att * omega_q;
    d.d_quat = Quat(0.5f * q_dot.scalar(),
                    0.5f * q_dot.x(),
                    0.5f * q_dot.y(),
                    0.5f * q_dot.z());

    // Euler's equations with a full inertia tensor:
    //   omega_dot = I^-1 * (tau - omega x (I*omega) - c_w*omega)
    Vec3 Iw      = params_.inertia * s.omega_b;
    Vec3 wxIw    = Vec3::crossProduct(s.omega_b, Iw);
    Vec3 net_tau = torque_b - wxIw - s.omega_b * params_.angular_drag;
    d.d_omega    = I_inv_ * net_tau;

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

    groundClamp(dt);
    resolveObstacles();
    sanitize();
}

// Resolve drone-vs-obstacle contact. The drone is modelled as a small
// footprint (four arm points at +/-R, not just the CoM) so contacts off to
// one side apply an off-CoM impulse — tau = r x J — and the airframe tips off
// edges / catches a corner instead of sliding flat. Per contacting point we
// apply a normal impulse (with the body's rotational inertia coupled in) plus
// tangential friction, and lift the CoM out by the deepest penetration. NED.
void PhysicsCore::resolveObstacles() {
    if (obstacles_.empty()) return;
    constexpr float rc = 0.04f;   // contact radius per footprint point [m]
    constexpr float R  = 0.13f;   // drone footprint half-reach [m]
    const Vec3 fp[4] = {Vec3(R, R, 0), Vec3(-R, R, 0),
                        Vec3(-R, -R, 0), Vec3(R, -R, 0)};
    const float m = (params_.mass > 1e-3f) ? params_.mass : 1e-3f;
    auto dot = [](const Vec3& a, const Vec3& b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    };

    Vec3 posCorr(0, 0, 0);
    float maxPen = 0.0f;

    for (const SimObstacle& o : obstacles_) {
        // Rotation world<-local from Euler XYZ (deg): q = qx*qy*qz.
        auto axisQ = [](float deg, const Vec3& ax) {
            const float a = deg * float(M_PI) / 180.0f * 0.5f;
            const float s = std::sin(a);
            return Quat(std::cos(a), ax.x() * s, ax.y() * s, ax.z() * s);
        };
        const Quat q = axisQ(o.rot_deg.x(), Vec3(1, 0, 0)) *
                       axisQ(o.rot_deg.y(), Vec3(0, 1, 0)) *
                       axisQ(o.rot_deg.z(), Vec3(0, 0, 1));

        for (const Vec3& cpb : fp) {
            const Vec3 Pw = state_.pos_w + state_.att.rotatedVector(cpb);
            const Vec3 lp = q.conjugated().rotatedVector(Pw - o.pos);

            Vec3 nL;
            float pen = 0.0f;
            bool hit = false;
            if (o.type == 1) {  // sphere
                const float rad = o.size.x() + rc;
                const float d = std::sqrt(dot(lp, lp));
                if (d < rad && d > 1e-5f) { nL = lp / d; pen = rad - d; hit = true; }
            } else if (o.type == 2) {  // cylinder (axis = local z)
                const float rad = o.size.x() + rc, hh = o.size.z() * 0.5f + rc;
                const float rxy = std::sqrt(lp.x() * lp.x() + lp.y() * lp.y());
                if (rxy < rad && std::fabs(lp.z()) < hh) {
                    const float pr = rad - rxy, pz = hh - std::fabs(lp.z());
                    if (pr < pz && rxy > 1e-5f) {
                        nL = Vec3(lp.x() / rxy, lp.y() / rxy, 0); pen = pr;
                    } else {
                        nL = Vec3(0, 0, lp.z() >= 0 ? 1.f : -1.f); pen = pz;
                    }
                    hit = true;
                }
            } else {  // box
                const float hx = o.size.x() * 0.5f + rc;
                const float hy = o.size.y() * 0.5f + rc;
                const float hz = o.size.z() * 0.5f + rc;
                if (std::fabs(lp.x()) < hx && std::fabs(lp.y()) < hy &&
                    std::fabs(lp.z()) < hz) {
                    const float px = hx - std::fabs(lp.x());
                    const float py = hy - std::fabs(lp.y());
                    const float pz = hz - std::fabs(lp.z());
                    if (px <= py && px <= pz)
                        { nL = Vec3(lp.x() >= 0 ? 1.f : -1.f, 0, 0); pen = px; }
                    else if (py <= pz)
                        { nL = Vec3(0, lp.y() >= 0 ? 1.f : -1.f, 0); pen = py; }
                    else
                        { nL = Vec3(0, 0, lp.z() >= 0 ? 1.f : -1.f); pen = pz; }
                    hit = true;
                }
            }
            if (!hit) continue;

            const Vec3 nW = q.rotatedVector(nL);
            if (pen > maxPen) { maxPen = pen; posCorr = nW * pen; }

            // Contact-point velocity = v_cm + omega x r (lever = att*cpb).
            const Vec3 leverW = state_.att.rotatedVector(cpb);
            const Vec3 omegaW = state_.att.rotatedVector(state_.omega_b);
            const Vec3 vP = state_.vel_w + Vec3::crossProduct(omegaW, leverW);
            const float vn = dot(vP, nW);
            if (vn >= 0.0f) continue;  // separating: no impulse

            // Effective mass along the normal incl. rotational inertia, in the
            // body frame (where I_inv_ lives): k = 1/m + n·((I^-1 (r×n))×r).
            const Vec3 nB = state_.att.conjugated().rotatedVector(nW);
            const Vec3 rxn = Vec3::crossProduct(cpb, nB);
            const Vec3 ir = I_inv_ * rxn;
            float keff = 1.0f / m + dot(nB, Vec3::crossProduct(ir, cpb));
            if (keff < 1e-6f) keff = 1e-6f;
            // Split the impulse across the (up to 4) footprint points.
            const float j = -(1.0f + o.restitution) * vn / keff * 0.25f;

            const Vec3 Jw = nW * j;
            state_.vel_w += Jw / m;
            const Vec3 Jb = state_.att.conjugated().rotatedVector(Jw);
            state_.omega_b += I_inv_ * Vec3::crossProduct(cpb, Jb);

            // Coulomb-ish friction at the contact: opposes the tangential
            // slide, capped so it can't reverse it; also feeds spin via r×F.
            const Vec3 vt = vP - nW * vn;
            const float vtl = std::sqrt(dot(vt, vt));
            if (vtl > 1e-4f) {
                float jt = -0.4f * std::fabs(j);
                if (jt < -vtl * m * 0.25f) jt = -vtl * m * 0.25f;
                const Vec3 Ft = (vt / vtl) * jt;
                state_.vel_w += Ft / m;
                const Vec3 Ftb = state_.att.conjugated().rotatedVector(Ft);
                state_.omega_b += I_inv_ * Vec3::crossProduct(cpb, Ftb);
            }
        }
    }
    if (maxPen > 0.0f) state_.pos_w += posCorr;
}

// Reset on non-finite state, clamp runaway rates. A diverged controller
// (or a stiff config) can drive omega/vel toward inf; without this the
// first inf turns into NaN on the next step and poisons every downstream
// consumer (IMU, estimator, HUD) permanently.
void PhysicsCore::sanitize() {
    auto fin3 = [](const Vec3& v) {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    };
    const bool ok = fin3(state_.pos_w) && fin3(state_.vel_w) &&
                    fin3(state_.omega_b) && std::isfinite(state_.att.scalar()) &&
                    std::isfinite(state_.att.x()) && std::isfinite(state_.att.y()) &&
                    std::isfinite(state_.att.z());
    if (!ok) {
        const Vec3 keep = fin3(state_.pos_w) ? state_.pos_w
                                             : Vec3(0.0f, 0.0f, params_.ground_z - 0.05f);
        state_ = RigidBodyState{};   // level, zero vel/omega
        state_.pos_w = keep;
        return;
    }
    auto clampVec = [](Vec3& v, float lim) {
        v.setX(std::clamp(v.x(), -lim, lim));
        v.setY(std::clamp(v.y(), -lim, lim));
        v.setZ(std::clamp(v.z(), -lim, lim));
    };
    clampVec(state_.omega_b, 100.0f);   // rad/s — generous; real quad « this
    clampVec(state_.vel_w, 200.0f);     // m/s
}

// Hard floor at z == ground_z (NED, so larger z is lower). When the
// body's CoM would dip below the ground, snap z to ground_z and zero
// out any downward (positive-z) velocity. Also topple a tipped airframe:
// it can't balance on an edge, so gravity about the contact patch rotates
// it toward level.
void PhysicsCore::groundClamp(float dt) {
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

        // Ground-contact righting: a resting airframe topples toward level
        // (a quad can't balance on a tilted edge). worldUp = NED -Z; bodyUp
        // is where the body's up axis points in world. The cross product is a
        // world-frame axis that rotates bodyUp back onto worldUp; magnitude
        // ~ sin(tilt). Drive omega toward that (damped) so a tipped drone
        // tumbles flat instead of floating on its side.
        const Vec3 worldUp(0.0f, 0.0f, -1.0f);
        const Vec3 bodyUp = state_.att.rotatedVector(worldUp);
        const Vec3 tiltW = Vec3::crossProduct(bodyUp, worldUp);  // body up -> up
        const float tilt2 = tiltW.x() * tiltW.x() + tiltW.y() * tiltW.y() +
                            tiltW.z() * tiltW.z();
        if (tilt2 > 1e-6f) {
            // Body-frame angular accel: righting gain * tilt, minus damping.
            const Vec3 tiltB = state_.att.conjugated().rotatedVector(tiltW);
            const float kRight = params_.ground_right_gain;  // rad/s^2 per sin(tilt)
            const float kDamp  = params_.ground_right_damp;  // 1/s
            state_.omega_b += (tiltB * kRight - state_.omega_b * kDamp) * dt;
        }
    }
}

}  // namespace vsim
