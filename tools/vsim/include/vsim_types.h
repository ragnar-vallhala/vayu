// vsim_types.h — physics-side state structures for vsim_d.
//
// Same fields and units as the in-process port that lived in
// software/src/vsim/VsimTypes.h, minus the Qt dependency. NED frame
// throughout (+X north, +Y east, +Z down). Defaults are X3-class.
#ifndef VSIM_TYPES_H
#define VSIM_TYPES_H

#include "vsim_math.h"
#include <array>
#include <cstdint>

namespace vsim {

constexpr float kG = 9.81f;    // m/s^2

struct RigidBodyState {
    Vec3 pos_w   = {0.0f, 0.0f, 0.0f};   // world position [m] (NED)
    Vec3 vel_w   = {0.0f, 0.0f, 0.0f};   // world velocity [m/s] (NED)
    Quat att     = Quat(1.0f, 0.0f, 0.0f, 0.0f);  // body->world, level
    Vec3 omega_b = {0.0f, 0.0f, 0.0f};   // body angular vel [rad/s]
};

// Quad params. Defaults are roughly X3-class.
struct DroneParams {
    float mass         = 1.0f;                              // kg
    // Full body-frame inertia tensor [kg*m^2]. Default is the old
    // diagonal X3 value; a mesh-derived tensor (with products of
    // inertia) can be pushed in via VSIM_CTL_SET_GEOMETRY.
    Mat3  inertia      = Mat3::diagonal(0.012f, 0.012f, 0.022f);
    float linear_drag  = 0.10f;                             // N per (m/s)
    float angular_drag = 0.005f;                            // N*m per (rad/s)
    float ground_z     = 0.0f;                              // NED z of ground
    float ground_restitution = 0.0f;                        // bounce factor
    float gravity      = kG;                                // m/s^2 (world, +Z down)
    float ground_right_gain = 40.0f;   // tipped-airframe righting gain
    float ground_right_damp = 6.0f;    // righting angular damping [1/s]
};

// M1=FR, M2=RR, M3=RL, M4=FL — matches firmware motor mixing. Positions
// are body-frame [m] relative to the CoM; thrust axes are body-frame
// unit vectors (default body -Z, i.e. up in NED). All per-rotor so the
// motor-mapping editor can drive each independently.
struct MotorParams {
    std::array<Vec3, 4> pos_b = {
        Vec3{ 0.13f, +0.22f, 0.0f},   // M1 FR
        Vec3{-0.13f, +0.20f, 0.0f},   // M2 RR
        Vec3{-0.13f, -0.20f, 0.0f},   // M3 RL
        Vec3{ 0.13f, -0.22f, 0.0f},   // M4 FL
    };
    // Per-rotor thrust axis (unit, body frame). Body +Z is DOWN in NED,
    // so lift points body -Z.
    std::array<Vec3, 4> axis_b = {
        Vec3{0.0f, 0.0f, -1.0f}, Vec3{0.0f, 0.0f, -1.0f},
        Vec3{0.0f, 0.0f, -1.0f}, Vec3{0.0f, 0.0f, -1.0f},
    };
    // +1 = CCW seen from above, -1 = CW. Vayu mixing: M1+M3 CCW, M2+M4 CW.
    std::array<int, 4> spin = {+1, -1, +1, -1};

    // thrust_N = k_thrust * omega^2 per rotor.
    std::array<float, 4> k_thrust = {1.522e-5f, 1.522e-5f, 1.522e-5f, 1.522e-5f};
    // reaction-torque magnitude per rotor.
    std::array<float, 4> k_moment = {2.44e-7f, 2.44e-7f, 2.44e-7f, 2.44e-7f};
    // duty=1 commands this omega [rad/s], per rotor.
    std::array<float, 4> max_omega = {1200.0f, 1200.0f, 1200.0f, 1200.0f};

    // First-order rotor spin-up filter time constants (asymmetric, shared).
    float tau_up   = 0.0125f;
    float tau_down = 0.025f;
};

struct ImuSample {
    Vec3  acc;             // m/s^2 in body frame, specific force
    Vec3  gyr;             // rad/s in body frame
    Vec3  mag;             // microtesla in body frame
    float temp;            // degC
};

// Bengaluru-area defaults: declination ~0 deg, inclination ~30 deg,
// total ~44 uT.
inline Vec3 magWorldNed() { return Vec3(38.0f, 0.0f, 22.0f); }

// A static world obstacle the drone (a point + small radius) collides with.
// NED world frame. type: 0 box (size = full extents), 1 sphere (size.x is
// radius), 2 cylinder (size.x radius, size.z height; axis = local z).
struct SimObstacle {
    int   type = 0;
    Vec3  pos;
    Vec3  size{1.0f, 1.0f, 1.0f};
    Vec3  rot_deg;
    float restitution = 0.3f;
};

}  // namespace vsim

#endif  // VSIM_TYPES_H
