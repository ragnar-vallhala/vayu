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
// physics_core.h — single-rigid-body RK4 integrator. Ported from
// navigator/src/vsim/PhysicsCore.h with Qt math types swapped for the
// Qt-free Vec3 / Quat in vsim_math.h. Behavior is bit-equivalent.
#ifndef VSIM_PHYSICS_CORE_H
#define VSIM_PHYSICS_CORE_H

#include "trimesh_bvh.h"
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

  void reset(const RigidBodyState &initial = {});
  // Caches the inverse inertia tensor so the RK4 derive() (4x per
  // 8 kHz step) doesn't re-invert a 3x3 every call.
  void setParams(const DroneParams &p) {
    params_ = p;
    I_inv_ = params_.inertia.inverse();
  }

  // Static world obstacles the drone collides with (push-out + restitution).
  void setObstacles(const std::vector<SimObstacle> &o) { obstacles_ = o; }

  // Imported static world mesh (a non-owning BVH view over mmap'd bytes).
  void setWorldMesh(const trimesh::Bvh &b, float restitution) {
    world_mesh_ = b;
    world_mesh_restitution_ = restitution;
    has_world_mesh_ = b.valid();
  }
  void clearWorldMesh() {
    world_mesh_ = trimesh::Bvh{};
    has_world_mesh_ = false;
  }

  // Test-rig mode: leave rotation free and hold translation near `pos` — a
  // frictionless attitude gimbal for autotuning. tether_k == 0 hard-pins
  // translation (zero linear velocity each step); tether_k > 0 instead pulls
  // the body back with a critically-damped spring (stiffness [1/s^2]) so it
  // can translate during a maneuver and the accel sees free-flight thrust-tilt.
  // off restores normal free-flight integration + ground/obstacle contact.
  void setTestRig(bool on, const Vec3 &pos, float tether_k = 0.0f) {
    test_rig_ = on;
    rig_pos_ = pos;
    tether_k_ = tether_k;
  }

  // World-frame wind velocity [m/s] NED. Drag acts on the air-relative
  // velocity v_rel = vel_w - wind, so a hovering craft in steady wind is
  // pushed (drag no longer zeroes at zero ground speed). Default zero =
  // still air = original behaviour. Set once per step by the controller.
  void setWind(const Vec3 &wind_w) { wind_w_ = wind_w; }

  // One RK4 step. force_b and torque_b are body-frame.
  void step(const Vec3 &force_b, const Vec3 &torque_b, float dt);

  const RigidBodyState &state() const { return state_; }
  const DroneParams &params() const { return params_; }
  // True if the last step ended resting on the ground plane. Lets the IMU
  // model report a clean gravity reaction instead of the per-step ground-
  // clamp impulses the finite-difference accel would otherwise pick up.
  bool grounded() const { return grounded_; }

private:
  struct Deriv {
    Vec3 d_pos;   // = vel_w
    Vec3 d_vel;   // = a_world
    Quat d_quat;  // = 0.5 * q * Quat(0, omega_b)
    Vec3 d_omega; // = I^-1 * (tau_b - omega_b x I*omega_b)
  };

  Deriv derive(const RigidBodyState &s, const Vec3 &force_b,
               const Vec3 &torque_b) const;
  RigidBodyState advance(const RigidBodyState &s, const Deriv &k,
                         float dt) const;
  void groundClamp(float dt);
  void resolveObstacles(Vec3 &posCorr, float &maxPen);
  void resolveWorldMesh(Vec3 &posCorr, float &maxPen);
  void applyContact(const Vec3 &cpb, const Vec3 &nW, float pen,
                    float restitution, Vec3 &posCorr, float &maxPen);
  // Reset on non-finite state and clamp runaway rates so a control
  // divergence can't permanently poison the sim with NaN/inf.
  void sanitize();

  RigidBodyState state_;
  DroneParams params_;
  std::vector<SimObstacle> obstacles_;
  trimesh::Bvh world_mesh_; // non-owning view (mmap'd in main.cpp)
  bool has_world_mesh_ = false;
  float world_mesh_restitution_ = 0.3f;
  bool grounded_ = false;          // resting on the ground this step
  bool test_rig_ = false;          // hold translation, free rotation
  Vec3 rig_pos_{0.0f, 0.0f, 0.0f}; // held position when test_rig_
  float tether_k_ = 0.0f;          // >0: soft spring instead of hard pin
  Vec3 wind_w_{0.0f, 0.0f, 0.0f};  // world-frame wind [m/s]; 0 = still air
  // Cached inverse of params_.inertia; kept in sync by setParams().
  // Default matches the default DroneParams inertia.
  Mat3 I_inv_ = DroneParams{}.inertia.inverse();
};

} // namespace vsim

#endif // VSIM_PHYSICS_CORE_H
