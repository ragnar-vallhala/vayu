#pragma once

#include <QQuaternion>
#include <QVector3D>
#include <array>
#include <cstdint>

// vsim - in-app drone simulator for the Navigator GCS.
//
// Coordinate frame is NED (North-East-Down):
//   +X forward (north), +Y right (east), +Z down.
// Gravity is +9.81 m/s^2 in +Z. This matches what the vayu firmware
// expects from the BMX160 driver, so no FLU<->NED transform is needed
// at the sensor boundary. (The firmware previously got NED-converted
// samples from gz_imu_to_vayu.py's FLU->NED step; the in-app sim
// models in NED natively and skips the conversion.)
//
// All state vectors, sensor outputs, motor positions, and wrenches
// below are in NED unless explicitly named *_b (body frame).
namespace vsim {

using Vec3 = QVector3D;
using Quat = QQuaternion;

constexpr float kG = 9.81f;     // m/s^2

struct RigidBodyState {
  Vec3 pos_w   = {0.0f, 0.0f, 0.0f};   // world position [m] (NED)
  Vec3 vel_w   = {0.0f, 0.0f, 0.0f};   // world velocity [m/s] (NED)
  Quat att     = Quat(1.0f, 0.0f, 0.0f, 0.0f);  // body->world, level
  Vec3 omega_b = {0.0f, 0.0f, 0.0f};   // body angular vel [rad/s]
};

// Quad params. Defaults are roughly X3-class - tune per actual airframe.
struct DroneParams {
  float mass         = 1.0f;                              // kg
  Vec3  inertia_diag = {0.012f, 0.012f, 0.022f};          // kg*m^2
  float linear_drag  = 0.10f;                             // N per (m/s)
  float angular_drag = 0.005f;                            // N*m per (rad/s)
  float ground_z     = 0.0f;                              // NED z of ground
  float ground_restitution = 0.0f;                        // bounce factor
};

// Motor placement & properties. M1=FR, M2=RR, M3=RL, M4=FL to match
// vayu firmware's motor mixing.
struct MotorParams {
  // Body-frame XY positions of each rotor (NED body: +X fwd, +Y right).
  std::array<Vec3, 4> pos_b = {
      Vec3{ 0.13f, +0.22f, 0.0f},   // M1 FR (right of body in NED-y)
      Vec3{-0.13f, +0.20f, 0.0f},   // M2 RR
      Vec3{-0.13f, -0.20f, 0.0f},   // M3 RL
      Vec3{ 0.13f, -0.22f, 0.0f},   // M4 FL
  };
  // Spin direction: +1 = CCW seen from above, -1 = CW. Vayu mixing
  // expects M1+M3 CCW, M2+M4 CW.
  std::array<int, 4> spin = {+1, -1, +1, -1};

  // Thrust coefficient: thrust_N = k_thrust * omega_rad_s^2 (per rotor).
  float k_thrust = 1.522e-5f;
  // Reaction torque coefficient about rotor axis (body +Z = down in NED,
  // CCW rotor produces +Z reaction torque on body from Newton's 3rd law
  // applied to NED-down convention -- spin sign carries this).
  // Magnitude: tau_N_m = k_moment * omega_rad_s^2.
  float k_moment = 2.44e-7f;

  // Throttle (0..1) -> commanded omega [rad/s].
  float max_omega = 1200.0f;

  // First-order rotor spin-up filter time constants [s] (asymmetric:
  // spin-up slower than spin-down on real props).
  float tau_up   = 0.0125f;
  float tau_down = 0.025f;

  // Motor idle: when motor_task forces 0 (disarmed) the bridge sees
  // duty=0; even tiny commanded duty maps to a real angular velocity
  // proportional to max_omega via duty * max_omega. No deadband.
};

// IMU sample in NED, body frame. Matches what BMX160 driver
// expects from the firmware side.
struct ImuSample {
  Vec3  acc;             // m/s^2 in body frame (specific force,
                         //   so at rest acc = -gravity_body = (0,0,-G))
  Vec3  gyr;             // rad/s in body frame (deg/s on the wire to fw)
  Vec3  mag;             // microtesla in body frame
  float temp;            // degrees C
};

// World magnetic field in NED, microtesla. Bengaluru-area defaults
// (declination ~0 deg, inclination ~30 deg, total ~44 uT). Adjust to
// match the mag calibration the firmware was tuned against if needed.
inline Vec3 magWorldNed() { return Vec3(38.0f, 0.0f, 22.0f); }

}  // namespace vsim
