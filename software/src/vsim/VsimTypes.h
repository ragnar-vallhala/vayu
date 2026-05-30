#pragma once

#include <QQuaternion>
#include <QString>
#include <QVector3D>

#include <array>

// Navigator-side type aliases for SimSnapshot fields. The full physics
// state structures (RigidBodyState, DroneParams, MotorParams,
// SensorNoise, ImuSample) used to live here when the simulator was
// in-process; they now live in tools/vsim/include/vsim_types.h on the
// daemon side. Navigator only consumes pose snapshots, so all it needs
// is the Qt-typed Vec3 / Quat the renderer reads off SimSnapshot.
namespace vsim {

using Vec3 = QVector3D;
using Quat = QQuaternion;

// Per-rotor configuration as edited in the GeometryEditorWidget. Mirrors
// (a subset of) the daemon-side MotorParams; pushed to vsim_d via
// SimWorker::sendGeometry -> VSIM_CTL_SET_GEOMETRY.
struct MotorConfig {
  QVector3D pos{0, 0, 0};        // body-frame [m]
  QVector3D axis{0, 0, -1};      // unit thrust axis (default -Z = up in NED)
  int spin = +1;                 // +1 CCW / -1 CW (seen from above)
  float k_thrust = 1.522e-5f;    // thrust_N = k_thrust * omega^2
  float k_moment = 2.44e-7f;     // reaction torque magnitude
  float max_omega = 1200.0f;     // duty=1 omega [rad/s]
};

// The full airframe geometry the editor produces: a mesh (GCS-side only,
// for rendering + inertia computation) plus the mass properties and motor
// layout pushed to the daemon. meshPath/scale/com never cross the wire.
struct GeometryConfig {
  QString meshPath;
  float scale = 1.0f;            // meters per mesh unit
  float mass = 1.0f;             // kg
  // Inertia tensor about the CoM, body frame [kg*m^2], row-major.
  std::array<float, 9> inertia =
      {0.012f, 0, 0, 0, 0.012f, 0, 0, 0, 0.022f};
  QVector3D com{0, 0, 0};        // CoM offset from model origin [m]
  std::array<MotorConfig, 4> motors = {{
      {{ 0.13f, +0.22f, 0}, {0, 0, -1}, +1, 1.522e-5f, 2.44e-7f, 1200.0f},
      {{-0.13f, +0.20f, 0}, {0, 0, -1}, -1, 1.522e-5f, 2.44e-7f, 1200.0f},
      {{-0.13f, -0.20f, 0}, {0, 0, -1}, +1, 1.522e-5f, 2.44e-7f, 1200.0f},
      {{ 0.13f, -0.22f, 0}, {0, 0, -1}, -1, 1.522e-5f, 2.44e-7f, 1200.0f},
  }};
};

}  // namespace vsim
