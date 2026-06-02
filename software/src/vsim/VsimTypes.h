#pragma once

#include <QQuaternion>
#include <QString>
#include <QVector>
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
  QVector3D translate{0, 0, 0};  // body-frame offset applied to the mesh [m]
  QVector3D rotate{0, 0, 0};     // body-frame orientation, XYZ Euler [deg]
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

// Environment + aerodynamics edited in the World tab; pushed to vsim_d via
// SimWorker::sendWorld -> VSIM_CTL_SET_WORLD. NED: +Z is down, so gravity
// is positive and the ground plane sits at ground_z.
// A static world obstacle. Rendered GCS-side now; collision in vsim_d is a
// later phase (restitution carried so the wire format won't change then).
// All in the NED world frame the drone pose uses (z down → on-ground center
// sits at z = -size.z/2). size: box = full extents; sphere = x is radius;
// cylinder = x is radius, z is height.
struct Obstacle {
  enum Type { Box = 0, Sphere = 1, Cylinder = 2 };
  int type = Box;
  QVector3D pos{2.0f, 0.0f, -0.5f};
  QVector3D size{1.0f, 1.0f, 1.0f};
  QVector3D rotate{0.0f, 0.0f, 0.0f};  // Euler XYZ [deg]
  float restitution = 0.3f;
};

struct WorldConfig {
  float gravity = 9.81f;          // m/s^2
  float ground_z = 0.0f;          // NED z of ground [m]
  float restitution = 0.0f;       // bounce factor [0,1]
  float linear_drag = 0.10f;      // N per (m/s)
  float angular_drag = 0.005f;    // N*m per (rad/s)
  // Ground-contact righting: how hard/fast a tipped airframe topples to level.
  float ground_right_gain = 40.0f;  // rad/s² per sin(tilt)
  float ground_right_damp = 6.0f;   // 1/s
  QVector<Obstacle> obstacles;    // static world shapes

  // Imported world mesh (rendered now; collidable in a later phase).
  QString   worldMeshPath;        // empty = none
  float     worldScale = 1.0f;    // mesh units -> metres
  int       worldUpAxis = 0;      // 0 = Z-up (Blender), 1 = Y-up (glTF)
  float     worldMeshRestitution = 0.3f;
  bool      worldMeshDoubleSided = true;
};

}  // namespace vsim
