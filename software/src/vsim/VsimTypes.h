#pragma once

#include <QQuaternion>
#include <QVector3D>

// Navigator-side type aliases for SimSnapshot fields. The full physics
// state structures (RigidBodyState, DroneParams, MotorParams,
// SensorNoise, ImuSample) used to live here when the simulator was
// in-process; they now live in tools/vsim/include/vsim_types.h on the
// daemon side. Navigator only consumes pose snapshots, so all it needs
// is the Qt-typed Vec3 / Quat the renderer reads off SimSnapshot.
namespace vsim {

using Vec3 = QVector3D;
using Quat = QQuaternion;

}  // namespace vsim
