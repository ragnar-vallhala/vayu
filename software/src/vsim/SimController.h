#pragma once

#include "MotorModel.h"
#include "PhysicsCore.h"
#include "SensorModels.h"
#include "VsimTypes.h"

namespace vsim {

// Composes physics + motors + sensors and exposes a single tick
// entry point. Stateless w.r.t. timing: the caller is responsible
// for choosing dt and call cadence (SimWorker does that).
//
// One tick:
//   1. read motor duty (passed in)
//   2. MotorModel.update -> body wrench, latest rotor omegas
//   3. PhysicsCore.step  -> integrate state
//   4. SensorModels.sample -> emit synthetic IMU/mag for the firmware
class SimController {
 public:
  SimController();

  void setDroneParams (const DroneParams& p)  { phys_.setParams(p); }
  void setMotorParams (const MotorParams& p)  { motors_.setParams(p); }
  void setNoise       (const SensorNoise& n)  { sensors_.setNoise(n); }
  void seedSensors    (uint64_t s)            { sensors_.seed(s); }

  void resetState(const RigidBodyState& s = {});

  // Advances by dt. Returns the IMU sample produced at this step.
  ImuSample tick(const std::array<float, 4>& duty, float dt);

  const RigidBodyState& state()  const { return phys_.state(); }
  const std::array<float, 4>& motorOmegas() const { return motors_.omegas(); }

 private:
  PhysicsCore  phys_;
  MotorModel   motors_;
  SensorModels sensors_;

  // Stash the last world acceleration so the accelerometer model
  // can subtract gravity correctly. (Specific force = a_world - g.)
  Vec3 last_a_world_{0.0f, 0.0f, 0.0f};
};

}  // namespace vsim
