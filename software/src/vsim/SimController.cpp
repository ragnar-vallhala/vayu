#include "SimController.h"

namespace vsim {

SimController::SimController() = default;

void SimController::resetState(const RigidBodyState& s) {
  phys_.reset(s);
  motors_.reset();
  last_a_world_ = Vec3(0.0f, 0.0f, 0.0f);
}

ImuSample SimController::tick(const std::array<float, 4>& duty, float dt) {
  // 1) Motors -> body-frame wrench
  Vec3 F_b, T_b;
  motors_.update(duty, dt, &F_b, &T_b);

  // 2) Stash vel before stepping so we can derive a_world after.
  Vec3 vel_before = phys_.state().vel_w;

  // 3) Step physics
  phys_.step(F_b, T_b, dt);

  // 4) Approximate a_world = dvel/dt. This is what the accelerometer
  //    "feels" once gravity is subtracted out -- doing the divide
  //    here keeps the SensorModels independent of how the integrator
  //    arranged the substeps. (For a more "correct" reading we could
  //    have PhysicsCore expose its instantaneous a, but this is
  //    indistinguishable for the firmware's mahony filter.)
  if (dt > 0.0f) {
    last_a_world_ = (phys_.state().vel_w - vel_before) / dt;
  }

  // 5) Synthesize the sensor sample
  return sensors_.sample(phys_.state(), last_a_world_, dt);
}

}  // namespace vsim
