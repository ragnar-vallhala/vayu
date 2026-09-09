// sim_controller.h — composes PhysicsCore + MotorModel + SensorModels
// and exposes a single tick entry point. Ported verbatim from
// navigator/src/vsim/SimController.h.
#ifndef VSIM_SIM_CONTROLLER_H
#define VSIM_SIM_CONTROLLER_H

#include "motor_model.h"
#include "physics_core.h"
#include "sensor_models.h"
#include "vsim_types.h"
#include "wind_model.h"

namespace vsim {

class SimController {
public:
  SimController() = default;

  void setDroneParams(const DroneParams &p) { phys_.setParams(p); }
  void setMotorParams(const MotorParams &p) { motors_.setParams(p); }
  void setNoise(const SensorNoise &n) { sensors_.setNoise(n); }
  void setObstacles(const std::vector<SimObstacle> &o) {
    phys_.setObstacles(o);
  }
  void setWorldMesh(const trimesh::Bvh &b, float rest) {
    phys_.setWorldMesh(b, rest);
  }
  void clearWorldMesh() { phys_.clearWorldMesh(); }
  void setTestRig(bool on, const Vec3 &pos, float tether_k = 0.0f) {
    phys_.setTestRig(on, pos, tether_k);
  }
  void seedSensors(uint64_t s) { sensors_.seed(s); }
  // Thrust-scaled vibration gain [g per unit mean motor command], applied to
  // the IMU each physics step from the live duty. 0 = off (default).
  void setVibeGain(float g_per_unit) { vibe_gain_ = g_per_unit; }
  void setWind(const WindConfig &w) { wind_.setConfig(w); }
  void seedWind(uint64_t s) { wind_.seed(s); }
  // Last instantaneous world wind [m/s] NED, for the pose-frame telemetry.
  const Vec3 &windWorld() const { return wind_.world(); }

  void resetState(const RigidBodyState &s = {});

  // Advances by dt. Returns the IMU sample produced at this step.
  ImuSample tick(const std::array<float, 4> &duty, float dt);

  // Substep-friendly split of tick(): advance the motors + rigid body by dt
  // (no sensor read), then sample the IMU once after N substeps. Lets the
  // daemon integrate physics faster than it samples/paces.
  void stepOnce(const std::array<float, 4> &duty, float dt);
  ImuSample sampleImu(float dt);

  const RigidBodyState &state() const { return phys_.state(); }
  const std::array<float, 4> &motorOmegas() const { return motors_.omegas(); }

private:
  PhysicsCore phys_;
  MotorModel motors_;
  SensorModels sensors_;
  WindModel wind_;

  // Stashed last world acceleration so the accelerometer can subtract
  // gravity correctly. Specific force = a_world - g.
  Vec3 last_a_world_{0.0f, 0.0f, 0.0f};
  // Seconds remaining to report a clean gravity reaction after the last ground
  // contact. Bridges the brief airborne phases of the resting-contact bounce
  // limit cycle so a parked airframe reads a steady -g, not impulse noise.
  float ground_hold_s_ = 0.0f;
  float vibe_gain_ = 0.0f; // g per unit mean motor cmd (0 = off)
};

} // namespace vsim

#endif // VSIM_SIM_CONTROLLER_H
