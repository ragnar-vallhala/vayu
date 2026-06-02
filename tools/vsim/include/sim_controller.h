// sim_controller.h — composes PhysicsCore + MotorModel + SensorModels
// and exposes a single tick entry point. Ported verbatim from
// software/src/vsim/SimController.h.
#ifndef VSIM_SIM_CONTROLLER_H
#define VSIM_SIM_CONTROLLER_H

#include "motor_model.h"
#include "physics_core.h"
#include "sensor_models.h"
#include "vsim_types.h"

namespace vsim {

class SimController {
public:
    SimController() = default;

    void setDroneParams (const DroneParams& p)  { phys_.setParams(p); }
    void setMotorParams (const MotorParams& p)  { motors_.setParams(p); }
    void setNoise       (const SensorNoise& n)  { sensors_.setNoise(n); }
    void setObstacles   (const std::vector<SimObstacle>& o) { phys_.setObstacles(o); }
    void seedSensors    (uint64_t s)            { sensors_.seed(s); }

    void resetState(const RigidBodyState& s = {});

    // Advances by dt. Returns the IMU sample produced at this step.
    ImuSample tick(const std::array<float, 4>& duty, float dt);

    // Substep-friendly split of tick(): advance the motors + rigid body by dt
    // (no sensor read), then sample the IMU once after N substeps. Lets the
    // daemon integrate physics faster than it samples/paces.
    void      stepOnce(const std::array<float, 4>& duty, float dt);
    ImuSample sampleImu(float dt);

    const RigidBodyState&        state()       const { return phys_.state(); }
    const std::array<float, 4>&  motorOmegas() const { return motors_.omegas(); }

private:
    PhysicsCore  phys_;
    MotorModel   motors_;
    SensorModels sensors_;

    // Stashed last world acceleration so the accelerometer can subtract
    // gravity correctly. Specific force = a_world - g.
    Vec3 last_a_world_{0.0f, 0.0f, 0.0f};
};

}  // namespace vsim

#endif  // VSIM_SIM_CONTROLLER_H
