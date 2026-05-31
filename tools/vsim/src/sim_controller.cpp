#include "sim_controller.h"

namespace vsim {

void SimController::resetState(const RigidBodyState& s) {
    phys_.reset(s);
    motors_.reset();
    last_a_world_ = Vec3(0.0f, 0.0f, 0.0f);
}

ImuSample SimController::tick(const std::array<float, 4>& duty, float dt) {
    Vec3 F_b, T_b;
    motors_.update(duty, dt, &F_b, &T_b);

    Vec3 vel_before = phys_.state().vel_w;
    phys_.step(F_b, T_b, dt);

    // Approximate a_world via finite-difference of vel_w. This is the
    // same approximation the in-process port used; see physics-correctness
    // notes for the caveats (averages over the step, picks up ground-
    // clamp impulses). Carry forward unchanged in the daemon refactor.
    if (dt > 0.0f) {
        last_a_world_ = (phys_.state().vel_w - vel_before) / dt;
    }

    return sensors_.sample(phys_.state(), last_a_world_, phys_.params().gravity, dt);
}

}  // namespace vsim
