#include "sim_controller.h"

namespace vsim {

// Accelerometer low-pass time constant [s]. ~3 ms ≈ 53 Hz: high enough to pass
// real flight accelerations, low enough to swallow the one-step ground-clamp
// impulses that would otherwise dominate the resting Z-axis noise.
static constexpr float kAccelLpfRc = 0.003f;

// Hold a clean gravity reaction for this long after the last ground contact, to
// bridge the resting-contact bounce limit cycle so a parked airframe reads a
// steady -g instead of the hard clamp's per-step velocity impulses.
static constexpr float kGroundHoldS = 0.3f;

void SimController::resetState(const RigidBodyState& s) {
    phys_.reset(s);
    motors_.reset();
    wind_.reset();
    last_a_world_ = Vec3(0.0f, 0.0f, 0.0f);
}

void SimController::stepOnce(const std::array<float, 4>& duty, float dt) {
    Vec3 F_b, T_b;
    motors_.update(duty, dt, &F_b, &T_b);

    // Thrust-scaled IMU vibration: scale the per-step injection by the mean
    // motor command so it rises from the noise floor at idle to ~vibe_gain g
    // at full throttle (real: ~0.06 g idle -> ~1.5 g active). No-op when off.
    if (vibe_gain_ > 0.0f) {
        float mean_cmd = 0.25f * (std::clamp(duty[0], 0.0f, 1.0f) +
                                  std::clamp(duty[1], 0.0f, 1.0f) +
                                  std::clamp(duty[2], 0.0f, 1.0f) +
                                  std::clamp(duty[3], 0.0f, 1.0f));
        float acc_std = vibe_gain_ * 9.81f * mean_cmd;     // m/s^2
        sensors_.setVibe(acc_std, acc_std * 0.05f);        // gyr ~ small frac
    }

    // Advance the wind field and hand the physics integrator this step's
    // air-relative wind (still air -> zero -> original drag behaviour).
    phys_.setWind(wind_.step(dt));

    Vec3 vel_before = phys_.state().vel_w;
    phys_.step(F_b, T_b, dt);

    // Approximate a_world via finite-difference of vel_w. This averages over
    // the step and, critically, picks up the single-step velocity impulses the
    // hard ground clamp injects when the craft rests on the ground -- which
    // otherwise surface as a huge phantom vertical-accel noise (sigma ~1 m/s^2
    // on Z while parked). A first-order low-pass models a real accelerometer's
    // limited bandwidth and attenuates those one-step impulses, while passing
    // genuine (slower) accelerations. test-rig mode pins translation so a_world
    // is ~0 there and the filter is a no-op.
    if (dt > 0.0f) {
        Vec3 raw_a = (phys_.state().vel_w - vel_before) / dt;
        // A body resting on the ground has ~zero proper acceleration (the ground
        // reaction just cancels gravity), but the hard position clamp injects a
        // per-step velocity impulse that the finite difference blows up into a
        // ~1g phantom on Z. The clamp also drives a small bounce limit cycle, so
        // the craft is briefly airborne between contacts. Hold a clean zero for a
        // short window after each ground contact to bridge those bounces, so a
        // parked airframe reads a steady -g. Genuine takeoff (no contact for the
        // hold window) lets real acceleration through.
        if (phys_.grounded()) ground_hold_s_ = kGroundHoldS;
        if (ground_hold_s_ > 0.0f) {
            raw_a = Vec3(0.0f, 0.0f, 0.0f);
            ground_hold_s_ -= dt;
        }
        const float alpha = dt / (dt + kAccelLpfRc);   // kAccelLpfRc ~3 ms ~= 53 Hz
        last_a_world_ = last_a_world_ + (raw_a - last_a_world_) * alpha;
    }
}

ImuSample SimController::sampleImu(float dt) {
    return sensors_.sample(phys_.state(), last_a_world_, phys_.params().gravity, dt);
}

ImuSample SimController::tick(const std::array<float, 4>& duty, float dt) {
    stepOnce(duty, dt);
    return sampleImu(dt);
}

}  // namespace vsim
