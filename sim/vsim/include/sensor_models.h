// sensor_models.h — synthetic IMU/mag with noise + random-walk bias.
// Ported from navigator/src/vsim/SensorModels.h, Qt-free.
#ifndef VSIM_SENSOR_MODELS_H
#define VSIM_SENSOR_MODELS_H

#include "vsim_types.h"
#include <random>

namespace vsim {

struct SensorNoise {
    // Defaults sized to a real BMX160 at a few-hundred-Hz bandwidth:
    // accel ~0.03 m/s^2 RMS, gyro ~0.08 deg/s RMS (= 0.0014 rad/s); bias
    // clips kept tight so the resting mean doesn't wander further than a
    // calibrated part would.
    float acc_noise_std    = 0.03f;     // m/s^2 RMS
    float acc_bias_walk    = 0.0002f;
    float acc_bias_clip    = 0.08f;

    float gyr_noise_std    = 0.0014f;   // rad/s RMS ~= 0.08 deg/s
    float gyr_bias_walk    = 5e-6f;
    float gyr_bias_clip    = 0.012f;    // ~0.7 deg/s

    float mag_noise_std    = 0.3f;
    float mag_bias_walk    = 0.001f;
    float mag_bias_clip    = 5.0f;

    float temp_mean        = 25.0f;
    float temp_noise_std   = 0.05f;
};

class SensorModels {
public:
    SensorModels();

    void setNoise(const SensorNoise& n) { noise_ = n; }
    const SensorNoise& noise() const { return noise_; }

    // Per-step thrust-scaled vibration std added on top of the noise floor
    // (acc in m/s^2, gyr in rad/s). 0 = off (default). Set from the motor
    // command each physics step by SimController.
    void setVibe(float acc_std, float gyr_std) {
        vibe_acc_std_ = acc_std; vibe_gyr_std_ = gyr_std;
    }

    void seed(uint64_t s);

    // Build one IMU sample from current rigid-body state + the world
    // acceleration we just integrated. Accelerometer measures specific
    // force (= a_world - gravity_world) rotated into body frame.
    ImuSample sample(const RigidBodyState& state,
                     const Vec3& a_world,
                     float gravity,
                     float dt);

private:
    Vec3 walk(Vec3& bias, float walk_std, float clip);
    float randn(float std);

    SensorNoise noise_;
    std::mt19937_64                 rng_;
    std::normal_distribution<float> norm_;

    Vec3 acc_bias_{0.0f, 0.0f, 0.0f};
    Vec3 gyr_bias_{0.0f, 0.0f, 0.0f};
    Vec3 mag_bias_{0.0f, 0.0f, 0.0f};

    float vibe_acc_std_ = 0.0f;   // m/s^2, thrust-scaled (0 = off)
    float vibe_gyr_std_ = 0.0f;   // rad/s, thrust-scaled (0 = off)
};

}  // namespace vsim

#endif  // VSIM_SENSOR_MODELS_H
