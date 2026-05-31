// sensor_models.h — synthetic IMU/mag with noise + random-walk bias.
// Ported from software/src/vsim/SensorModels.h, Qt-free.
#ifndef VSIM_SENSOR_MODELS_H
#define VSIM_SENSOR_MODELS_H

#include "vsim_types.h"
#include <random>

namespace vsim {

struct SensorNoise {
    float acc_noise_std    = 0.10f;
    float acc_bias_walk    = 0.0005f;
    float acc_bias_clip    = 0.20f;

    float gyr_noise_std    = 0.0087f;
    float gyr_bias_walk    = 1e-5f;
    float gyr_bias_clip    = 0.035f;

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
};

}  // namespace vsim

#endif  // VSIM_SENSOR_MODELS_H
