// motor_model.h — per-rotor first-order dynamics + thrust/torque
// aggregation. Ported from navigator/src/vsim/MotorModel.h, Qt-free.
#ifndef VSIM_MOTOR_MODEL_H
#define VSIM_MOTOR_MODEL_H

#include "vsim_types.h"
#include <array>

namespace vsim {

class MotorModel {
public:
    MotorModel() = default;

    void setParams(const MotorParams& p) { params_ = p; }
    const MotorParams& params() const { return params_; }

    // duty[4] in [0,1]; dt in seconds. Updates internal omegas and
    // returns the aggregated body-frame wrench.
    void update(const std::array<float, 4>& duty, float dt,
                Vec3* force_b, Vec3* torque_b);

    void reset();

    const std::array<float, 4>& omegas() const { return omega_; }

private:
    MotorParams params_;
    std::array<float, 4> omega_ = {0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace vsim

#endif  // VSIM_MOTOR_MODEL_H
