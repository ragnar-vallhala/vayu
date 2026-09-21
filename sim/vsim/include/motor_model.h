/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

  void setParams(const MotorParams &p) { params_ = p; }
  const MotorParams &params() const { return params_; }

  // duty[4] in [0,1]; dt in seconds. Updates internal omegas and
  // returns the aggregated body-frame wrench.
  void update(const std::array<float, 4> &duty, float dt, Vec3 *force_b,
              Vec3 *torque_b);

  void reset();

  const std::array<float, 4> &omegas() const { return omega_; }

private:
  MotorParams params_;
  std::array<float, 4> omega_ = {0.0f, 0.0f, 0.0f, 0.0f};

  // Transport-delay line: per-rotor circular buffer of past duty commands.
  // update() is called at the PHYSICS substep rate (8 kHz in the daemon), so
  // the tap is round(delay/dt_sub); 2048 covers 256 ms @8 kHz (2 s @1 kHz).
  static constexpr int kDelayBuf = 2048;
  std::array<std::array<float, kDelayBuf>, 4> duty_hist_{};
  int hist_pos_ = 0;

  std::array<bool, 4> stalled_ = {false, false, false, false};
};

} // namespace vsim

#endif // VSIM_MOTOR_MODEL_H
