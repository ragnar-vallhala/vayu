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
#include "motor_model.h"

#include <algorithm>
#include <cmath>

namespace vsim {

void MotorModel::reset() {
  omega_ = {0.0f, 0.0f, 0.0f, 0.0f};
  for (auto &h : duty_hist_)
    h.fill(0.0f);
  hist_pos_ = 0;
  stalled_ = {false, false, false, false};
}

void MotorModel::update(const std::array<float, 4> &duty, float dt,
                        Vec3 *force_b, Vec3 *torque_b) {
  Vec3 F(0.0f, 0.0f, 0.0f);
  Vec3 T(0.0f, 0.0f, 0.0f);

  // Push this step's duty into the per-rotor delay line. The tap is read
  // `delay` seconds back; with delay==0 it reads the just-written value, so
  // the whole block is a no-op for the default (ideal) actuator.
  int delay_n = 0;
  if (params_.transport_delay > 0.0f && dt > 1e-9f) {
    delay_n = static_cast<int>(std::lround(params_.transport_delay / dt));
    if (delay_n >= kDelayBuf)
      delay_n = kDelayBuf - 1;
  }
  for (int i = 0; i < 4; ++i)
    duty_hist_[i][hist_pos_] = std::clamp(duty[i], 0.0f, 1.0f);

  for (int i = 0; i < 4; ++i) {
    int tap = hist_pos_ - delay_n;
    if (tap < 0)
      tap += kDelayBuf;
    // The buffer is zero-initialised (reset()), so before it has filled the
    // delay window the tap reads 0 = motor off — the correct startup value.
    float d = duty_hist_[i][tap];

    // Idle stall: a rotor driven below stall_duty produces no thrust and
    // must re-spin from where it is with the slower respin_tau. Self-selects
    // the saturating axis (the one whose mixer floors it), matching the real
    // pitch-cycles / roll-stable asymmetry.
    bool restalled = false;
    if (params_.stall_duty > 0.0f) {
      if (d < params_.stall_duty) {
        stalled_[i] = true;
        d = 0.0f;
      } else if (stalled_[i]) {
        restalled = true;
      } // re-spinning
    }

    float target = d * params_.max_omega[i];

    // Asymmetric first-order tracking; pick tau by direction.
    // The alpha formula is a backward-Euler discretization.
    // Per-rotor spin-up constant; spin-down is 2x.
    const float tau_up = restalled ? params_.respin_tau : params_.tau[i];
    float tau = (target > omega_[i]) ? tau_up : (tau_up * 2.0f);
    if (tau < 1e-6f)
      tau = 1e-6f;
    float alpha = dt / (tau + dt);
    omega_[i] += (target - omega_[i]) * alpha;
    // Clear the stall once the rotor has re-spun back near its command.
    if (restalled && omega_[i] >= 0.95f * target)
      stalled_[i] = false;

    float w2 = omega_[i] * omega_[i];
    float thrust = params_.k_thrust[i] * w2;

    // Thrust along the per-rotor axis (unit, body frame). Default
    // axis is body -Z (lift = up in NED). Normalize defensively in
    // case the editor pushed a non-unit vector.
    Vec3 axis = params_.axis_b[i];
    float an = std::sqrt(axis.x() * axis.x() + axis.y() * axis.y() +
                         axis.z() * axis.z());
    if (an > 1e-6f)
      axis = axis / an;
    Vec3 F_i = axis * thrust;
    F += F_i;

    // Moment from rotor offset: r x F.
    Vec3 r = params_.pos_b[i];
    T += Vec3::crossProduct(r, F_i);

    // Reaction torque about the spin (= thrust) axis, signed by spin
    // direction. The -axis keeps parity with the previous body-Z
    // convention for the default -Z axis (reaction was +Z * spin).
    float tau_react =
        params_.k_moment[i] * w2 * static_cast<float>(params_.spin[i]);
    T -= axis * tau_react;
  }

  // Advance the delay line.
  hist_pos_ = (hist_pos_ + 1) % kDelayBuf;

  if (force_b)
    *force_b = F;
  if (torque_b)
    *torque_b = T;
}

} // namespace vsim
