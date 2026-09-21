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
#include "control/throttle_curve.h"

#include "maths/maths_interface.h"

/** @noreq Trivial clamp helper. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** @noreq Expo derived from hover; see throttle_curve.h. */
float throttle_curve_expo(float hover_duty) {
  return clampf(-(hover_duty - 0.5f) / 0.375f, -0.5f, 1.0f);
}

/** @noreq Collective stick shaping; see throttle_curve.h. */
float throttle_curve(float stick, float hover_duty) {
  stick = clampf(stick, 0.0f, 1.0f);

  /* A nonsensical hover constant must not produce surprising stick behaviour —
   * fall back to the raw linear map the airframe had before. */
  if (!(hover_duty > 0.0f) || !(hover_duty < 1.0f)) {
    return stick;
  }
  const float expo = throttle_curve_expo(hover_duty);

  /* Expo about mid-stick: x in [-1,1], y = (1-e)x + e x^3. Cubic keeps the
   * endpoints and the centre fixed while flattening the slope near centre. */
  float x = 2.0f * (stick - 0.5f);
  float y = (1.0f - expo) * x + expo * x * x * x;
  float s = 0.5f + 0.5f * y;

  /* Stick -> thrust fraction, hover pinned at mid-stick. Two straight segments:
   * the lower half spans 0..hover_thrust, the upper half hover_thrust..1. */
  const float hover_thrust = hover_duty * hover_duty;
  float thrust =
      (s <= 0.5f) ? (s * 2.0f * hover_thrust)
                  : (hover_thrust + (s - 0.5f) * 2.0f * (1.0f - hover_thrust));

  /* Thrust ~ duty^2, so invert to get the duty the mixer wants. */
  return clampf(m_sqrt(clampf(thrust, 0.0f, 1.0f)), 0.0f, 1.0f);
}
