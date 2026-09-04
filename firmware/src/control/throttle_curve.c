#include "control/throttle_curve.h"

#include "maths/maths_interface.h"

/** @noreq Trivial clamp helper. */
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** @noreq Collective stick shaping; see throttle_curve.h. */
float throttle_curve(float stick, float hover_duty, float expo) {
  stick = clampf(stick, 0.0f, 1.0f);

  /* A nonsensical hover constant must not produce surprising stick behaviour —
   * fall back to the raw linear map the airframe had before. */
  if (!(hover_duty > 0.0f) || !(hover_duty < 1.0f)) {
    return stick;
  }
  expo = clampf(expo, 0.0f, 1.0f);

  /* Expo about mid-stick: x in [-1,1], y = (1-e)x + e x^3. Cubic keeps the
   * endpoints and the centre fixed while flattening the slope near centre. */
  float x = 2.0f * (stick - 0.5f);
  float y = (1.0f - expo) * x + expo * x * x * x;
  float s = 0.5f + 0.5f * y;

  /* Stick -> thrust fraction, hover pinned at mid-stick. Two straight segments:
   * the lower half spans 0..hover_thrust, the upper half hover_thrust..1. */
  const float hover_thrust = hover_duty * hover_duty;
  float thrust = (s <= 0.5f)
                     ? (s * 2.0f * hover_thrust)
                     : (hover_thrust + (s - 0.5f) * 2.0f * (1.0f - hover_thrust));

  /* Thrust ~ duty^2, so invert to get the duty the mixer wants. */
  return clampf(m_sqrt(clampf(thrust, 0.0f, 1.0f)), 0.0f, 1.0f);
}
