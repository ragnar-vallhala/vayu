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
#ifndef VAYU_THROTTLE_CURVE_H
#define VAYU_THROTTLE_CURVE_H

/*
 * Collective stick shaping — hover at mid-stick, in THRUST space.
 *
 * The raw map was `collective = stick`, which on a high-thrust airframe is
 * unflyable by hand. On the 5in racer (TWR ~6.8, hover collective 0.38):
 *
 *     stick 0.25 -> 0.42x hover thrust  = -0.6 g, it drops
 *     stick 0.38 -> hover
 *     stick 0.50 -> 1.69x hover thrust  = +0.7 g, mid-stick already climbs
 *     stick 1.00 -> 6.8x                = +5.8 g
 *
 * Two problems compound. Hover sits at 38% of travel, so most of the stick is
 * climb and the whole descend range is squeezed below it; and thrust goes as
 * duty^2, so the response accelerates as you push up. The usable gentle band was
 * about 15% of stick travel.
 *
 * This maps stick -> THRUST FRACTION with hover at mid-stick, then converts to
 * duty. Centre is hover, authority is symmetric either side, and equal stick
 * increments give equal thrust increments instead of accelerating ones:
 *
 *     stick 0.0 -> thrust 0
 *     stick 0.5 -> thrust = hover_duty^2      (hover)
 *     stick 1.0 -> thrust 1
 *     collective = sqrt(thrust)
 *
 * `expo` then flattens the response around centre, trading resolution at the
 * extremes for fine control near hover. 0 = pure piecewise-linear-in-thrust.
 *
 * hover_duty is passed in rather than compiled in, so there is ONE airframe
 * hover constant in the tree (HEIGHT_HOVER_GUESS) rather than two that can
 * silently disagree — and so this stays unit-testable across airframes.
 *
 * Endpoints are preserved exactly: stick 0 gives 0 (motors off / disarm still
 * work) and stick 1 gives full. Only the shape between them changes.
 */

/* Expo is DERIVED from hover, not hand-tuned — ArduPilot's construction
 * (Mode::get_pilot_desired_throttle, mode.cpp):
 *
 *     expo = constrain(-(hover - 0.5) / 0.375, -0.5, 1.0)
 *
 * The lower the hover, the more of the stick sits above it and the steeper the
 * upper half becomes, so the more flattening is wanted around centre. A craft
 * that hovers at exactly mid-stick needs none, and one that hovers HIGH gets
 * negative expo (steeper near centre, finer at the extremes). For this airframe
 * (hover 0.38) it yields 0.32 — which is what had been picked by feel, so the
 * formula reproduces the hand-tuned value while also tracking hover when the
 * estimator moves it. */
float throttle_curve_expo(float hover_duty);

/* Map collective stick [0,1] -> commanded collective [0,1].
 *   stick       pilot input, clamped
 *   hover_duty  collective that hovers this airframe RIGHT NOW (0..1) — the
 *               in-flight estimate when there is one, else the airframe
 *               constant. <=0 or >=1 makes this a pass-through, so a bad value
 *               degrades to the old linear map rather than to something
 *               surprising. */
float throttle_curve(float stick, float hover_duty);

#endif /* VAYU_THROTTLE_CURVE_H */
