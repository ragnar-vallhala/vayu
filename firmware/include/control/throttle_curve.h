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

/* Expo applied about mid-stick: 0 = none, ->1 = increasingly flat near hover. */
#ifndef THROTTLE_EXPO
#define THROTTLE_EXPO 0.30f
#endif

/* Map collective stick [0,1] -> commanded collective [0,1].
 *   stick       pilot input, clamped
 *   hover_duty  collective that hovers this airframe (0..1); <=0 or >=1 makes
 *               this a pass-through, so a bad constant degrades to the old
 *               linear behaviour rather than to something surprising
 *   expo        0..1, flattening about centre */
float throttle_curve(float stick, float hover_duty, float expo);

#endif /* VAYU_THROTTLE_CURVE_H */
