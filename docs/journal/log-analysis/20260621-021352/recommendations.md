# Recommendations

Organised as the status of every previously-detected fault plus the one open
item and its options.

## Resolved this cycle (no action)

1. **Yaw / heading observability** — ✅ fixed. The producers now pack
   `mag_fusion[3]` (commit `f14e53e`); verified yaw RMS 121° → 2.9° against
   ground truth. Nothing further.
2. **Altitude estimator** — ✅ healthy (RMS 0.12 m). No action.
3. **Ground-truth recording gap** — ✅ closed. `gt-*.bin` now logged
   (commit `5dad3fe`), making estimate-vs-truth a one-command overlay. Keep
   using it for every estimator change.
4. **`EKF_Q_BG` gyro-bias hypothesis** — ✅ disproved and reverted. Don't
   revisit; it was not the cause.

## Open item — roll/pitch tilt under sustained acceleration

**Problem:** in free flight the FC reads ~level while the craft is tilted (up to
23°), error ∝ horizontal speed. Root cause is the accelerometer's
gravity-vs-acceleration ambiguity; it is *not* a bug and *not* gateable.

**Decide by intent:**

- **Manual acro / stabilize only** → accept it. This is normal GPS-less
  behaviour; the median 2.3° is small and the pilot corrects drift. No change.
- **Hands-off leveling / position or velocity hold / autonomy** → a velocity
  reference is required. Options, in order of fidelity vs. cost:
  1. **Optical-flow + downward rangefinder** — horizontal velocity for
     low-altitude/indoor; the lightest real fix.
  2. **GPS** (velocity fusion) — the outdoor standard.
  3. **VIO / camera** — GPS-denied, heavier compute.
  4. **Drag-model velocity** (no new sensor) — estimate horizontal velocity from
     the IMU via a calibrated multirotor drag coefficient (ArduPilot EKF3 drag
     fusion / Betaflight). Approximate (degrades in wind / hard maneuvers) but
     hardware-free; a reasonable interim mitigation.

**Before committing to hardware — quantify the ceiling in SITL.** We have true
velocity in the sim, so prototype a velocity-compensated accel update (subtract
`a_kinematic`/`ω×v` from specific force) and re-fly this exact profile. That
yields the best-case number — "with perfect velocity aiding, tilt error drops
from ~14° p90 to Y°" — which tells you whether GPS/flow is worth it on the
airframe. (This is an experiment, not a shippable change, since real HW lacks the
true velocity.)

## Carry-over (separate from the estimator)

5. **Roll/pitch rate-loop softness** — rate tracking was soft this run (roll
   corr 0.58, pitch 0.28) but the flight was benign so it wasn't stressed; the
   2026-06-21 001850 triplet showed it can tumble under aggression. This is a
   control-tuning item (autotune / System-ID) independent of the estimator —
   track it separately; don't conflate with the tilt finding above.

## Capture notes for next time

- **Log the active PID/estimator config** at arm (one telemetry line) so a run
  is self-describing — we still infer gains from data.
- **Re-fly this profile after any estimator change** and compare against this
  archive as the post-`mag_fusion` baseline (yaw 2.9°, alt 0.12 m, free-flight
  tilt p90 14.6°).
