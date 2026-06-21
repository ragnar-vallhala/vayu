# Recommendations

## Resolved / validated (no further action)

1. **Yaw / heading** — ✅ exact (RMS 0.32°) under aggressive flight. The
   `mag_fusion` producer fix holds.
2. **Altitude** — ✅ exact (0.04 m).
3. **Roll/pitch tilt under-report** — ✅ mitigated by `EKF_R_ACC_DIR = 2.5e-2`:
   stick-centered under-report 2.15° → 0.15°, and the craft flies genuinely more
   level.
4. **Long-flight drift risk** of the accel-trust change — ✅ cleared by the
   176 s headless validation (early 0.48° ≈ final 0.49°).

## The decision in front of us

**Commit `EKF_R_ACC_DIR = 2.5e-2`?** The evidence says yes:
- Big win in the common regimes (level/cruise sub-degree; under-report ~gone).
- No drift penalty over 2+ minutes.
- Yaw/altitude unaffected.

**Caveat before relying on a vehicle:** this is a real firmware change (it
affects hardware, not just SITL). One **real-hardware** flight should confirm the
SITL result before trusting it on an airframe — the sim's IMU noise/bias model is
representative but not identical to the BMX160 + vibration environment.

**Suggested commit message note:** record the SITL validation (this archive + the
176 s drift check) and the residual-during-sustained-accel caveat, so the
rationale travels with the change.

## Open / out of scope (by decision)

5. **Sustained-acceleration tilt residual** (~8° during a multi-second
   coordinated accel; p90 9° overall on aggressive flights) — the irreducible
   IMU-only floor. Removing it needs a **velocity reference** (GPS or
   optical-flow). A drag-model velocity estimate was explicitly **declined**
   (adds a calibration/modeling burden). So this residual is accepted: fine for
   manual/acro flight, and the bar for hands-off leveling / position hold would
   require the velocity sensor regardless.

## Capture notes for next time

- **Log the active estimator config** (e.g. `EKF_R_ACC_DIR`, filter type) at arm
  so each run is self-describing — we currently track it out-of-band.
- For the **real-HW** check, fly a profile with a sustained coordinated
  acceleration (a fast, held lateral translation) — that's the regime that
  separates this estimator from a velocity-aided one.
- Keep using the dual-log (`export` + `gt-*.bin`) overlay as the regression
  baseline; this archive is the post-accel-trust reference.
