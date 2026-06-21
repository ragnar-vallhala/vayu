# Session analysis — 2026-06-21 05:31 aggressive verification flight

A short (31.7 s) but **deliberately aggressive** in-app SITL flight, flown to
verify the lowered accelerometer trust (`EKF_R_ACC_DIR = 2.5e-2`) under real
maneuvering — large attitudes, fast translation, and a hard flick — rather than
the gentle profile of earlier runs.

## What the run did

- **Mode** ANGLE (stabilize) throughout.
- **Attitude envelope** truth reached **roll +60°, pitch −50°** — far past the
  ~20° of the baseline run.
- **Speed** up to **14.5 m/s** ground speed (vs ~4–9 m/s before).
- **Vertical** a climb to 15 m and back; manual throttle.
- **Heading** a yaw rotation to **−108°**, held.

## Timeline (aligned grid, t from first common sample)

```
 0–8 s    on ground / spin-up, level
 8–11 s   climb begins; gentle
11–15 s   SUSTAINED nose-down pitch maneuver (truth → −33°, forward accel) ← the residual shows here
15–24 s   climb to 15 m, descents, drifting; yaw rotates to −108° near t≈24 s
24–26 s   HARD flick: roll +60° / pitch −50° transient, then recover
26–32 s   settle / land
```

## How to read it against the baseline

This run is the **after** to [`20260621-021352`](../20260621-021352/)'s
**before** on the accel-trust axis (both already have the `mag_fusion` yaw fix):

| | 021352 (R=2.5e-3) | **053142 (R=2.5e-2, this)** |
|--------|------------------:|----------------------------:|
| flight character | moderate free flight | aggressive (60° roll, 14.5 m/s) |
| yaw est-vs-truth RMS | 2.9° | **0.32°** |
| altitude RMS | 0.12 m | **0.04 m** |
| stick-centered tilt under-report | 2.15° | **0.15°** |
| true tilt when "level" (median) | 2.33° | **0.32°** |

Two caveats when comparing:
1. The flights aren't identical maneuvers, so the headline is the **structural**
   change (the under-report collapsing), not a maneuver-matched delta.
2. This run is **short** — it is *not* a long-duration drift test. The concern
   that a lower accel trust could let attitude drift over time was cleared
   separately by a 176 s headless validation (early-settle 0.48° ≈
   final-hover 0.49°). See [estimator-analysis.md](estimator-analysis.md).

## Alignment & integrity

Motor-command cross-correlation between FC `MotorTelemetry` and gt `motor_duty`
peaks at **lag 0.04 s, corr 1.000** — the two logs are the same run, sample-
aligned, so every estimate-vs-truth number is a direct comparison. 0 CRC errors;
loop timing rock-steady (4.000 / 1.000 ms, 0 µs jitter).
