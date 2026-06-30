# Session analysis — 2026-06-21 02:13 free flight (post-fix verification)

A single manual SITL flight flown specifically to verify the `mag_fusion` seam
fix and to re-check the estimator against ground truth across a realistic
profile. Unlike the earlier rig tests (translation pinned), this is **free
flight** — the craft climbs, drifts, and yaws — which is exactly the regime that
exposes acceleration-dependent estimator behaviour.

## What the run did

- **Duration** 76.9 s (FC) / 82.6 s (GT); the GT log opens ~5 s earlier because
  the sim starts before the GCS Export is armed.
- **Mode** ANGLE (stabilize) throughout — roll/pitch sticks command attitude,
  centered = "hold level"; yaw was deliberately commanded.
- **Vertical** a 0 → 80 m climb, a dip to ~52 m, a second climb to **120 m**,
  then descent to ground — a smooth ±15 m/s profile (manual throttle; no
  altitude hold in this build).
- **Heading** a commanded yaw rotation to **−134°**, then held — the key test
  for the mag fix.
- **Horizontal** drifted to ~(90 N, 44 E) m ≈ **100 m** from origin, peaking at
  **9.4 m/s** ground speed — the craft was not holding position (it can't; no
  GPS/flow), and this drift is what drives the roll/pitch finding.

## Timeline (t relative to the common aligned grid)

```
 0–10 s   on ground / spin-up, level
10–20 s   climb begins; first roll/pitch inputs; YAW commanded → −127° at ~18 s
20–45 s   climb to 80 m, dip; drifting; sustained small tilts
45–70 s   second climb to 120 m; largest pitch excursion (truth +20° @ ~66 s)
70–77 s   descent to ground
```

## How to read this run

Two things make the numbers honest:

1. **Ground truth is recorded**, so "error" is `estimate − physics`, not
   `estimate − another estimate`. This is the first archive where that's true.
2. **Alignment is exact** (motor-command cross-correlation, corr 1.000), so
   per-sample comparisons are valid even during maneuvers.

One caveat carried into the per-axis numbers: the FC telemetry is ~11–21 Hz
while truth is 62.5 Hz, so during fast transients a "latest-sample" comparison
adds a few degrees of *sampling skew* (not estimator error). The estimator
analysis separates the sustained, acceleration-correlated error (real) from this
transient skew (measurement artifact). See
[estimator-analysis.md](estimator-analysis.md).

## Cross-run context

| metric | 2026-06-20 | 01:12 (pre-fix) | **02:13 (this, post-fix)** |
|--------|-----------:|----------------:|---------------------------:|
| yaw est-vs-truth RMS | — (no GT) | 121° | **2.9°** |
| altitude est-vs-truth RMS | 0.25 m | 0.4 m | **0.12 m** |
| roll/pitch (free-flight, stick-centered truth tilt p90) | — | ~14° | **14.6°** |
| ground truth logged? | no | yes | yes |

The yaw collapse from 121° → 2.9° is the headline; the roll/pitch free-flight
tilt is unchanged (it's physics, not the mag bug) and is the remaining work.
