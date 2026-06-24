# SITL run 2026-06-21 02:13 — estimate vs. ground truth (first dual-log archive)

> **Provenance — SIMULATOR (SITL), and the first archive with BOTH halves of the
> seam.** This run was captured *after* the `mag_fusion` producer fix
> (commit `f14e53e`). It pairs the **FC's own telemetry** (`export-*.bin`, the
> firmware's estimated/sensed view) with the **physics ground truth**
> (`gt-*.bin`, the true pose written by the sim — new in commit `5dad3fe`). The
> two share one clock, so estimate-vs-truth is a direct overlay, not a guess.
> Sensor pathologies of real hardware don't apply; what carries over is the FC
> control + estimation code, so a discrepancy here is a real firmware behaviour.

## Headers

| field            | FC export                          | physics ground truth                |
|------------------|------------------------------------|-------------------------------------|
| file             | `export-20260621-021352.bin` (502 KB) | `gt-2026-06-21_02-13-48.bin` (640 KB) |
| container        | VREC v1 / proto v1                 | `VGT1` v1                           |
| start wall clock | 1781988234374 ms (02:13:54)        | 1781988228690 ms (02:13:48)         |
| records / frames | 9,656                              | 5,164 @ 62.5 Hz                     |
| time span        | 76.9 s                             | 82.6 s                              |
| CRC errors       | **0**                              | n/a (fixed-record)                  |

**Same run, provably:** cross-correlating the motor command common to both
(`MotorTelemetry` vs `gt motor_duty`) locks at **lag +0.04 s, correlation
1.000**. Every estimate-vs-truth number below is therefore a real aligned
comparison. Decode: `../parse_log.py` (FC) and `../parse_gt.py` (truth).

## FC message inventory

| message        | Hz   | message       | Hz   |
|----------------|------|---------------|------|
| ImuCompressed  | 27.2 | VerticalState | 11.1 |
| ControlTrace   | 20.8 | Heartbeat     | 3.3  |
| MotorTelemetry | 20.8 | FlightMode    | 3.3  |
| AttitudeEuler  | 11.1 | SystemHealth  | 3.3  |
| RcChannels     | 11.1 | ImuRaw        | 1.7  |
| Baro           | 11.1 | EstPerf       | 0.5  |

`FlightMode = ANGLE (0)` for 100 %. Loop timing rock-steady (`outer_dt` 4.000 ms,
`inner_dt` 1.000 ms, **0 µs jitter**). Motors balanced (4-rotor mean equal to
3 d.p., peak duty 0.66). A manual free flight: climbed to 120 m and drifted
~100 m horizontally (max ground speed 9.4 m/s), with a deliberate yaw rotation
to −134°.

## Headline

**With the `mag_fusion` fix in place, the estimator now matches truth on yaw and
altitude; the one remaining gap is roll/pitch under sustained acceleration.**

- **Yaw — FIXED.** Estimate tracks the full true heading swing (−134°), error
  **RMS 2.9°** (median 0.7°). Before the fix this run-class showed RMS 121° with
  the estimate pinned near 0 while the craft rotated away.
- **Altitude — excellent.** Fused vs truth **RMS 0.12 m** across a 0→120 m
  profile.
- **Roll/pitch — the open issue.** Accurate when calm, but during free-flight
  drift the FC reads ~level while the craft is genuinely tilted: with sticks
  centered (89 % of the flight) the estimated tilt median is **0.18°** but the
  **true** tilt median is **2.3°**, p90 **14.6°**, max **23.2°** — and the gap
  grows with horizontal speed (corr **+0.46**). This is the accelerometer's
  gravity-vs-acceleration ambiguity, not a bug, and not fixable without a
  velocity reference (GPS / optical-flow).

## Previously-detected faults — status now

| # | fault (where found) | status | evidence this run |
|---|---------------------|--------|-------------------|
| 1 | **Yaw heading unobservable** — estimate doesn't track true heading (2026-06-17 "yaw untrustworthy"; 00:51 HUD-vs-sim; 01:12 RMS 121°) | ✅ **RESOLVED** | root cause was a SITL data bug — producers never packed `mag_fusion[3]`, so the FC read `{temp,0,0}` as its heading reference. Fixed in `f14e53e`. Yaw RMS **121° → 2.9°**; tracks the −134° swing. |
| 2 | **Roll/pitch tilt under-report in accelerated flight** (00:51 −0.85° est vs −8° truth) | ⚠️ **OPEN — inherent** | confirmed cleanly here: est ~level while truth tilted (up to 23°), ∝ horizontal speed. Magnitude-gating proven ineffective (no discrimination); needs GPS/optical-flow. Normal for GPS-less angle mode. |
| 3 | **EKF gyro-bias tracking** (hypothesised cause of yaw drift) | ✅ **disproved** | bumping `EKF_Q_BG` 1e-8→1e-6 changed the yaw residual by 0.2° (127.5→127.3). Reverted — was not the cause (fault #1 was). |
| 4 | **Vertical estimator** (2026-06-20 healthy) | ✅ **still healthy** | fused−baro RMS 0.12 m here. |
| 5 | **Roll/pitch rate-loop under-gain → tumbles** (2026-06-21 001850 triplet) | ↪️ **separate / benign here** | gentle flight, no tumble; rate-loop tracking still soft (roll corr 0.58, pitch 0.28) but not exercised. A control-tuning item, independent of the estimator. |

## Inventory of this archive

- [`session-analysis.md`](session-analysis.md) — the run, timeline, what it did
- [`estimator-analysis.md`](estimator-analysis.md) — **the headline**: estimate vs truth on all four states; the yaw fix and the roll/pitch limitation
- [`recommendations.md`](recommendations.md) — fault statuses + next steps
- [`plots/`](plots/) — 4 estimate-vs-truth figures (`make_plots.py` in this dir)
- `export-20260621-021352.bin` / `gt-2026-06-21_02-13-48.bin` — the paired raw logs
- `make_plots.py` — session-specific plotting (dual-log; NOT the generic one)

## Raw findings log

1. FC 76.9 s / 0 CRC; GT 82.6 s @ 62.5 Hz; motor-xcorr **lag 0.04 s, corr 1.000**
   → same run, exactly aligned.
2. `FlightMode = ANGLE` 100 %; `outer/inner dt` 4.000/1.000 ms, 0 µs jitter.
3. **Yaw** est-vs-truth **RMS 2.86°** (median 0.74°, max 12.2°); truth swung
   −134°…0°, estimate tracked it. (Was 121° pre-fix.)
4. **Altitude** est-vs-truth **RMS 0.12 m** (max 0.45 m) over 0→120 m.
5. **Roll/pitch**: calm-hold sub-degree; but stick-centered (89 % of flight) the
   estimate reads tilt median **0.18°** vs **truth 2.33°** (p90 14.6°, max 23.2°).
6. `corr(truth_tilt − est_tilt, horizontal_speed) = +0.46` — the error is
   acceleration-driven (specific force ≠ gravity).
7. Accel **magnitude gating can't help**: `|‖a‖−g|` is 0.54 in failure episodes
   vs 0.56 in genuinely-level ones — no discriminating power.
8. Rate loop (gentle flight): roll corr 0.58, pitch 0.28, yaw 0.98; roll/pitch
   `_out` peaks 0.02 / 0.19 — soft but not exercised here.
9. Motors balanced; peak duty 0.66; no saturation.
