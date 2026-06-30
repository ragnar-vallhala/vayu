# SITL run 2026-06-21 05:31 — lowered accel trust (EKF_R_ACC_DIR=2.5e-2), aggressive flight

> **Provenance — SIMULATOR (SITL), dual-log (FC estimate + physics ground
> truth).** Captured *after* two firmware/seam changes vs the
> [`20260621-021352`](../20260621-021352/) baseline: the `mag_fusion` producer
> fix (yaw) and **`EKF_R_ACC_DIR` raised 2.5e-3 → 2.5e-2** (the accelerometer is
> trusted less, so the gyro carries attitude through accelerations — the
> GPS-less PX4/ArduPilot approach). This run is the in-app verification of that
> accel-trust change under deliberately **aggressive** flight.

## Headers

| field            | FC export                          | physics ground truth                |
|------------------|------------------------------------|-------------------------------------|
| file             | `export-20260621-053142.bin` (205 KB) | `gt-2026-06-21_05-31-38.bin` (287 KB) |
| container        | VREC v1 / proto v1                 | `VGT1` v1                           |
| start wall clock | 1782000104050 ms (05:31:44)        | 1781999498690-ish (05:31:38)        |
| records / frames | 3,977                              | 2,317 @ 62.6 Hz                     |
| time span        | 31.7 s                             | 37.0 s                              |
| CRC errors       | **0**                              | n/a                                 |

**Same run, provably:** motor-command cross-correlation locks at **lag +0.04 s,
correlation 1.000**. `FlightMode = ANGLE` 100 %; loop timing 4.000 / 1.000 ms,
**0 µs jitter**. A short, **aggressive** free flight: truth attitude reached
**roll +60°, pitch −50°**, a yaw rotation to −108°, climb to 15 m, and **14.5 m/s**
ground speed.

## Headline

**The lowered accel trust holds up: yaw and altitude are exact, and roll/pitch
now tracks the true tilt closely — the under-report the FC used to have when it
*thought* it was level is essentially gone.**

| axis | this run (R=2.5e-2) | baseline `021352` (R=2.5e-3) |
|------|---------------------|------------------------------|
| **yaw** est-vs-truth RMS | **0.32°** (max 2.5°, tracks −108° swing) | 2.9° |
| **altitude** RMS | **0.04 m** | 0.12 m |
| **roll/pitch tilt** median / p90 | **0.52° / 9.1°** | 2.95° / — |
| **stick-centered tilt under-report** (est vs true when "level") | **0.15°** (est 0.17° vs true 0.32°) | **2.15°** (est 0.18° vs true 2.33°) |

The decisive number is the last row: with sticks centered (92 % of the flight),
the FC's estimated tilt and the *true* tilt now agree to **0.15°** — versus a
**2.15°** gap at the old accel trust. The craft is also genuinely more level
(true tilt median 0.32° vs 2.33°), because an accurate estimate lets the
controller actually correct the tilt instead of holding a hidden one.

The residual is confined to **sustained acceleration**: the clearest example is
the t≈12–15 s pitch maneuver where the craft holds −33° while accelerating
forward and the estimate reads −25° (an ~8° under-report) — the irreducible
accelerometer gravity-vs-acceleration ambiguity that only velocity aiding could
remove. `corr(truth_tilt − est_tilt, horizontal_speed) = +0.67` confirms the
residual is acceleration-driven. Brief, fast flicks (the t≈25 s roll +60° /
pitch −50°) track well because the gyro carries them.

## Previously-detected faults — status now

| # | fault | status |
|---|-------|--------|
| 1 | Yaw heading unobservable (`mag_fusion` seam bug) | ✅ **resolved & holding** — yaw RMS **0.32°** here, tracks the −108° swing |
| 2 | Roll/pitch tilt under-report in accelerated flight | ✅ **largely mitigated** by `EKF_R_ACC_DIR=2.5e-2` — stick-centered under-report **2.15° → 0.15°**; residual only during *sustained* accel (the IMU-only floor) |
| 3 | Long-flight drift from the lower accel trust (the change's risk) | ✅ **cleared** in the prior 176 s headless validation (early-settle 0.48° ≈ final-hover 0.49°); this run is short (31.7 s) and not a drift test |
| 4 | Altitude / vertical estimator | ✅ **healthy** — RMS 0.04 m |

## Inventory of this archive

- [`session-analysis.md`](session-analysis.md) — the run, timeline, comparison to the baseline
- [`estimator-analysis.md`](estimator-analysis.md) — **the headline**: estimate vs truth; the accel-trust effect and the residual
- [`recommendations.md`](recommendations.md) — fault statuses + the commit/real-HW decision
- [`plots/`](plots/) — 3 estimate-vs-truth figures (`make_plots.py` in this dir)
- `export-20260621-053142.bin` / `gt-2026-06-21_05-31-38.bin` — the paired raw logs

## Raw findings log

1. FC 31.7 s / 0 CRC; GT 37.0 s @ 62.6 Hz; motor-xcorr **lag 0.04 s, corr 1.000**.
2. Aggressive: truth roll +60°, pitch −50°, yaw −108°, alt 0→15 m, 14.5 m/s.
3. **Yaw** RMS **0.32°** (median 0.13°, max 2.5°) — tracks the full −108° swing.
4. **Altitude** RMS **0.04 m** (max 0.15 m).
5. **Roll/pitch tilt** median **0.52°**, p90 9.1°, max 15.7°.
6. **Stick-centered (92 %)**: est tilt median **0.17°** vs true **0.32°** — the
   under-report dropped from 2.15° (021352) to **0.15°**.
7. Residual is acceleration-driven: `corr(tilt_err, hspeed) = +0.67`; the
   sustained t≈12–15 s pitch hold under-reports ~8°, fast flicks track well.
8. Loop 4.000/1.000 ms 0 µs jitter; motors balanced (mean 0.19), roll-pair peak
   0.69 vs pitch-pair 0.40 (the asymmetric aggressive inputs).
