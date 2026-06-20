# SITL triplet 2026-06-21 — autotune-validation flights: roll rate gain decides tumble vs. clean

> **Provenance — these are SIMULATOR (SITL) captures, not real hardware.**
> Three back-to-back firmware-in-the-loop sessions (real FC control logic,
> simulated physics + sensors), recorded during the 2026-06-21 autotune /
> System-ID work. Bench-rig sensor pathologies (mag uncalibrated, accel scale,
> kernel FIFO drops) do **not** apply — the sim feeds clean sensors and reports
> `cpu_load = 0`. What carries over is the **control law**, which is the real
> firmware: a control-tuning result seen here is a real firmware behaviour,
> reproduced deterministically.

This archive holds **three** logs analysed together. They are consecutive
manual ANGLE-mode flights flown with aggressive roll/pitch stick. Their only
material difference is the **roll/pitch rate gain** — and that difference
decides whether the aircraft holds attitude or tumbles inverted.

## Headers

| field            | `001607`            | `001716`            | `001850`            |
|------------------|---------------------|---------------------|---------------------|
| file             | `export-20260621-001607.bin` | `export-20260621-001716.bin` | `export-20260621-001850.bin` |
| size             | 235,673 bytes       | 452,872 bytes       | 381,236 bytes       |
| start wall clock | 1781981172345 ms    | 1781981241379 ms    | 1781981332356 ms    |
| (local)          | 2026-06-21 00:16:12 | 2026-06-21 00:17:21 | 2026-06-21 00:18:52 |
| format / proto   | VREC v1 / proto v1  | VREC v1 / proto v1  | VREC v1 / proto v1  |
| records / frames | 4,709               | 8,836               | 7,453               |
| time span        | **37.5 s**          | **70.4 s**          | **59.3 s**          |
| CRC errors       | **0**               | **0**               | **0**               |

All three decode with zero CRC errors — clean captures, healthy decoder.

## Message inventory (identical streams in all three)

| message        | rate Hz | message       | rate Hz |
|----------------|---------|---------------|---------|
| ImuCompressed  | 27.2    | VerticalState | 11.1    |
| ControlTrace   | 20.8    | Heartbeat     | 3.3     |
| MotorTelemetry | 20.8    | FlightMode    | 3.3     |
| AttitudeEuler  | 11.1    | SystemHealth  | 3.3     |
| RcChannels     | 11.1    | ImuRaw        | 1.7     |
| Baro           | 11.1    | EstPerf       | 0.5     |

No `PerfGlobal` / `PerfFifo` / `Mag` streams in this build, so the kernel-load
and magnetometer figures from the 2026-06-17 hardware archive have no analogue.
`FlightMode = 0 (ANGLE / stabilize)` for 100 % of every run.

## Headline

**Same airframe, same flight task, same code — three runs, and the roll/pitch
rate gain is the only thing that changes the outcome.**

- **`001607` — tumble.** Roll rate loop is near-inert (effective `Kp ≈ 5.4e-5`,
  `corr(rate_sp, rate_curr) = +0.08`). Under aggressive roll stick the loop
  cannot hold; roll crosses 90° at **t ≈ 20.2 s** and the craft spends **38 %**
  of the flight inverted (|roll| up to 179°). Mean roll attitude error **60.8°**.
- **`001716` — clean.** Roll rate `Kp_eff ≈ 2.1e-4` (**~4×** higher) and pitch
  `Kp_eff ≈ 1.5e-4`. Now the loop tracks: roll `corr = +0.70`, pitch `+0.84`.
  Flown *harder* than 001607 (roll setpoint σ 17° vs 13°) yet **never exceeds
  90°** and only 11 % of the time past 45°. Mean roll error **10.3°**.
- **`001850` — late tumble.** Roll gain falls back to the weak regime
  (`Kp_eff ≈ 6.3e-5`, `corr = +0.46`); flies controlled until **t ≈ 51.4 s**,
  then a brief roll excursion past 90° (3 % of the run).

**Yaw — the reference axis — tracks cleanly wherever it is commanded**
(`corr = +0.99` in 001716/001850; not exercised in 001607). And in **every**
run the roll/pitch loop **never spends more than ~11 % of its ±1 authority** and
never saturates — the same "the gain never asks for the headroom it has" defect
documented on 2026-06-17 (rig) and 2026-06-20 (SITL). The 001716 gain proves a
modest bump (still ~45× below yaw) is enough to make the loop track and stop the
tumble.

The **vertical estimator is healthy** in all three (fused−baro RMS
0.21–1.08 m, `valid = 1` for 100 % of samples). There is **no altitude-hold
loop** — throttle is manual passthrough — so the large/extreme altitude
excursions (up to 4.2 km in 001850) are RC input plus the tumble dynamics, not
a controller fault. Timestep is rock-steady (`outer_dt = 4.000 ms`,
`inner_dt = 1.000 ms`, zero jitter) in every run.

## Inventory of this archive

- [`session-analysis.md`](session-analysis.md) — the three runs, timeline, what each did
- [`control-loop-analysis.md`](control-loop-analysis.md) — **the headline**: rate-loop tracking across the triplet, the gain differentiator, authority, the two tumbles
- [`vertical-analysis.md`](vertical-analysis.md) — vertical estimator health + the 001850 4.2 km artifact
- [`recommendations.md`](recommendations.md) — fixes + next-capture plan
- [`plots/`](plots/) — 5 comparison figures (generated by `make_plots.py` in this dir)
- `make_plots.py` — session-specific plotting (NOT the generic one; see its header)
- `csv/` — per-message CSV dumps (git-ignored, regenerable via `parse_log.py --csv`)

## Raw findings log

1. Three clean captures (37.5 / 70.4 / 59.3 s), **0 CRC errors**, decoder healthy.
2. `FlightMode = ANGLE (0)` for 100 % of all three — no ACRO blips.
3. Roll/pitch **angle setpoints are large and dynamic** (σ 9–17°, peaks to ±100°)
   — these are aggressive manual flights, NOT the level-hold of 2026-06-20.
4. **Roll rate tracking** `corr(rate_sp,rate_curr)`: 001607 **+0.08**,
   001716 **+0.70**, 001850 **+0.46**. Pitch: +0.11 / +0.84 / +0.45.
5. **Effective roll rate Kp** (slope of `out` vs `rate_err`): 5.4e-5 / **2.1e-4** /
   6.3e-5. The clean run (001716) has ~4× the gain of the two that tumble.
6. **Yaw tracks** wherever commanded: corr +0.99 (001716/001850). Yaw `Kp_eff`
   8.5–9.5e-3 — ~45× the roll/pitch gain even in the best run.
7. **Authority unused**: roll/pitch `_out` peaks 0.029–0.112 — never above ~11 %,
   never saturates, in any run. Yaw `_out` reaches 0.38.
8. **Roll tumbles**: 001607 crosses 90° at t≈20.2 s, 38 % of flight inverted,
   |roll|→179°. 001850 crosses at t≈51.4 s, 3 % inverted. 001716 never crosses.
9. **dt** rock-steady everywhere: `outer_dt` 4.000 ms, `inner_dt` 1.000 ms, 0 µs σ.
10. **Vertical estimator**: fused−baro RMS 0.74 / 0.21 / 1.08 m, max |3.06| m,
    `valid = 100 %` in all three. Peak altitude 328 / 900 / **4194** m;
    001850 also shows +202 m/s "climb" — an artifact of the tumble (see
    vertical-analysis.md), not a real climb-rate capability.
11. **Health**: `cpu_load = 0` (SITL), `tx_overflow = 0`, `imu_drop = 0` in all
    three. EstPerf mean ~11 µs, peak ≤ 230 µs.
12. **Motors** balanced (4-motor mean equal to 3 d.p. each run); peak duty
    0.51 / 0.78 / 0.76 — no motor ever saturates to 1.0.
