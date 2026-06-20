# Session analysis — three autotune-validation flights, 2026-06-21

Three SITL sessions recorded back-to-back during the 2026-06-21 autotune /
System-ID work, ~70 s apart on the wall clock:

| run      | local time | span    | what it shows                                |
|----------|-----------|---------|----------------------------------------------|
| `001607` | 00:16:12  | 37.5 s  | weak roll gain → **tumble** (38 % inverted)  |
| `001716` | 00:17:21  | 70.4 s  | ~4× roll gain → **clean** aggressive flight  |
| `001850` | 00:18:52  | 59.3 s  | weak roll gain again → **late tumble**       |

All three are flown in **ANGLE (stabilize) mode** for 100 % of their duration —
no ACRO blips, unlike the 2026-06-17 rig session. The operator commands
roll/pitch/throttle (and yaw in the last two) by hand; there is no autopilot
guidance loop and no altitude hold. So every run is a manual stress-test of the
**stabilisation cascade** under large, dynamic attitude commands.

## These are aggressive flights, not a level-hold

The 2026-06-20 SITL analysis was a *level-hold* test (roll/pitch sticks centred,
setpoint ≈ 0). This triplet is the opposite: roll/pitch angle setpoints have
**σ 9–17°** and reach **±100°**. That matters for reading the numbers:

- Attitude *error* is no longer "how far from level" — it is "how well the loop
  followed a moving, aggressive command".
- The rate-loop **tracking correlation** `corr(rate_sp, rate_curr)` becomes the
  honest single number for loop health, because both signals now have real
  dynamic range to correlate over.

## What each run did

### `001607` — the tumble (37.5 s)

A short flight that loses the roll axis. Roll tracks for the first ~20 s, then
under continued roll stick the under-gained loop can't hold: roll crosses 45° at
**t ≈ 19.5 s**, 90° at **t ≈ 20.2 s**, and stays inverted (|roll| up to **179°**)
for **38 %** of the remaining flight. Mean roll attitude error over the whole
run is **60.8°**. Pitch stays comparatively tame (mean error 8.6°, peak 73.7°)
and yaw is **not commanded at all** (yaw rate setpoint ≡ 0). Peak altitude
328 m; motors peak 0.51 duty.

### `001716` — the clean run (70.4 s)

The longest and best flight. Flown *harder* than 001607 (roll setpoint σ **17°**
vs 13°, the most aggressive of the three) yet **never tumbles** — roll never
crosses 90° and is past 45° only 11 % of the time. Roll/pitch/yaw rate loops all
track (corr **+0.70 / +0.84 / +0.99**). Mean roll error **10.3°**, pitch 3.0°,
yaw 1.9°. This is what the cascade looks like when the inner loop has enough
gain to do its job. Peak altitude 900 m; motors peak 0.78.

### `001850` — the late tumble (59.3 s)

Back to the weak roll regime. Flies controlled for most of the run, then a brief
loss past 90° starting at **t ≈ 51.4 s** (3 % of the flight inverted, |roll| to
179°). Roll rate corr **+0.46**, pitch +0.45 — middling. Mean roll error 12.7°.
This run also logs an extreme vertical excursion (peak altitude **4194 m**, a
+202 m/s transient) that is a by-product of the tumble dynamics, not a real
climb — see [vertical-analysis.md](vertical-analysis.md). Peak motor duty 0.76.

## Timeline (per-run, t relative to each run's first frame)

```
001607  |--- roll tracks ---|XX tumble: |roll|>90° from 20.2 s, 38% inverted XX|   (37.5 s)
001716  |---------------- clean: |roll| never >90°, 11% past 45° ----------------|  (70.4 s)
001850  |------------- controlled -------------|x late tumble from 51.4 s, 3% x|    (59.3 s)
```

## Cross-run summary

| metric                         | 001607 | 001716 | 001850 |
|--------------------------------|-------:|-------:|-------:|
| roll rate corr(sp,curr)        | +0.08  | **+0.70** | +0.46 |
| pitch rate corr(sp,curr)       | +0.11  | **+0.84** | +0.45 |
| yaw rate corr(sp,curr)         |  n/c¹  | +0.99  | +0.996 |
| roll rate Kp_eff               | 5.4e-5 | **2.1e-4** | 6.3e-5 |
| pitch rate Kp_eff              | 2.5e-6 | 1.5e-4 | 1.7e-4 |
| yaw rate Kp_eff                |  n/c¹  | 9.5e-3 | 8.5e-3 |
| roll out peak (of ±1)          | 0.085  | 0.112  | 0.085  |
| mean \|roll attitude error\|   | 60.8°  | **10.3°** | 12.7° |
| % flight inverted (\|roll\|>90)| 38 %   | **0 %**   | 3 %   |
| vertical fused−baro RMS        | 0.74 m | 0.21 m | 1.08 m |
| peak altitude                  | 328 m  | 900 m  | 4194 m |
| `outer_dt` / `inner_dt`        | 4.000 / 1.000 ms (0 jitter) ||||

¹ *n/c = not commanded:* yaw stick was untouched in 001607 (yaw rate setpoint ≡ 0),
so its correlation/gain are undefined for that run.

The control-loop story behind these numbers is in
[control-loop-analysis.md](control-loop-analysis.md).
