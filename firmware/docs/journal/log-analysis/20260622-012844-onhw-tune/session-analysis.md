# Combined session analysis — on-hardware tune, 2026-06-21/22

This archive analyses the **whole on-hardware tuning night**, not a single capture.
It fuses four independent data sources collected across the rig→free-flight effort:

1. **`data/sysid_roll_capture.csv`** — the armed roll system-ID (500 Hz `u`/gyro).
2. **`data/rig_telem_235115.bin`** — rig run, ANGLE mode (VREC GCS recording).
3. **`data/rig_telem_235304_sysid-run.bin`** — rig run, ANGLE→ACRO, the substance.
4. **`freeflight/freeflight_001043_yaw-departure.bin`** — a later low-throttle test.

plus `data/flight_traces.txt` (reconstructed harness monitor of the un-recorded
free-flight crashes) and the two live tunes (`data/tune_*.json`). Every `.bin`
decodes with **0 CRC errors** via `../parse_log.py` (generated NavLink v2 codec);
every number below is reproducible from the archived files.

This session is the **direct sequel to [`../20260617-124210/`](../20260617-124210/)**,
which found the roll/pitch rate loop unstable (Kp=5e-4, **Kd=0**, integral-dominant
Ki=0.01) and a ~8 % accel scale error + uncalibrated mag, and recommended: add Kd,
raise Kp off the integrator, lower Ki, calibrate accel+mag, **stay rig-only until
boring**. We did the retune (via on-hardware system-ID) and the calibration — this
report grades the result.

## TL;DR

- **The retune removed the slow limit cycle and created a faster one.** 2026-06-17
  oscillated at **~0.3 Hz** (integral-driven). With the sysid gains the rig now
  oscillates at **~1.4 Hz in PITCH** (peaks 322–378 °/s) — a **cascade** limit cycle
  (outer angle loop faster than the softened inner rate loop), exactly the failure
  the free-flight notes describe. Pitch is the dominant unstable axis in *recorded*
  data. See [`control-loop-analysis.md`](control-loop-analysis.md).
- **The yaw spin-sign bug is confirmed *and* its fix is verified, from data.** In
  both rig logs the yaw motor-differential is **anti-correlated** with `yaw_out`
  (corr −0.76, −0.70) under the firmware default mix → the default yaw sign drove
  **positive feedback**. The later free-flight log (after `CMD_SET_MOTOR_GEOMETRY
  spin=[-1,1,-1,1]`) flips to **+0.75** → the fix realises the correct yaw torque.
  See [`motor-analysis.md`](motor-analysis.md).
- **A persistent LEFT-heavy thrust bias is visible even on the rig.** Across both
  rig runs the left pair commands **+24–26 % more** thrust than the right
  (L−R = +0.24…+0.26 of a motor-unit) and **M3 (rear-left) saturates** (148 frames
  at 1.00 in the powered run). This is the recorded fingerprint of the same
  left-drop that crashed every free-flight attempt — a real bias the loop is
  fighting, **not** explained by the (correct-looking) roll/pitch mixer signs.
- **Roll/pitch mixer signs look correct in the data** (corr +0.37…+0.77). The
  earlier "roll/pitch sign inverted" hypothesis is **not** corroborated by these
  logs; the left bias points to thrust/CG/trim, not a mix flip (though the rig
  constrains roll, so roll is not 100 % excluded — see recommendations).
- **Calibration improved but isn't finished.** Accel scale error fell from **+7.8 %**
  (06-17) to **+2.5–5.1 %**; mag field-magnitude spread tightened from ~140 % to as
  low as 64 % of mean in the post-cal log — but hard-iron offsets (z ≈ +25 µT)
  remain. See [`sensor-analysis.md`](sensor-analysis.md).
- **Kernel still healthy, two regressions to watch:** `task_id 4` stack crept
  74 %→**80 %**, and the free-flight log shows **`ipc_timeouts = 7099`** (was 0 on
  06-17) alongside the IMU_CALIB FIFO saturating — a new contention signal. Control
  FIFOs still never drop; **`tx_overflow` is now 0** (was 2 k) — the downlink
  saturation is gone. See [`kernel-analysis.md`](kernel-analysis.md).
- **System-ID is sound.** Roll plant ω/u = K/(s(τs+1)), **K=563, τ=20.9 ms,
  BW 7.6 Hz, R²=0.83**; loop-shaped gains are correctly signed (controller `out`
  opposes rate error, corr +0.6…+0.84 on the engaged axes).
- **Telemetry data-quality bugs from 06-17 are all still open:** `AttitudeEuler`
  body rates = 0, `ImuRaw.sample_time_us` = 0, `SystemHealth.cpu_load` = 0.

## Provenance & integrity

All originals from `~/vayu-logs/export-<export id>.bin`; archived under `data/` or
`freeflight/` with the descriptive names used throughout this report.

| run | export id | size (B) | SHA-256 (head…tail) |
|---|---|---|---|
| rig 235115 | 20260621-235115 | 688,339 | `b6e7e7d5…282593` |
| rig 235304 (sysid) | 20260621-235304 | 1,042,514 | `4b351d4d…eadc9f` |
| ff 001043 | 20260622-001043 | 458,209 | `68179bb1…a49afe` |

All `VREC` format v1 / protocol v1, 0 CRC errors, 0 unknown msgids, 0 trailing
bytes. Wall-clock starts (epoch s): 235115 = 1782066078, 235304 = 1782066187,
001043 = 1782067246 — i.e. **2026-06-21 ~23:51, ~23:53 and 2026-06-22 ~00:10 local
(IST)**. The FC clock (`t_us`) is monotonic within each but not shared (separate
power states); 001043's `t_us` base differs (561 M vs 11 M) → a power cycle between
the rig runs and 001043, consistent with the "re-apply the tune each boot" note.

## Per-log identity card

| | rig_235115 | rig_235304 | ff_001043 |
|---|---|---|---|
| duration | 1.68 min | 2.55 min | 1.18 min |
| nav_state reached | **IN_AIR** (56 s) | **IN_AIR** (122 s) | ARMED only (no IN_AIR) |
| flight mode | ANGLE | ANGLE→ACRO→ANGLE | ANGLE→ACRO |
| throttle max / active mean | 0.41 / 0.36 | **0.57 / 0.48** | 0.29 / 0.21 |
| AGL range | −1.2…+1.0 m | −0.7…+1.0 m | −1.2…+0.6 m |
| gyro pk r/p/y (°/s) | 68 / **378** / 50 | 84 / **322** / 42 | 133 / 114 / **701** |
| dominant osc | pitch ~1.6 Hz | pitch ~1.4 Hz | yaw 701 (gated, near-idle) |
| left−right thrust | **+0.239** | **+0.255** | +0.010 (idle) |
| motor saturation | M3 2 frames | **M3 148, M4 59** | none |

**Note on "IN_AIR".** Both rig runs *latched* `IN_AIR` (nav_state 5) but **AGL never
left ±1 m** — the state machine tripped on throttle/dynamics while the rig held the
frame, so "IN_AIR" here is not real altitude. ff_001043 never even latched IN_AIR
(throttle ≤0.29, authority-ramp gated, motors ≈0.04–0.23): its 701 °/s yaw is an
external/hand spin the gated loop couldn't oppose, **not** a powered departure. The
real free-flight crashes (powered, ~0.44–0.58 throttle, left dive) were monitored
over the harness and were **not** GCS-recorded — they survive only in
`data/flight_traces.txt`.

## Message inventory (per log, Hz)

Same NavLink v2 stream as 06-17. Counts/rates (decoded frames = records, 0 CRC):

| message | 235115 | 235304 | 001043 |
|---|---:|---:|---:|
| ImuCompressed | 2609 (25.9) | 3917 (25.6) | 1809 (25.5) |
| ControlTrace | 2110 (20.9) | 3201 (21.0) | 1471 (20.7) |
| MotorTelemetry | 1902 (18.9) | 2810 (18.4) | 1303 (18.4) |
| AttitudeEuler | 1062 (10.5) | 1648 (10.8) | 730 (10.3) |
| RcChannels | 1026 | 1593 | 712 |
| Baro / VerticalState | 1010 / 994 | 1526 / 1438 | 663 / 646 |
| PerfTask / PerfFifo / PerfGlobal | 977 / 478 / 100 | 1388 / 715 / 134 | 591 / 246 / 62 |
| Heartbeat / FlightMode / SystemHealth | 327 / 323 / 320 | 500 / 483 / 480 | 221 / 216 / 200 |
| ImuRaw / EstPerf / TimeSync | 150 / 76 / 18 | 210 / 119 / 29 | 85 / 54 / 15 |

No `Statustext`, `CommandAck`, `Param*`, or `CalibrationStatus` in any log
(calibration was driven over the separate harness, not the GCS link).

## How the pieces fit — the combined story

1. **06-17 left us** with a soft, undamped rate loop (0.3 Hz limit cycle), an 8 %
   accel error, and an uncalibrated mag. Plan: sysid → gains, calibrate, rig-only.
2. **We calibrated** (gyro/accel/mag over the harness): accel error → +2.5–5 %, mag
   spread → as low as 64 %. Partial success (§sensors).
3. **We ran on-hardware roll system-ID** (`sysid_roll_capture.csv`): K=563, τ=20.9 ms,
   → loop-shaped gains (rate kp 0.012, ki 0.0081, kd 0.00025; angle_kp 1.69). On the
   rig this is **stable in roll** (rig constrains it) but **pitch — still on stale
   defaults / its own untuned seed — limit-cycles at ~1.4 Hz** (§control-loop).
4. **We found and fixed the yaw positive-feedback** (geometry spin flip). The rig
   logs captured the *broken* sign (mix corr −0.7); 001043 captured the *fixed* sign
   (+0.75) (§motors).
5. **We went to free flight** (against 06-17's "rig-only until boring") and it dove
   left every time. The rig logs already carried the warning: a **+25 % left-heavy
   thrust bias** and M3 saturation the gimbal was hiding (§motors, §control-loop).

The honest conclusion: the rig made roll/pitch *look* tuned because the gimbal
absorbed the left bias and the pitch cycle was "only" 1.4 Hz; free flight removed
the constraint and the bias + cascade took over. **Fix the cascade (slow the angle
loop / finish pitch sysid), find the left-thrust bias (props-off + per-motor
check), then re-test on the rig until AGL-real-IN_AIR is boring — before flight.**

> Deep dives: [`control-loop-analysis.md`](control-loop-analysis.md) ·
> [`motor-analysis.md`](motor-analysis.md) ·
> [`sensor-analysis.md`](sensor-analysis.md) ·
> [`kernel-analysis.md`](kernel-analysis.md) ·
> [`recommendations.md`](recommendations.md). Tuning narrative & file inventory:
> [`README.md`](README.md).
