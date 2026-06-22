# On-hardware tuning session — 2026-06-21/22

Real FC over the ESP/UDP bridge (port 14555), monitored/driven from the host with
tools/sysid_excite.py, tools/sysid_fit.py, tools/apply_tune.py, and inline NavLink
monitors. All gains/geometry are LIVE-only (not persisted) — see data/tune_*.json.

## Combined analysis (start here)

A full cross-data report — every FC subsystem, fusing the sysid capture, the three
GCS recordings, and the reconstructed free-flight traces. Sequel to
[`../20260617-124210/`](../20260617-124210/).

| doc | covers |
|---|---|
| [`session-analysis.md`](session-analysis.md) | **combined overview**, provenance, per-log identity, the whole-night story & verdict |
| [`control-loop-analysis.md`](control-loop-analysis.md) | cascade + sysid plant, the **~1.4 Hz pitch limit cycle**, sign-of-feedback audit |
| [`motor-analysis.md`](motor-analysis.md) | mixer, **yaw-sign bug confirmed+fixed from data**, **left-thrust bias**, saturation |
| [`sensor-analysis.md`](sensor-analysis.md) | accel scale (+7.8 %→+2.5 %), mag, gyro, EKF fusion, calibration evidence |
| [`kernel-analysis.md`](kernel-analysis.md) | vaios CPU/stacks/heap, **the 001043 IPC-timeout anomaly**, FIFO drops |
| [`recommendations.md`](recommendations.md) | prioritised P0–P3 fixes + status of the 06-17 recommendations |

Reproduce any number from repo root:
```sh
python3 docs/journal/log-analysis/parse_log.py docs/journal/log-analysis/20260622-012844-onhw-tune/data/rig_telem_235304_sysid-run.bin
python3 docs/journal/log-analysis/parse_log.py <that.bin> --csv /tmp/csv   # per-message CSVs
```

---


## Phases
1. **Rig** (free-rotating gimbal, raised inertia): diagnosed+fixed a throttle-up
   runaway, ran roll system-ID, designed gains. STABLE on the rig.
2. **Free flight** (rigless): repeated takeoff attempts, all ended in a crash, every
   one diving the SAME way (left). => suspected physical/sign issue, not gains.

## Rig results
- Runaway root causes (NOT the sysid chirp):
  - roll/pitch default `ki=0.01` (knee 20 rad/s, above crossover) + the plant's own
    integrator => double-integrator => growing oscillation. Fix: pure-P (ki=0).
  - yaw default spin-sign BACKWARDS for this build => positive feedback => spin-up.
    Fix: CMD_SET_MOTOR_GEOMETRY spin=[-1,1,-1,1] (flips s_mix_yaw only).
  - After both: STABLE at 46% throttle (peak gyro roll15 pitch27 yaw8 dps).
- **Roll system-ID** (1 chirp 0.5->12Hz @47% throttle, captured u=PID-output + gyro,
  data/sysid_roll_capture.csv): plant omega/u = K/(s(tau s+1)),
  **K=563 (rate/s)/u, tau=20.9ms, actuator BW 7.6Hz, R^2=0.83**.
  Designed (loop-shaped): rate kp=0.012 ki=0.0081 kd=0.00025, angle_kp=1.69.

## Free-flight attempts (all crashed LEFT)  — see data/flight_traces.txt
| run | rate kp | angle kp | throttle | peak gyro r/p/y (dps) | outcome |
|-----|---------|----------|----------|------------------------|---------|
| FF-1 | 0.0001476 (autotune) | 4.317 | 0.22 | 102 / 29 / 87 | never lifted, toppled (too soft); yaw drift (no ki) |
| FF-2 | 0.001  | 4.317 | ~0.38 | 571 / 195 / 308 | lifted slightly, rolled LEFT |
| FF-3 | 0.004  | 4.317 | 0.44 | 731 / 592 / 237 | flew 4-5 ft, OSCILLATED, disarmed |
| FF-4 | 0.0025 | 4.317 | 0.56 | 760 / 412 / 239 | brisk takeoff, BIG oscillation/diverge |
| (power cycle — re-applied full tune) |||||
| FF-5 | 0.004  | **1.0** | 0.58 | 344 / 494 / 268 | IN_AIR, immediately dived LEFT into a wall |

Yaw fix mid-session: restored yaw integral (kp=0.018 ki=0.008) -> yaw drift gone
(yaw held heading, drift -5 dps).

## Diagnosis / open item
- Reducing rate kp made the oscillation WORSE (FF-3 0.004 -> FF-4 0.0025: 731->760)
  => CASCADE inversion: the autotune `angle_kp=4.317` outer loop was faster than the
  soft rate loop. Softening angle_kp to 1.0 (FF-5) still crashed.
- **The decisive signal: it dives LEFT every time, independent of all gain changes.**
  Gains don't pick a direction. Combined with the proven-wrong yaw spin-sign, the
  prime suspect is a **roll/pitch mix/wiring SIGN error that the rig MASKED** (the
  gimbal constrained roll/pitch, so they read "stable" when they weren't).
- **NEXT (not yet done): props-off motor-command validation** — arm props-off at low
  throttle, tilt the airframe by hand, read MOTOR_TELEMETRY to confirm the firmware
  spins up the correct motors to correct roll/pitch. If a sign is inverted, flip it
  (as for yaw) before any further flight/tuning.

## Files
- data/sysid_roll_capture.csv      — the armed roll sysid (t, u=PID-output, gyro_dps), 500 Hz, the fit input
- data/sysid_roll_capture_early.csv — an earlier disarmed capture
- data/tune_rig.json               — rig sysid tune (roll kp=0.012 ...)
- data/tune_freeflight.json        — free-flight tune progression (current: rate kp=0.004, angle kp=1.0, yaw+integral, geometry sign fix)
- data/flight_traces.txt           — raw per-0.35s gyro/throttle/state traces for the key runs

## GCS-recorded telemetry — the real per-sample data
Found in ~/vayu-logs/ (VREC NavLink recordings from the Navigator GCS this tuning
night); all decode with 0 CRC errors. Decode with
`docs/journal/log-analysis/parse_log.py <file> --csv <dir>`. Far richer than the
reconstructed flight_traces.txt (full-rate ControlTrace+Attitude+Motor).

RIG runs (in `data/`, with the rig sysid CSVs + tune_rig.json):

| run | dur | thro | gyro pk r/p/y | content |
|---|---|---|---|---|
| rig 235304 | 2.55 min | 0.57 | 84/322/42 | THE rig tuning run: 118 s in flight-throttle band, ANGLE→ACRO |
| rig 235115 | 1.68 min | 0.41 | 68/378/50 | rig: 378 °/s PITCH oscillation under throttle (ANGLE) |

FREE-FLIGHT run (in `freeflight/`):

| run | dur | thro | gyro pk r/p/y | content |
|---|---|---|---|---|
| ff 001043 | 1.18 min | 0.29 | 133/114/701 | 701 °/s YAW spin, throttle-gated (no real departure) |

<sub>Files: `data/rig_telem_235304_sysid-run.bin`, `data/rig_telem_235115.bin`,
`freeflight/freeflight_001043_yaw-departure.bin`.</sub>

NOT kept: export-20260622-000025.bin (clean but idle, thro<=0.21, no event).
Out of scope: export-20260621-054535.bin (277 MB, 17:36) is the earlier long
SITL/EKF aggressive-flight run, not this hardware tuning — left in ~/vayu-logs/.
