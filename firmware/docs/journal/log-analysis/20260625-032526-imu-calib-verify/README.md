# IMU calibration overhaul + on-hardware verification — 2026-06-25

Rebuilt the IMU calibration (firmware) and verified all three sensors on the real
FC over the ESP/UDP bridge (`10.42.0.30:14555`). Directly closes the sensor
concerns from [`../20260622-012844-onhw-tune/`](../20260622-012844-onhw-tune/)
(accel scale, mag hard-iron, unverified gyro bias). Sequel in the log-analysis
journal.

## Report (start here)

| doc | covers |
|---|---|
| [`sensor-analysis.md`](sensor-analysis.md) | **the full report** — what we built (shared `src/calib` module, pose-tolerant full-3×3 accel, stillness-gated gyro), per-sensor before/after with plots, 50 Hz method, and the status of every 06-22 sensor recommendation |

## Results at a glance

| sensor | before | after | metric |
|---|---:|---:|---|
| Accel | +4.2 % | **−0.08 %** | `‖a‖` vs g (σ 0.013) |
| Gyro | ~0.45 dps | **<0.04 dps** | per-axis bias |
| Mag | ~24 µT / CoV 25 % | **3.2 µT / CoV 7.3 %** | hard-iron residual (LS fit) |

Firmware commits `6755073`·`f1cca38`·`56aedfb`·`1336d8c`·`22c42b2`.

## Data

| file | capture | rows |
|---|---|---|
| `imu_static_accel_gyro_50hz.csv` | board static, ~25 s | 1108 |
| `mag_rotation_50hz.csv` | board rotated through all orientations, ~35 s | 1627 |

Columns: `t,ax,ay,az,gx,gy,gz,mx,my,mz,temp` (accel m/s², gyro dps, mag µT,
temp °C). Captured live and reconstructed to 50 Hz from `ImuRaw` keyframes
(~1.7 Hz) + `ImuCompressed` f16 deltas (drop-aware).

Regenerate the figures from the CSVs:
```sh
python3 docs/journal/log-analysis/20260625-032526-imu-calib-verify/make_plots.py
```

- `plots/01_accel_magnitude.png` — `‖a‖` static + distribution
- `plots/02_gyro_bias.png` — gyro axes + before/after bias bars
- `plots/03_mag_sphere.png` — mag XY/XZ/YZ projections + LS sphere fit
- `plots/04_mag_norm.png` — `‖m‖` over rotation
