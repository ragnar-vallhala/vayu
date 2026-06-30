# Magnetometer spread check — 2026-06-25 23:09

A dense hand-rotation magnetometer capture off the real FC over the ESP/UDP
bridge (`10.42.0.30:14555`), reduced to the **spread** (XY/YZ/XZ projections),
**mean**, and **% off the mean** field magnitude. A verification spot-check that
the mag stays well-centred after the
[`../20260625-032526-imu-calib-verify/`](../20260625-032526-imu-calib-verify/)
calibration overhaul.

## Report (start here)

| doc | covers |
|---|---|
| [`mag-analysis.md`](mag-analysis.md) | the full check — spread/mean/%-off plots, LS-vs-mean centre, method |

## Results at a glance

| metric | value |
|---|---:|
| points | 2240 @ ~50 Hz (45 s, 0 % loss) |
| `‖m‖` mean | 40.5 µT |
| CoV | 4.95 % |
| hard-iron residual (LS centre) | **0.41 µT** |

Mag is **heading-grade**; no re-calibration needed.

## Data

| file | capture | rows |
|---|---|---|
| `mag_rotation_50hz.csv` | board rotated through all orientations, 45 s | 2240 |

Columns: `t,ax,ay,az,gx,gy,gz,mx,my,mz,temp` (accel m/s², gyro dps, mag µT,
temp °C). Captured live and reconstructed to 50 Hz from `ImuRaw` keyframes
(~1.7 Hz) + `ImuCompressed` f16 deltas, via the sniffer's recorder:

```sh
python3 tools/telemetry/udp_telem_sniff.py --seconds 45 \
    --mag-csv docs/journal/log-analysis/20260625-230911-mag-spread/mag_rotation_50hz.csv
```

Regenerate the figures from the CSV:
```sh
python3 docs/journal/log-analysis/20260625-230911-mag-spread/make_plots.py
```

- `plots/01_mag_planes.png` — mX–mY / mY–mZ / mX–mZ spread + mean & LS centre
- `plots/02_mag_pct_off_mean.png` — `‖m‖` over rotation + per-sample % off mean
