# 2026-09-09 00:17 — the IMU was never configured

**Headline: the BMX160 has been running on its power-on defaults since the
beginning — ~100 Hz ODR and ±2 g range — and nothing in the driver ever writes
a configuration to it.** Every accel-health failsafe, the 2026-09-07 flyaway's
collapsing `|accel|`, and the FFT notch's inability to find anything are all
downstream of that one fact.

This is the first archive built from the **high-speed IMU-to-SD recorder**
(`storage/imu_hs_log`, added 2026-09-08), not from telemetry. Telemetry caps at
50 Hz and had never been able to show any of this.

## Provenance

| | |
|---|---|
| capture | `data/imuhs-20260909-001748.bin` — the live part of `0:imuhs.bin`, read off the SD card |
| firmware | `build-notch` (`VAYU_FFT_NOTCH=ON`, `VAYU_GYRO_NOTCH_FORCE_ON=ON`), flashed 2026-09-09 00:16 |
| airframe | 5" racer, 2400 KV, 3S — bench, props on, never intentionally flown |
| arms | 20 sessions, 136.7 s of armed time, 248,078 IMU samples |
| ring | 7368 of 65535 sectors (11.2%), 0 wraps, **0 dropped sectors** |

Only arm 1 carries a real wall clock (`2026-09-09 00:17:48`); the rest were
recorded in power cycles where the GCS never sent a `TIME_SYNC`, so their
`SESSION` frames hold FC uptime instead. See `analysis.md` §8.

## Files

```
data/imuhs-20260909-001748.bin   3.77 MB   the recording (trimmed to live sectors)
analysis.md                                what the data shows
recommendations.md                         what to change, in order
plots/accel-vs-throttle.png                the core finding, all 20 arms
plots/run9-timeline.png                    one arm, throttle -> collapse -> failsafe
plots/sample-staleness.png                 1827 Hz logging of a ~97 Hz sensor
```

## Reproducing

```sh
python3 tools/telemetry/hslog.py \
    firmware/docs/journal/log-analysis/20260909-001748-imu-never-configured/data/imuhs-20260909-001748.bin
python3 tools/telemetry/hslog.py <file> --fft      # per-arm spectra
python3 tools/telemetry/hslog.py <file> --csv out.csv
```

The decoder reconstructs arms from the ring's `seq` ordering; slot order is not
time order once it has wrapped. Format spec:
`firmware/include/storage/imu_hs_log.h`.

## Status

Nothing here is fixed yet. **Do not fly on this firmware** — see
`recommendations.md` P0. The 20 arms in this capture are bench runs; 6 of them
ended in FAILSAFE on the accel-health veto, which is the guard working
correctly against data that should never have looked like that.
