# Sensor report — `export-20260617-124210.bin`

Focused look at the **sensors**: calibration fidelity and reporting quality.
Source is the `ImuRaw` (full snapshot) and `ImuCompressed` (delta) streams plus
`SystemHealth`/`Heartbeat` context. All numbers reproducible via
`../parse_log.py`. Companion to the [session analysis](session-analysis.md).

## Verdict at a glance

| Sensor | Calibration | Reporting | Verdict |
|---|---|---|---|
| **Gyroscope** | bias < 0.1 °/s, noise 0.3–0.5 °/s | clean | ✅ **good** |
| **Accelerometer** | **+8 % scale error** at rest | clean | ⚠ **needs scale cal** |
| **Magnetometer** | **uncalibrated** (hard+soft iron) | reported but unusable | ❌ **fail** |
| **Temperature** | 35.6–37.1 °C, sane drift | clean | ✅ **good** |

**No calibration was performed or captured in this session** — zero
`CalibrationStatus` messages and `nav_state` never entered `CALIBRATING`. The
sensors ran on default/whatever-was-stored calibration, which is consistent with
the accel/mag errors below. Units in use: **gyro °/s, accel m/s², mag µT,
temp °C** (the codec field comment guessing "rad/s" for gyro is wrong — verified
against `ControlTrace.*_rate_curr`, abs max ~100 either way).

## Gyroscope — good

Measured over the cleanest stationary window (177–205 s, 36 samples, board
still):

| axis | bias (°/s) | noise σ (°/s) |
|---|---:|---:|
| x | +0.092 | 0.348 |
| y | −0.013 | 0.279 |
| z | +0.033 | 0.548 |

Bias is **< 0.1 °/s on every axis** and noise is sub-degree — a well-behaved,
well-zeroed gyro. Full-session range is ±~100 °/s during handling, with no
clipping or stuck values. This is the one sensor that is clearly trustworthy.

## Accelerometer — ~8 % scale error

Gravity magnitude `|acc|` should read **9.807 m/s²** at rest. It doesn't:

| condition | samples | mean `|acc|` | error vs g |
|---|---:|---:|---:|
| static, **motors OFF** | 52 | 10.571 | **+7.8 %** |
| static, **motors ON** | 32 | 10.105 | +3.0 % |

The over-read is **largest with motors off**, so it is **not** vibration
rectification (that would push the motors-on figure higher, not lower) — it is a
genuine **scale-factor / calibration error of ~8 %**. Per-axis noise at rest is
low (σ 0.04–0.08 m/s²), so the sensor is quiet; it's just mis-scaled. A 6-side
accelerometer calibration should bring this in line. Until then, anything that
trusts accel magnitude (level estimation, gravity vector, vertical accel) is
biased high by ~8 %.

> The motors-on figure being *closer* to g is a coincidence of orientation/linear
> motion in those samples, not better calibration — treat the **+7.8 % motors-off**
> number as the real static error.

## Magnetometer — uncalibrated (do not use as-is)

A calibrated magnetometer reads a near-**constant** field magnitude (Earth's
field, ~25–65 µT) in every orientation. Here `|mag|` swings **22.3 → 92.4 µT**, a
spread of **140 % of the mean** — the signature of an uncorrected hard- and
soft-iron field. Per-axis min/max imply large hard-iron offsets:

| axis | min | max | hard-iron offset | half-span |
|---|---:|---:|---:|---:|
| x | −87.2 | +37.0 | **−25.1** | 62.1 |
| y | −37.7 | +57.2 | **+9.7** | 47.5 |
| z | −29.1 | +70.7 | **+20.8** | 49.9 |

Non-zero centers (offsets up to 25 µT) and unequal half-spans (47–62 µT) mean
both hard-iron (bias) and soft-iron (scale/skew) correction are missing. The data
*is* present and plausibly scaled (µT), but **unusable for heading** until a full
ellipsoid-fit magnetometer calibration is run. (Note: this session also ran
motors, whose current adds field distortion — calibrate *with the frame powered*
and ideally characterize throttle-dependent interference.)

## Temperature — good

IMU `temp` reads **35.6 – 37.1 °C** with a gentle warm-up drift
(35.9 → 36.4 °C over the session). Sensible absolute values and smooth — no
glitches. (No evidence of temperature compensation being applied to gyro/accel,
but the channel itself reports correctly.)

## Reporting fidelity

The IMU uses a **keyframe + delta** scheme:

| stream | rate | payload | role |
|---|---:|---|---|
| `ImuRaw` | ~1.6 Hz (median dt 621 ms, **max gap 2.44 s**) | 10× f32 (acc,gyr,mag,temp) | absolute keyframe |
| `ImuCompressed` | ~38 Hz (median dt 26 ms) | 10× f16 delta | high-rate deltas |

8,764 delta frames vs 508 keyframes (**17.3×**). The delta half-floats decode to
real, non-trivial values, so the compression path is alive. **But two reporting
bugs undercut it:**

1. **`ImuCompressed.ref_seq` is stuck at 0** for all 8,764 frames (only one
   distinct value). The whole point of `ref_seq` is to bind each delta to the
   keyframe it was differenced against; with it pinned to 0, a consumer **cannot
   reliably reconstruct absolute IMU** from the compressed stream — it can't tell
   which of the 508 keyframes a delta belongs to. Combined with keyframe gaps up
   to 2.44 s, blind delta accumulation will drift. **This is a fidelity bug, not
   just cosmetic.**
2. **`ImuRaw.sample_time_us` is always 0.** No per-sample hardware timestamp, so
   IMU frames can only be timed by their telemetry-arrival `t_us` (jittery,
   subject to the link stalls noted in the session report). Real sample timing is
   lost.

Also relevant to "sensor reporting": **`AttitudeEuler` body rates
(`rollspeed/pitchspeed/yawspeed`) are always 0** — the fused-attitude angular
rates aren't populated, even though the raw gyro carries them (see the
[session report](session-analysis.md), issue #1).

## Recommendations (sensor priority order)

1. **Run accelerometer calibration** (6-side) — remove the ~8 % scale error
   before trusting any accel-derived estimate.
2. **Run magnetometer calibration** (ellipsoid fit, frame powered) — current mag
   is unusable for heading; characterize motor-current interference while at it.
3. **Fix `ImuCompressed.ref_seq`** to carry the real keyframe seq so the delta
   stream is reconstructable; consider a faster `ImuRaw` keyframe cadence (the
   2.4 s gaps are large for a 38 Hz delta stream).
4. **Populate `ImuRaw.sample_time_us`** so IMU samples can be time-aligned from
   their own payload.
5. **Capture a calibration session** (drive `CALIBRATING` + `CalibrationStatus`)
   and re-log so the stored cal can be verified against fresh data.
6. Gyro and temperature need no action — they're reporting correctly.
