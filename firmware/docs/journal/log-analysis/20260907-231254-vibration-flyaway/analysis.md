# Findings

Ten findings across both captures, by severity. File 1 =
`export-20260907-231254.bin`, file 2 = `export-20260907-231706.bin`.

---

## Critical

### 1. The accelerometer stops measuring gravity when the props spin

Raw `IMU_RAW` magnitude, `sqrt(ax²+ay²+az²)`, split by mean motor command:

| capture | motors off | motors > 0.30 |
|---|---|---|
| 2026-09-06 19:32 (**the first hover**) | 9.671 | **2.651** |
| file 1 | 9.860 | **2.579** |
| file 2 | 9.713 | **2.542** (min **0.201**) |

A stationary craft must read 9.81. It reads ~2.5, with individual samples down to
0.20 — essentially free-fall. This is not an offset that can be subtracted; the
signal is gone. PX4 names the mechanism exactly: *"clipping usually causes the
average accel reading to move towards zero"*
([`height_control.cpp`](../../../plans/vertical-velocity-vibration.md)).

Note the middle row: **this was already true during the hover that flew fine.**
It flew because stabilise rate control is gyro-driven and the hop was short.

An earlier note in [`vertical-velocity-vibration.md`](../../../plans/vertical-velocity-vibration.md)
ruled clipping out on the grounds that the range is ±8 g against 0.1 g of bench
noise. That reasoning was built on **props-off** data and does not survive
contact with a spinning rotor.

### 2. Uncommanded climb — airmode converting saturation into collective

File 2, `t = 93.9 s`, with the height mode `OFF` the whole time:

```
 t=93.6  stick 0.54 | M 0.36 0.64 0.50 0.26  mean 0.44  agl 0.77 m
 t=93.9  stick 0.38 | M 1.00 1.00 0.43 0.15  mean 0.64  agl 1.47 m   <- 1.69x
 t=94.2  stick 0.02 | M 0.15 0.15 0.15 0.15  mean 0.15  agl 1.60 m
```

The stick came **down** 0.54 → 0.38 while the mean motor command went **up**
0.44 → 0.64. Two motors pinned at 1.00, a third on the `MOTOR_IDLE_FLOOR` of
0.15. `s_airmode = MIXER_AIRMODE_RP` (`angle_rate_controller.c:146`) preserves
roll/pitch authority under saturation by shifting collective upward — the
mechanism previously measured as capable of 3.54× commanded thrust.

This was not confined to the final event. **A motor sat on the idle floor while
another exceeded 0.2 in 2.7% of samples in *both* files**, so the attitude loop
was fighting the whole session, not just at the end.

| | file 1 | file 2 |
|---|---|---|
| samples with a motor ≥ 0.995 | 32/2051 (1.6%) | 12/2434 (0.5%) |
| motor at idle floor while another > 0.2 | 55/2051 (2.7%) | 66/2434 (2.7%) |

### 3. The vertical estimate is unusable with motors running

Direct consequence of Finding 1:

| | file 1 | file 2 |
|---|---|---|
| `climb_rate` while thr > 0.25 | −9.66 … −0.86 m/s | −9.71 … −3.78 m/s |
| fused altitude drift, stationary | **3.72 m** | **3.39 m** |
| `accel_bias` range | −3.00 … +2.04 | −3.00 … +0.38 |

The bias state pushed to its ±3 m/s² clamp trying to cancel an error of roughly
−7. The clamp was sized at "~3× the measured bias" on the props-off figure of
−0.7; props-on makes that sizing meaningless.

### 4. The rangefinder mostly is not measuring

`VL53L0X` bring-up log, by device status (11 = `RANGECOMPLETE`, 8 = TCC fail):

| capture | status 11 (valid) | status 8 (fixed 20 mm sentinel) | other |
|---|---|---|---|
| file 1 | 6 | **24** | 1 × status 9 @ 8190 mm |
| file 2 | 5 | **40** | — |

The 20 mm value is a constant emitted on failure, not a range — confirmed
previously by observing it while the craft was at 1.65–2.08 m. It is currently
filtered only because it happens to fall below the 30 mm value gate in
`vl53l0x.c:100`; the driver's own comment asks for a status whitelist "once a
bench run shows what this unit actually reports." These two runs are that bench
run.

---

## Worked as designed

### 5. The accel health veto held

| | file 1 | file 2 |
|---|---|---|
| `accel_unhealthy` samples | 173/802 | 131/964 (first at t = 89.4 s) |
| height mode requested | `HOLD` | `HOLD` |
| height mode engaged | **never** | **never** |

First real exercise of the guard added in `99234e0`, and it did exactly its job:
saturated bias → latch → veto → collective handed back through the normal `OFF`
path. Without it, `HOLD` would have been flying on a −9.7 m/s phantom descent.

### 6. The control path lost nothing

```
 id name                 peak/cap   drops
  2 IMU_CONTROL             2/9         0
  6 ATTITUDE_CONTROL        2/9         0
  8 RC_CONTROL              1/4         0
  1 IMU_TELEMETRY           9/9     65535   (saturated counter)
  5 ATTITUDE_TELEMETRY      9/9     19688 / 15190
  7 RC_TELEMETRY            4/4      7918 / 3471
  9 TELEMETRY               8/8     65535   (saturated counter)
```

Every dropping FIFO is a **telemetry** queue — producers at 1 kHz feeding queues
drained at 50 Hz, lossy by design so telemetry cannot backpressure control.
**Every control queue is clean at ~22% occupancy.** Whatever else went wrong,
no control data was lost.

### 7. FC internals healthy throughout

`tx_overflow 0`, `imu_drop 0`, `log_wrap 0`, `heap_oom 0`. Heap peak
27,624 / 40,960 B (67%), allocations 70 with 2 frees — init-time only, no leak,
identical across both files. Worst task stack **812/1216 (67%)**, id 16
(`perf_telemetry`); `rate_ctl` well inside its margin. No resource pressure
anywhere.

---

## Instrumentation gaps

These did not cause the failures, but they materially slowed diagnosing them.

### 8. The attitude body-rate fields are dead

`ATTITUDE.rollspeed` / `pitchspeed` / `yawspeed` are **zero in 0 of 2440
samples** — never populated. For a vibration fault this is the single worst
field to be missing: the gyro story had to be reconstructed from `IMU_RAW` at
1.7 Hz (12 usable samples) instead of read at 50 Hz.

What those 12 samples showed, motors running:

```
gyro sd   x 38.96   y 26.40   z 7.32  deg/s
extremes  −84.49 … +63.03 deg/s
```

on a craft barely moving — the rate loop's input, and the reason it commands
differential that saturates the mixer.

### 9. `PerfFifo.drops` saturates, `cpu_load` is dead

`drops` is u16 and two FIFOs are pegged at 65535, so the counter cannot express
a rate. `SystemHealth.cpu_load` reads 0 in every sample of both files.

### 10. The FFT notch was compiled out

`NOTCH_STATUS.enabled = 0` in every sample (39 and 52) — but the deeper cause is
that the flown build had **`VAYU_FFT_NOTCH=OFF`**, which stubs `gyro_notch` to
passthrough and lets `--gc-sections` drop the FFT entirely. The notch could not
have been enabled over the link; it was not in the binary. Corrected by the
`VAYU_GYRO_NOTCH_FORCE_ON` build flashed 2026-09-07 23:41.

---

## Other observations

- Vertical estimator update: **782 µs peak at 250 Hz ≈ 19.6% CPU** (file 2;
  756 µs / file 1), mean 553–581 µs. Substantial for one estimator on an F401.
- `ipc_timeouts` 18,404 / 14,357 with ~670 k / 521 k blocked takes. No baseline
  to compare against — flagged, not diagnosed.
- Attitude reported ±0.7° roll and ±0.4° pitch while motors ran, i.e. the
  estimator believed it was level. With the accel destroyed, that number is
  carried by the gyro alone and should not be trusted over a long flight.
