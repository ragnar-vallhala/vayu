# Vertical velocity is corrupted once the motors run

## The observation

Hand-lift bench test, 2026-09-05 (`~/vayu-logs/export-20260905-235356.bin`). The
craft was raised by hand from 0.07 m to 2.10 m over ~5 s:

| t | agl_tof | baro_alt | fused_alt | climb_rate | vertical_accel |
|---|---|---|---|---|---|
| 9.0 | 0.07 | 457.29 | 457.28 | +0.04 | -0.52 |
| 12.5 | 1.18 | 458.39 | 457.84 | **-0.83** | -0.57 |
| 15.2 | 2.10 | 459.24 | 458.69 | **-0.86** | -0.48 |

**Both height sources agree** the craft rose ~2 m (ToF +2.03, baro +1.95), and
the fused altitude rises with them. But `climb_rate` reads **-1.0 m/s throughout
a genuine +0.7 m/s climb** — wrong sign, not merely noisy.

The altitude state survives only because the baro correction keeps dragging it
back: the innovation runs at +0.55 m, which pushes climb UP at
`k_vel x 0.55 x 15 Hz ~= +0.8 m/s per second`. For the state to sit at -1.0
anyway, the accel path must be pushing down about as hard — and
`vertical_accel` is indeed ~-0.7 sustained.

Measured `|accel|` from the same log:

| condition | mean | vs 9.807 | sd |
|---|---|---|---|
| motors idle | 9.609 | -0.197 | 0.04 g |
| motors running | **8.802** | **-1.004** | 0.10 g |

`vertical_accel = -(a_world_z + g)`, so an 8.8 reading where 9.807 is expected
produces exactly -1.0 m/s^2 of phantom downward acceleration. The arithmetic
closes on the observed symptom.

**At rest the accelerometer is fine** (`vertical_accel` mean +0.06 m/s^2 over the
disarmed window), so this is NOT a calibration error. It appears with motor
power.

## Why it matters

`climb_rate` is the height controller's inner-loop feedback
(`height_controller.c`, `climb_rate_pi`). A false -1 m/s reads as *falling*:

```
err = climb_sp - climb_rate = climb_sp + 1.0
  -> +0.12 collective via HEIGHT_KP_VZ
  -> up to +0.25 more as the integrator winds
```

Against a hover of 0.38 that is roughly **double thrust demanded from a
stationary craft**, and it is self-reinforcing: more throttle -> more vibration
-> more negative bias -> more throttle. That is a complete flyaway mechanism and
it explains the 2026-09-04 ceiling climb better than the hover guess does (see
[`altitude-hold-and-in-air-plan.md`](altitude-hold-and-in-air-plan.md)).

It also explains the bench observation that started this: collective *rose* as
the craft was lifted, when both altitude sources agreed it was going up.

**The height mode must not fly until this is addressed.** With props fitted the
vibration is worse, so the corruption is worst exactly when it matters. No gain
tuning helps — the feedback signal itself is wrong.

## What is NOT yet established

Why the magnitude drops. `BMX_ACC_RANGE` is **8 g** (`variables.h:56`) against
~0.1 g of measured noise, so simple ADC clipping is ruled out. Aliasing of
above-Nyquist content, or rectification in the sensor's internal path, are the
leading candidates but are UNVERIFIED. The `|accel|` figures come from `ImuRaw`
at ~1.7 Hz — 12 and 27 samples — which is thin, and the craft was being moved by
hand during the "running" window, which injects real accelerations.

**First step is therefore to measure it properly**, not to start fixing: log
`|accel|` at high rate, craft clamped and stationary, motors stepped 0 -> 0.2 ->
0.4 -> 0.6, props off and then on. If mean `|accel|` falls with throttle on a
stationary craft, the effect is real and quantified; if it does not, the
hand-motion confounds this whole analysis and the diagnosis needs redoing.

---

## What the references do

### PX4 — detect it from the innovations, not from the cause

`ekf2/EKF/height_control.cpp:124` `checkVerticalAccelerationHealth()`, whose
comment describes our failure exactly:

> *"Check for IMU accelerometer vibration induced clipping as evidenced by the
> vertical innovations being positive and not stale. Clipping usually causes the
> average accel reading to move towards zero which makes the INS think it is
> falling and produces positive vertical innovations."*

Average accel toward zero: 9.807 -> 8.802. INS thinks it is falling: -1.0 m/s.

Two independent legs:

1. **Clip counter**, per axis, incremented on hardware clip flags and decayed
   otherwise; warning at >50% of samples in 1 s, critical at 100%.
2. **`estimateInertialNavFallingLikelihood()`** (`:181`) — the part that needs no
   clip flags. For each height source it forms
   `innov_ratio = innov / sqrt(innov_var)` and tests it against
   `vert_innov_test_min` / `vert_innov_test_lim`. Note the test is **one-sided**
   (`innov_ratio > threshold`): vibration makes the INS think it is falling,
   which is a *positive* innovation, so direction alone carries information.
   ONE source failing the limit -> MEDIUM. TWO sources **of different reference
   type** (PRESSURE / GNSS / GROUND) failing -> HIGH.

Combined: `bad_vert_accel = (clipping frequently && MEDIUM) || HIGH` — so it can
declare the fault on innovations alone. The declaration then persists for
`BADACC_PROBATION` so it cannot chatter. Separately,
`estimator_interface.cpp:536` simply refuses accel samples that are clipping.

### ArduPilot — measure the vibration and expose it

`AP_InertialSensor::calc_vibration_and_clipping()` (`:2249`):

- clip count when any axis exceeds the backend's clip limit;
- a vibration metric: filter accel at 5 Hz, subtract to get the AC part, square
  it, filter at 2 Hz (`ACCEL_VIBE_FLOOR_FILT_HZ 5.0`, `ACCEL_VIBE_FILT_HZ 2.0`).

That is the `VIBE` figure in its logs, and it is what lets a pilot see that the
airframe needs soft-mounting rather than guessing.

---

## Fix order

0. **Measure it properly** (above). Everything below is contingent on the effect
   being real and reproducible on a clamped, stationary craft.
1. **Port PX4's innovation test.** vayu already computes the exact signal it
   needs: `vert_est_correct()`'s innovation is `baro_alt - ve.altitude`. And we
   now have TWO sources of different reference type — baro (PRESSURE) and the
   ToF AGL (GROUND) — which is precisely what PX4's HIGH-confidence case wants.
   This defends against the failure **without needing to know its cause**, which
   is the strongest argument for doing it first.
2. **Respond to the declaration**: stop integrating accel into the velocity
   state while `bad_vert_accel` holds, and lean on the height sources for
   velocity. Persist it with a probation period.
3. **Add ArduPilot's vibe metric** (5 Hz floor, diff, square, 2 Hz) to telemetry.
   Cheap, and it turns "the airframe vibrates" from a hypothesis into a number.
4. **Soft-mount the FC.** The hardware half; reduces the input rather than
   compensating for it.
5. **Rangefinder-derived climb rate** near the ground as a cross-check. The ToF
   tracked this lift correctly while the accel path was lying, and
   differentiating it is noisy but *unbiased* — which is the property that
   matters here.

Note the ordering deliberately puts the detector before any filtering: a DC bias
from rectification survives a low-pass, so filtering the input is not obviously
sufficient, whereas the innovation test catches the symptom whatever its origin.
