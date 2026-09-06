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

**The estimator's own header predicted this number.** It documented that a
2-state filter leaves a steady-state velocity offset of `(k_alt/k_vel)*b` under
a constant accel bias `b`, and `test_vertical_est.c` VERT-006 asserted it. With
b = -1.0 m/s^2 that is **-1.47 m/s**; the craft was genuinely rising ~0.5 m/s,
giving the -1.0 observed. The predicted steady innovation, `-b*T/k_vel`, is
0.625 m against 0.55 measured. So this was not a mystery in the filter — it was
a documented limitation meeting a bias large enough to matter.

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

**Step 1 is DONE** (commit below). The rest is unchanged.

### 1. Third state: estimate the bias and subtract it — SHIPPED

The header named the fix while documenting the limitation: *"a 2-state filter
cannot fully separate a DC accel bias from true velocity (that needs a 3rd bias
state)"*. So the estimator now has one.

```
predict:  a_corr = a_up - accel_bias           /* subtract the DC error */
correct:  accel_bias -= k_bias * (baro - alt)  /* learn it from the innovation */
```

That innovation is the same signal PX4's `checkVerticalAccelerationHealth()`
watches — used here to CANCEL the error rather than merely to flag it.

The gains turn out to be nearly free. A critically-damped 3rd-order
complementary filter with poles at -w and gains applied at baro rate f wants
K1 = 3w, K2 = 3w^2, K3 = w^3. The shipped `k_alt = 0.15` at f ~ 16 Hz is
w ~ 0.8 rad/s, which asks for `k_vel = 0.12` (shipped: 0.10) and
`k_bias = 0.032` (chosen: 0.03). **The third state completes the design the
first two already implied.**

Verified in `test_vertical_est.c` (25 checks): with b = -1 m/s^2 the bias state
learns -1.0000 and `climb_rate` converges to zero, where the 2-state filter gave
-1.4688. It settles in 7.6 s, does not absorb a real 1 m/s steady climb, and the
extra ringing on an unphysical 5 m baro step is bounded and measured rather than
assumed away (peak 2.87 m/s vs 2.44, settles by 12 s).

Cost, before the rangefinder aiding below: **~7 s to converge**, and the bias
appears at throttle-up, so lift-off happened with the error at full strength and
the correction at zero. Measured, with the bias stepping in at t=0:

| t (s) | climb_rate, 3-state | climb_rate, 2-state |
|---|---|---|
| 1.0 | -0.83 | -0.85 |
| 2.0 | -1.14 | -1.25 |
| 4.0 | -0.84 | -1.47 |
| 7.0 | -0.17 | -1.49 |
| 9.0 | +0.00 | -1.50 |

For the first ~2.5 s the third state buys almost nothing. That is structural, not
a gain choice: the bias is learned from a POSITION innovation, so it cannot be
seen until it has been integrated twice — once into a velocity error, again into
a position error large enough to show above baro noise. `climb_rate` is wrong
immediately, because it sits downstream of the same accel with none of that lag.

Residual: **the bias is not truly constant** — it tracks throttle. The state
follows slow changes, not fast transients.

### 1b. Rangefinder bias aiding — SHIPPED

The fix for that window is to observe the bias one integration earlier. A
rangefinder differences into a VELOCITY measurement, which reveals an accel bias
after ONE integration instead of two — and the VL53L0X is in range below ~1.5 m,
which is exactly the takeoff band.

`vert_est_correct_tof()` differences consecutive in-range samples and drives
**the bias state only**. It deliberately touches neither altitude nor
climb_rate:

- **altitude** — the ToF is an AGL reference over whatever is directly below and
  steps when the ground does, while the filter tracks a baro reference. Folding
  one into the other makes the fused altitude jump on every terrain step. This
  preserves the decision already recorded in `vertical_task.c`.
- **climb_rate** — a differentiated ToF carries ~0.2-0.3 m/s of noise, and
  climb_rate IS the height controller's inner loop. Routing through the bias
  integrator low-passes that instead of handing it to the controller.

Gated on an implausible one-sample velocity (>3 m/s), an over-long gap (>0.25 s),
and the same validity test that qualifies `agl_tof` for publication. A rejected
sample also breaks the history, so the next pair is not differenced across the
hole.

Gain swept in simulation (1 m/s^2 bias, 0.3 m baro noise, ToF at 21 Hz):

| k_tof | settle | peak abs climb | steady RMS |
|---|---|---|---|
| 0 (baro only) | 7.10 s | 1.23 | 0.057 |
| 0.02 | 3.58 s | 0.92 | 0.033 |
| 0.05 | 2.35 s | 0.72 | 0.033 |
| **0.10** | **1.76 s** | **0.63** | **0.037** |
| 0.20 | 1.24 s | 0.50 | 0.027 |

0.10 halves the peak excursion and cuts convergence from ~7 s to ~1.8 s while
holding up at 30 mm of ToF noise. 0.20 is faster still but leaves less margin for
un-modelled rangefinder behaviour (multipath over grass, tilt error), so it is
the obvious knob if the bench run says the sensor is cleaner than assumed.

Crucially, this does NOT confuse real motion with bias the way a zero-velocity
update would: during a genuine climb the ToF velocity is truth, so the innovation
goes to zero on its own. VERT-009 asserts exactly that, plus rejection of a 40 cm
terrain step and that the aiding moves nothing but `accel_bias`.

### 2. Health flag + veto — SHIPPED

`accel_bias` is clamped to +/-3 m/s^2 (~3x the measured bias). Saturation means
the mismatch exceeds what the filter can absorb, so `accel_unhealthy` latches,
clearing only after 80 clean corrections (~5 s) — PX4's `BADACC_PROBATION` idea,
so it cannot chatter at the clamp and strobe the height mode.

`angle_controller` now vetoes the height mode on it, handing the collective back
through the normal OFF path with its throttle re-sync. The height controller's
inner loop IS `climb_rate`, so a corrupted one does not degrade the mode, it
inverts it.

### 3. Telemetry — SHIPPED

`VERTICAL_STATE` (1040) gains `accel_bias` and `accel_unhealthy` as **extension**
fields, so they are excluded from CRC_EXTRA and old peers are unaffected
(navlink/ABI.md sec 2 rule 2).

`accel_bias` is worth watching in its own right: it is the vibration read-out, in
the units that matter, and it makes the "measure it properly" step below a matter
of reading one number rather than post-processing a log.

### 4. Still to do

- **A zero-velocity update on the ground** is still available if 1.8 s is too
  slow, but it is now the weaker option: a ZUPT learns the idle-throttle,
  resting-on-its-feet bias, which is not the hover bias, whereas the rangefinder
  measures the real thing at the real throttle.
- **Measure it properly.** Clamped, STATIONARY craft (see the caveat below —
  motion contaminates the reading), throttle stepped 0 -> 0.2 -> 0.4 -> 0.6,
  props off then on, watching `accel_bias` settle at each step. The 2026-09-06
  run establishes the effect is real and motor-driven; what it does NOT give is
  a clean bias-vs-throttle curve, which is what sizes the clamp and says whether
  a ground zero-velocity update would have been worth it.
- **ArduPilot's vibe metric** (5 Hz floor filter, subtract, square, 2 Hz filter)
  for the AC part. `accel_bias` captures the DC rectification; this captures the
  vibration itself, which is what tells you the airframe needs soft-mounting.
- **Soft-mount the FC.** Reduces the input rather than compensating for it. Still
  the only fix that addresses the cause.
- **Rangefinder-derived climb rate** near the ground as an independent check. The
  ToF tracked the lift correctly while the accel path was lying; differentiating
  it is noisy but *unbiased*, which is the property that matters here.

## Hardware result, 2026-09-06

Flashed to the real FC and run through a full hand-lift sequence: arm, HOLD,
raise by hand to ~2 m, LAND, lower, disarm. Recorded with
`tools/telemetry/handlift_record.py`
(`~/vayu-logs/handlift-20260906-175648.bin`).

**The phantom descent is gone.** Same manoeuvre, before and after:

| | 2026-09-05 | 2026-09-06 |
|---|---|---|
| craft raised ~2 m by hand | climb_rate **-0.83 .. -0.86** | climb_rate **+0.66 .. +1.08** |

Correct sign, and the magnitude matches the lift (0.18 -> 2.06 m in ~3 s).

**The vibration bias is real and motor-driven** — measured directly for the
first time, rather than inferred from 27 samples of raw accel:

| phase | collective | accel_bias |
|---|---|---|
| disarmed, motors off | 0.01 | +0.05 |
| armed, idle floor | 0.15 | -0.16 |
| HOLD engaged, on ground | 0.49 | -0.25 |
| hand lift, climbing | 0.29 | **-0.57 (min -0.71)** |
| held at ~2 m | 0.26 | -0.22 |
| landed, disarmed | 0.00 | +0.04 |

It appears with motor power and returns to ~0 without it. `accel_unhealthy`
never tripped; max |bias| was 0.71 against the 3.0 clamp, so the authority is
generous — leave it until a second airframe says otherwise.

The rest of the sequence behaved: collective backed off as the craft was raised
(0.51 on the ground -> 0.25 at 2 m), `LANDED` latched at 0.115 m, disarm clean.

### Two caveats from this run

**`accel_bias` is only a clean vibration read-out when the craft is steady.** At
2 m the collective was 0.26 with bias -0.22; during the lift it was 0.29 with
bias -0.57 — nearly the same throttle, very different bias. The likely cause is
that the ToF aiding was active below 1.5 m and drove the bias hard to make
`climb_rate` catch up with a real hand acceleration, then relaxed once the ToF
dropped out. The aiding did its job, but it can only reach `climb_rate` THROUGH
the bias, so during fast genuine motion it transiently over-drives it. Harmless
here (climb_rate came out more correct, not less), but it means the bench
measurement must be done clamped and stationary to get the real number.

**The aiding is verified only over its own band.** The ToF was valid for 34/99
samples in the interesting window, because most of it was spent above the 1.5 m
ceiling; the ~2 m hold ran on baro alone. So the 0-1.5 m path is exercised and
works, and nothing above it is.

## Status

The estimator no longer converts this vibration into a phantom descent — now
confirmed on hardware, not just in host tests. The rangefinder pulls convergence
down to ~1.8 s so the takeoff window is covered, and the height mode refuses to
run when the error exceeds what it can cancel.

**Still not flown.** Everything so far is bench and hand-held with props off. The
open items are the clamped throttle-step measurement (which now reads one
telemetry field), the vibe metric, and soft-mounting.
