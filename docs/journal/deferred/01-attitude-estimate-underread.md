# Deferred issue #1 — attitude estimate under-reads in powered, translating flight

## The issue

During free flight the firmware attitude estimate **under-reads the true tilt by
~10–13×**: the craft banks/pitches to 15–20° but the EKF reports only 1–2°. It is
NOT a dt, gyro, or integration bug (those were instrumented and ruled out — see
below). The root cause is the **accel/tilt ambiguity in powered flight**:

- A multirotor's accelerometer does not measure gravity. It measures specific
  force `f_body = -(T/m)·e₃ − C·v_body + noise` (thrust along body −Z, plus rotor
  drag proportional to body-horizontal velocity).
- During a maneuver the thrust-aligned term dominates and `|a| ≈ g`, so it passes
  the EKF's magnitude gate (`ekf.c:259`) and *reads as "level."*
- Every step, `ekf_update_accel` then yanks the attitude back toward level and
  dumps the residual into the gyro-bias state — so the estimate stays pinned near
  zero and the bias winds up and oscillates with the maneuver.

This is shared by all three fusion filters (EKF/Mahony/complementary) because they
all fuse the same thrust-aligned accel. On a tuning rig (translation pinned) the
accel sees clean gravity, so the estimate tracks truth — confirming the cause is
translation, not the filter math.

Secondary contributor: the estimator state (gyro bias, covariance, Mahony
integral) is **never reset on arm**. `estimator_reset()` is called only in tests;
the STANDBY→ARMED hard reset (`angle_rate_controller.c:254`) resets only the rate
PID and gyro LPF. A bias wound up in one flight persists into the next arm.

## Short-run fix (ships without new infrastructure)

1. **Dynamics-aware accel gate.** Gate/down-weight the accel update on *dynamics*,
   not just `|a| ≈ g`: skip or de-weight it when `|ω|` (or throttle / known
   thrust) indicates the craft is maneuvering. Cheap (a comparison). Stops the
   pull-to-level. Cost: loses the tilt reference during sustained banks, so the
   estimate free-runs on gyro and drifts slowly — acceptable as a stopgap.
2. **Reset estimator on arm + bound the bias.** Call `estimator_reset()` on the
   STANDBY→ARMED transition, and clamp the EKF gyro-bias state (the Mahony `Ki`
   integral already clamps at ±0.5 rad/s). Removes the cross-arm carryover.

Both are F4-cheap and add no per-cycle loop cost.

## Long-run fix (resolves the ambiguity properly)

**Model-aided (rotor-drag) prediction.** Stop treating the accel as a
gravity-direction reference and model the real specific force. The multirotor
accel is effectively a *body-velocity* sensor via rotor drag, so:

- Extend the MEKF 6 → 8 states: attitude + gyro bias + **2 body-horizontal
  velocities**.
- Propagate velocity with the thrust + drag + gravity model (needs the **thrust
  command** plumbed to the estimator — it exists in the controller today but the
  EKF never sees it).
- Use the accel's horizontal channels as a velocity measurement; the residual
  after removing modeled thrust+drag becomes a valid tilt correction.

This keeps the accel *usable* during dynamic flight instead of rejecting it, and
costs only ~2 extra states + a few flops on the F4 (the Martin & Salaün / Leishman
"improved dynamic model" approach). Needs no new sensors.

**Prerequisites / caveats:**

- **vsim fidelity gate (blocking).** vsim models drag as isotropic world-frame
  `a -= (vel_w − wind)·(linear_drag/mass)` (`sim/vsim/src/physics_core.cpp:32`),
  not body-frame rotor drag. With small `linear_drag` the sim accel is ≈ pure
  thrust and reads level regardless of tilt — so a model-aided filter cannot be
  developed or validated in SITL until vsim has a proper body-frame thrust +
  rotor-drag model. (Building/validating against the current model would repeat
  the SITL-fidelity violation this whole investigation exists to fix.)
- **Identify `C` (drag) and the thrust map** per airframe — the existing
  system-ID rig tools (`step`/`chirp`/`doublet`) are the right instrument.
- **Limitation:** rotor-drag coefficients are small, so the horizontal signal is
  weak at low speed (a few degrees-equivalent near hover). Model-aiding is strong
  at speed but not rock-solid at hover; full velocity aiding (GPS/optic-flow,
  a separate nav-EKF subsystem this firmware does not yet have) is the eventual
  ceiling.

## How to reproduce

The diagnostic was a throwaway shim and is NOT in the tree (it was removed to keep
the FC code clean). To reproduce, re-add a `VAYU_SITL`-guarded log in
`attitude_task.c` at the predict call (after the filter dispatch, ~line 132),
printing per estimator step: `step_dt`, `|gyro|` = `sqrtf(gx²+gy²+gz²)`, the est
attitude (`ori.roll/pitch/yaw`), and the accumulating bias —
`ekf_get_gyro_bias()` for the EKF (rad/s → ×57.2958 for deg/s) or the Mahony
integral. The supporting Mahony getter (`estimator_get_mahony_integral`, declared
in `est.h`, defined in `sensor_fusion.c`) was likewise removed; add it only if you
need the Mahony path. Guard everything on `VAYU_SITL` so it stays out of the
firmware. SITL routes the FC's stderr to `/tmp/sitl.err` when `SITL_LAB_DEBUG=1`.

```sh
# 1. Build SITL (after re-adding the diagnostic)
cd sim/host && cmake --build build_sitl --target vayu_sitl

# 2. Healthy case (rig, translation pinned) — estimate TRACKS truth
cd ../../navigator/headless-sdk
rm -f /tmp/sitl.err
SITL_LAB_DEBUG=1 PYTHONPATH="$PWD:$PWD/examples" \
  python3 examples/step_response.py --axis pitch --amp 0.4 --csv /tmp/step.csv
grep ATT_DIAG /tmp/sitl.err     # est pitch ≈ true pitch (−6.4° vs −6.3°), bias flat ~0.06°/s

# 3. Broken case (free flight) — estimate PINNED near level, bias winds up
rm -f /tmp/sitl.err
SITL_LAB_DEBUG=1 PYTHONPATH="$PWD:$PWD/examples" \
  python3 examples/figure8.py --size 6 --laps 1 --lap-secs 16 --csv /tmp/fig8.csv
grep ATT_DIAG /tmp/sitl.err     # |gyro| 8–13°/s but est att stays ±1–2°, bias swings ±1.5°/s
```

**The signature:** `step_dt ≈ 0.008 s` and `|gyro|` matching truth in both cases
(input is correct), but in free flight the est attitude stays ±1–2° while the body
rate is 8–13°/s and the gyro bias winds to ±1.5°/s and sign-flips with each lobe —
the accel update fighting the gyro.

Full investigation log: `navigator/headless-sdk/docs/FINDINGS.md` (§H + UPDATE pt 3).
