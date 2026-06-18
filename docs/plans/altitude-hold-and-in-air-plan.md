# Vayu firmware — altitude hold, vertical estimation, and baro-driven IN_AIR

## Context

The BME280 barometer is now live end-to-end: a fidelity-correct driver on real
hardware (rides the IMU's single-owner I2C DMA loop), a modelled-pressure path in
SITL (`vsim_d` emits pressure → `host_baro` → the real `bme280_publish`), a
`BARO` NavLink message (msgid 1039), and a dual-axis MSL/AGL chart in the GCS.

So far the baro is **observed but unused** — `bme280_read_all()` returns
`{pressure, temperature, humidity, altitude}` and we draw it. This plan covers
turning it into **control input**: an `ALT_HOLD` flight mode, the vertical state
estimate it requires, and — separately and first in value — using the baro to
finally drive the `SYSTEM_STATE_IN_AIR` transition, which we could never do
before.

This is a design plan only. No code is written here. File:line references are to
the live tree at the time of writing and are pointers, not contracts.

---

## 1. What the barometer brings

Before the baro, the FC had **no observability of the vertical world axis**. The
estimator (`src/est/ekf.c`, `src/est/sensor_fusion.c`, `attitude_task`) is
attitude-only: it tracks the orientation quaternion and nothing about height or
vertical motion. Throttle is raw RC passthrough (`angle_controller.c:204`,
`:277`). That ceiling blocked three things, now unblocked:

1. **Absolute altitude** (MSL) and, against a captured ground reference, **AGL**.
2. **Vertical velocity (climb rate)** — *derivable* once baro is fused with
   accel-Z (raw baro alone is too noisy/laggy to differentiate; see §3).
3. **Airborne detection** — distinguishing "armed on the ground" from "flying",
   which needs altitude + climb rate (see §5).

Downstream, these enable: altitude hold, a real `IN_AIR` state, controlled
auto-descent on failsafe, landing/touchdown detection, takeoff detection for
integrator/gain scheduling, ground-effect awareness, and richer logging. This
plan scopes the first two consumers (`ALT_HOLD`, `IN_AIR`); the rest are noted as
follow-ons in §7.

---

## 2. Flight-mode change: `ch6` becomes a 3-position switch

### Current behaviour (verified)
`ch6` (`ACRO_SWITCH_CH = 5`, 0-based; `variables.h:237`) is a **2-position**
toggle read against a single threshold `ACRO_SWITCH_US = 1500`
(`variables.h:238`):

- `angle_controller.c:212-214` reads the channel and calls
  `flight_mode_resolve_acro(rc_acro)` (`src/control/flight_mode.c:21`).
- `> 1500 µs` → `FLIGHT_MODE_ACRO` (rate mode, bank-angle failsafe suppressed);
  else `FLIGHT_MODE_ANGLE` (stabilise, `MAX_ANGLE_CUTOFF = 70°` failsafe armed,
  `angle_controller.c:239`).
- GCS `CMD_SET_FLIGHT_MODE` can override RC; arbitration + the resolved mode are
  in `flight_mode.c` and published as `FLIGHT_MODE` telemetry.

### New behaviour
`ch6` becomes a **3-position** switch mapping the SwC detents:

| `ch6` µs (nominal) | Mode        | Attitude behaviour | Throttle source |
|--------------------|-------------|--------------------|-----------------|
| ~1000              | `ALT_HOLD`  | angle/stabilise    | altitude controller (§4) |
| ~1500              | `STABILISE` | angle/stabilise    | raw RC (`ch2`)  |
| ~2000              | `ACRO`      | body-rate          | raw RC (`ch2`)  |

Read with two thresholds + hysteresis (e.g. `< 1250` → ALT_HOLD, `1250–1750` →
STABILISE, `> 1750` → ACRO; ~50 µs hysteresis band around each edge to stop
detent jitter from flapping modes). The thresholds become named constants
alongside `ACRO_SWITCH_*` in `variables.h`.

**Key insight:** `ALT_HOLD` is *stabilise attitude + altitude-driven throttle*.
The roll/pitch/yaw path is byte-identical to `STABILISE` (same angle loop, same
`MAX_ANGLE_CUTOFF` bank failsafe). Only the throttle source changes. This keeps
the change surgical — the attitude controller is untouched; we swap what feeds
`angle_controller_outputs.throttle`.

### Touch list (no code here, just scope)
- `variables.h` — 3-position thresholds + hysteresis constants; keep
  `ACRO_SWITCH_*` or rename to `MODE_SWITCH_*`.
- `include/control/flight_mode.h` / `flight_mode.c` — add `FLIGHT_MODE_ALT_HOLD`;
  replace `flight_mode_resolve_acro(bool)` with a 3-way resolve that takes the
  raw `ch6` µs (or a decoded enum) and arbitrates with the GCS override. The
  `*_acro` getter stays (the rate controller only needs the acro bit); add an
  `is_alt_hold` query for the throttle path.
- `navlink/dialect.json` — extend the `flight_mode` enum (`ANGLE=0, ACRO=1,
  RELEASE_TO_RC=2`) with `ALT_HOLD`; regenerate. GCS mode pill + the
  `CMD_SET_FLIGHT_MODE` override gain the new value automatically once the enum
  carries it. (Wire-compatible: appending an enum value is non-breaking.)
- `angle_controller.c` — the throttle decision (see §4).

---

## 3. The vertical estimator (prerequisite for §4 and strengthens §5)

`ALT_HOLD` cannot use raw baro altitude directly: at ~10–25 Hz with ±0.5–2 m
noise, differentiating it for climb rate yields unusable velocity. Every
autopilot that holds altitude (ArduPilot `EKF3` + `AC_PosControl`, PX4 `EKF2`,
iNav NAV) first fuses baro with the accelerometer into a vertical state. We do
the minimal version of that: a **2-state filter** `[altitude_agl, climb_rate]`.

### Inputs (all already available)
- **Body specific force** `a_body` from the IMU (`bmx160_read_acc_mps2` /
  `imu_queue_*`).
- **Attitude quaternion** `q` from the estimator (`attitude_queue_*`), to rotate
  `a_body` into the world frame.
- **Baro altitude** from `bme280_read_all()` (MSL; converted to AGL against a
  ground reference, §4/§5).

### Model
- **Predict** (at IMU/attitude rate, fast): compute world-vertical inertial
  acceleration `a_z_world = (R(q)·a_body) + g` in NED (down-positive; mind the
  estimator's gravity sign — `ekf.c` notes a level board reads ≈ `−g` on its
  vertical axis; see `docs/reference/coordinate_ref.md`). Integrate `a_z_world` into
  `climb_rate`, integrate `climb_rate` into `altitude`.
- **Correct** (at baro rate, slow, latest-wins): nudge `altitude` toward the baro
  reading and let the cross-term correct accel bias — i.e. a complementary
  filter / steady-state 2-state Kalman. Two gains (position, velocity) or a fixed
  Kalman gain pair; no full covariance propagation needed for a 2-state.

This bounds accel drift with baro's absolute (but noisy) altitude and gives a
low-lag climb rate that baro alone can't.

### Where it lives
A new `src/est/vertical_estimator.c` (`VERT` module), run either inside
`attitude_task` (it already has `q` + the IMU cadence) or as a sibling task at
the same rate. It publishes `{altitude_agl, climb_rate, vertical_accel}` into a
small SPSC ring (like `attitude_queue`) for the control loop to drain.

### Make it observable first
Add the fused `altitude`/`climb_rate` to telemetry (extend `BARO`, or a new
`VERTICAL_STATE` message) so the GCS chart can show **fused vs raw** side by
side. This is the cheapest way to validate the estimator in SITL before any
control loop trusts it — and SITL's altitude is exact ground truth, so the fused
estimate has something to be checked against.

---

## 4. The `ALT_HOLD` controller

A cascade that replaces the raw-RC throttle when `ALT_HOLD` is engaged:

```
target_alt ──(P)──▶ desired_climb_rate ──(PID)──▶ Δthrottle
throttle = hover_ff + Δthrottle ,  then ÷ cos(tilt)
```

- **Outer position P:** `alt_error = target_alt − est_alt → desired_climb_rate`,
  clamped to ±~2 m/s.
- **Inner velocity PID:** `climb_rate_error → Δthrottle`, around a **hover
  feedforward** (`hover_ff`, ~0.55 per the X3 note at
  `angle_rate_controller.c:191`). Gains live in `pid_config` so they're tunable
  live via `CMD_SET_PID` (reuse the existing axis/controller plumbing or add an
  `ALT` controller id).
- **Tilt compensation:** divide commanded throttle by `cos(tilt)` (tilt from `q`)
  so vertical thrust holds while banking.
- **RC throttle stick = climb-rate command:** centred stick (deadband around
  1500 µs) = hold; off-centre = commanded climb/descend. So `ch2` still does
  something intuitive in `ALT_HOLD` — it trims height instead of raw thrust.

### Engage / disengage (bumpless)
- Engage only when **armed and (ideally) `IN_AIR`** — see §5. On a craft sitting
  on the ground, `ALT_HOLD` must not spin up to hover; gate on `IN_AIR` or a
  min-throttle arm-from-ground condition.
- On engage: `target_alt ← est_alt`, and **seed the velocity-PID integrator with
  the current throttle** so the output doesn't jump (bumpless transfer). On
  disengage: hand back to raw RC throttle cleanly.

### Where it plugs in
`angle_controller.c:204` currently does `target_throttle = channels[2]`, written
to `angle_controller_outputs.throttle` at `:277`. The only change at the
controller boundary: when `flight_mode == ALT_HOLD`, `target_throttle` comes from
the altitude controller (fed by the §3 estimate + the stick-as-climb-rate map)
instead of `channels[2]`. The rate controller / mixer downstream
(`angle_rate_controller.c`) is unchanged.

### Failsafe interplay
- `MAX_ANGLE_CUTOFF` bank failsafe stays armed in `ALT_HOLD` (it's an angle
  mode).
- On RC loss while in `ALT_HOLD`, the natural failsafe is **commanded descent**
  (hold a gentle negative climb rate to land) rather than the current cut — a
  follow-on, but the architecture should not preclude it.

---

## 5. Baro-driven `IN_AIR` (the gap we couldn't close before)

### The gap (verified)
`SYSTEM_STATE_IN_AIR` (`state.h:12`) and its transitions
(`ARMED→IN_AIR`, `IN_AIR→ARMED`, `IN_AIR→STANDBY`; `state.c:43-46`) exist, and
three consumers already branch on it:
- `rc_safety.c:119` — treats `ARMED || IN_AIR` as "armed" for the RC/failsafe gate.
- `sensor_fusion.c:59` — same armed-or-in-air gate for estimator behaviour.
- `heartbeat.c:136` — maps it to the telemetry `nav_state` / GCS pill.

**But nothing ever calls `system_state_set(SYSTEM_STATE_IN_AIR)`** — confirmed by
grep across `src/`. The arm gate sets `ARMED` (`rc_task.c:67`, `:143`) and the
craft never leaves it. `IN_AIR` is dead. The reason is exactly the §1 ceiling:
without altitude/climb-rate there's no robust airborne signal (throttle alone is
ambiguous — high throttle on the bench, low throttle in a descent).

### How baro closes it
A small **takeoff/landing detector** (in the vertical-estimator or a tiny
`flight_phase` helper) drives the transition with hysteresis + debounce:

- **`ARMED → IN_AIR` (takeoff)** when, sustained for ~0.3–0.5 s:
  `est_agl > TAKEOFF_ALT` (e.g. 0.3–0.5 m) **AND** `climb_rate > TAKEOFF_RATE`
  (e.g. +0.3 m/s) **AND** `throttle > ~hover-ish`. Requiring altitude *and* climb
  rate *and* throttle avoids false trips from baro noise, prop-wash pressure, or
  a bench throttle blip.
- **`IN_AIR → ARMED` (touchdown, still armed)** when, sustained for ~0.5–1 s:
  `est_agl < LAND_ALT` (near the ground ref) **AND** `|climb_rate| < LAND_RATE`
  **AND** `throttle < hover` — i.e. settled on the ground. From `ARMED` the
  existing disarm path can then take it to `STANDBY`.
- Ground reference for AGL is captured while **disarmed/STANDBY** (continuously,
  to absorb baro drift) and frozen at arm — the same scheme the GCS AGL chart
  already uses, but now authoritative on the FC.

### What flips on once `IN_AIR` is real
- The three consumers above become meaningful (e.g. the armed-gate semantics,
  estimator gating, and the GCS pill finally show IN_AIR vs ARMED-on-ground).
- `ALT_HOLD` engage can gate on `IN_AIR` (§4) instead of a throttle heuristic.
- Enables follow-ons: integrator/gain scheduling at takeoff, auto-descend
  failsafe, touchdown auto-disarm, "armed too long on the ground" warnings.

### Note
`IN_AIR` detection is independent of `ALT_HOLD` and **lower-risk** — it only
*reads* the vertical estimate and sets a state flag; it changes no actuator
output. It's the natural **first** thing to ship after the estimator (see §6).

---

## 6. Phasing (each step verifiable in SITL before the next)

SITL now models the baro from exact ground truth, so the whole vertical loop can
be built and tuned in the simulator — a bad gain is a sim wobble, not a broken
airframe — then validated on the bench/flight.

1. **Vertical estimator (§3)** + telemetry of fused altitude/climb-rate. Verify
   in SITL: fused altitude tracks vsim ground truth; climb rate is clean during
   modelled climbs. No control change yet.
2. **Baro-driven `IN_AIR` (§5)** — lowest-risk consumer; read-only on actuators.
   Verify the takeoff/landing transitions fire correctly in a SITL flight and the
   GCS pill shows IN_AIR. (Ships value even before ALT_HOLD.)
3. **`ch6` 3-position + `FLIGHT_MODE_ALT_HOLD` plumbing (§2)** — mode selection,
   enum, telemetry, GCS pill. Still no throttle change (ALT_HOLD falls back to RC
   until step 4).
4. **`ALT_HOLD` controller (§4)** — wire the cascade into the throttle path; tune
   `hover_ff` + the position/velocity gains in SITL; add bumpless transfer.
5. **Hardware bring-up** — bench thrust-stand checks, then a tethered/low hover.

---

## 7. Open questions / decisions

- **Hover throttle:** constant feedforward (~0.55) to start, or learn it
  (low-passed steady-state throttle while `IN_AIR` and climb≈0)? Adaptive is a
  follow-on; constant is fine for first flight.
- **Estimator placement:** inside `attitude_task` vs a sibling task — affects
  cadence and queue plumbing. Lean toward inside `attitude_task` (it has `q` and
  the IMU rate already).
- **Telemetry shape:** extend `BARO` with fused fields vs a new `VERTICAL_STATE`
  message. A new message keeps `BARO` as the raw-sensor record (cleaner; mirrors
  IMU_RAW vs ATTITUDE_EULER).
- **AGL reference ownership:** the GCS computes its own AGL today; once the FC
  owns the authoritative ground reference (§5), the GCS should consume the FC's
  AGL to avoid two references disagreeing.
- **Failsafe-in-ALT_HOLD policy:** controlled descent vs current cut — decide
  before ALT_HOLD ships to flight.
- **`RELEASE_TO_RC` interaction:** how the existing `flight_mode` value relates to
  the new 3-position switch (is it still reachable, GCS-only?).

---

## 8. Summary of the seam

- **Sensor in, estimate, decide, actuate** stays clean: baro + accel + `q` →
  vertical estimate (§3) → {`IN_AIR` flag (§5), `ALT_HOLD` throttle (§4)}.
- **Manual modes are unchanged.** STABILISE and ACRO keep raw-RC throttle, exactly
  as today (and as Betaflight does). The vertical estimator and altitude loop are
  additive, engaged only by the `ch6` ALT_HOLD detent.
- **Smallest first, riskiest last:** estimator → IN_AIR (read-only) → mode
  plumbing → ALT_HOLD actuation, every step SITL-verified.
