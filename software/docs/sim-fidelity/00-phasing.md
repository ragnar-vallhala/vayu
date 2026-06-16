# Phased implementation plan — sim fidelity

Status: 🗺️ sequencing plan. This is the cross-cutting layer over the five
per-feature plans ([01](01-wind-turbulence.md)–[05](05-sensor-error-models.md)):
the order to build them, the shared foundation to lay once, the per-phase exit
gates, and the places where the **mockup** (`../ui-mockup/`) diverges from what
those five plans assumed. Read a feature's own plan for the physics/math detail;
read this for *when* and *in what order* to land it.

All code line references below were verified against the tree on 2026-06-17.

## Phasing principles

- **Each phase ships independently** behind its own `VSIM_CTL_SET_*` opcode,
  builds clean, and is testable on its own. No phase leaves the tree broken.
- **Lay shared seams once.** The pose-frame extension (below) and the seeded-RNG
  contract are touched by several features — build them in Phase 0 so later
  phases only *populate*, never re-break the wire.
- **Follow the existing pattern**, don't invent: new opcode in
  `vsim_proto.h` → `case` in the `main.cpp` dispatch (lines 223–422) →
  `send*()` in `SimWorker` mirroring `sendNoise` (`SimWorker.cpp:205`) →
  staged group in `SimulatorWidget` (see "Apply model" below).
- **Opcode value ≠ phase order.** Keep the opcode numbers the per-feature plans
  already reserved (table below) so the docs stay accurate even though we build
  them in a different order.

## Opcode allotment (fixed; matches the per-feature plans)

Highest currently used is `VSIM_CTL_SET_FAULTS = 13` (verified `vsim_proto.h:111–125`).
Body cap is 256 B (`vsim_ctl_frame_t.body`); every struct below is well under.

| Opcode | Value | Body struct | Phase |
|--------|-------|-------------|-------|
| `VSIM_CTL_SET_WIND` | 14 | `vsim_ctl_wind_t` (32 B) | 1 |
| `VSIM_CTL_SET_ATMOS` | 15 | `vsim_ctl_atmos_t` (16 B) | 2 |
| `VSIM_CTL_SET_MAGFIELD` | 16 | `vsim_ctl_magfield_t` (12 B) | 4 |
| `VSIM_CTL_SET_POWER` | 17 | `vsim_ctl_power_t` (32 B) | 5 |
| `VSIM_CTL_SET_SENSOR_ERR` | 18 | `vsim_ctl_sensor_err_t` (~64 B) | 3 |
| `VSIM_CTL_SENSOR_CSV` | 19 | chunked rows | 3 |

## Build order

**0 → 1 → 2 → 3 → 4 → 5.** Rationale for the deviations from the per-feature
README's `1→2→5→3→4`:

- **Phase 0 first** because #1/#2/#5 all extend the pose frame; doing it once
  avoids three wire breaks (see below).
- **Magnetic field (4) after sensor errors (3)**, not before: both rewrite the
  **same** mag-sensor synthesis path. Building the field against the *old* synth
  and then reworking it in Phase 3 is wasted work — land the rich pipeline first,
  then slot the field projection into it.
- **Power (5) last** — it is the least mockup-backed (readout-only there; see
  divergence D4) and depends on nothing else.

---

## Phase 0 — shared foundation (no user-visible behaviour)

A single ABI bump + plumbing so later phases just fill fields in.

**Why it's safe to do up front:** `vsim_pose_frame_t` is consumed *only* by the
daemon (`tools/vsim/`), `SimWorker`, and `world_mesh_transport_test.cpp` — **no
firmware**. The firmware IMU shim uses `/tmp/vsim_imu`, a separate format. So this
is a contained daemon↔GCS change, not a cross-stack break.

- **Proto** (`vsim_proto.h`): extend `vsim_pose_frame_t` (currently 16+92 B,
  locked by static_asserts at lines 289 & 298) with the fields the later phases
  publish, reserved now and zero until populated:
  ```c
  float wind_w[3];      // Phase 1  (instantaneous world wind, for the graphs)
  float airspeed;       // Phase 2  (‖v_rel‖)
  float ge_factor;      // Phase 2  (live ground-effect multiplier)
  float batt_voltage;   // Phase 5
  float batt_current;   // Phase 5
  float batt_mah_used;  // Phase 5
  float batt_soc;       // Phase 5
  ```
  Bump `VSIM_PROTO_VERSION 2u → 3u`; update **both** static_asserts to the new size.
- **Daemon** (`main.cpp`): zero-init the new fields when building each pose frame.
- **GCS** (`SimWorker`): extend `SimSnapshot` (`SimWorker.h:22`) + `emitFromFrame()`
  (`SimWorker.cpp:414`) to decode them.
- **RNG contract (confirmed):** the master seed enters via `vsim_ctl_reset_t.seed`
  → `SimController::seedSensors()` → `SensorModels::seed()` (private `std::mt19937_64`,
  `sensor_models.h:54`). Phases 1 and 3 need determinism too, but must **not** share
  that one RNG *instance* — adding wind/sensor-err draws to it would shift the
  existing accel/gyro/mag noise sequence and break autotune-repeatability baselines.
  Instead each phase owns an **independent** `mt19937_64` seeded deterministically
  from the same master seed (e.g. `seed ^ salt`). Phase 1 introduces master-seed
  storage on the controller when it has the first consumer — nothing to add in
  Phase 0 beyond this note.
- **Exit gate:** sim builds, runs, renders identically; new snapshot fields read 0;
  `world_mesh_transport_test` updated and green.

---

## Phase 1 — Wind & turbulence  ([01](01-wind-turbulence.md))

The headline physics change; everything else reuses its `v_rel`.

- **Proto:** `SET_WIND = 14`, `vsim_ctl_wind_t`.
- **Daemon:** `WindState` on the controller (config + `v_turb[3]` filter state +
  sim-time accumulator), `Vec3 windAt(t, dt)` advanced once per `step()` using the
  **seeded** RNG; new dispatch `case` (~beside `SET_FAULTS`, line 408); write
  `pose.wind_w`.
- **Physics:** push `v_wind` into `PhysicsCore` each tick (setter), change the
  drag term at `physics_core.cpp:30` from `s.vel_w` to `v_rel = s.vel_w - wind_w_`.
- **GCS:** `sendWind(WindConfig)`; wind-speed/dir readouts + the two mini-plots
  (`#gWindSpd`/`#gWindDir`) fed from `snapshot.wind_w` (do **not** recompute — it's
  stochastic).
- **UI:** "Wind & Turbulence" group in the **World tab** (mockup `index.html:618`).
- **Exit gate:** unit (steady→const, gust→sinusoid, turb RMS≈σ via the √(1−α²)
  check, mean≈0); physics weathervane-drift test; manual SITL graphs animate.

## Phase 2 — Atmosphere & aero  ([02](02-atmosphere-aero.md))

- **Proto:** `SET_ATMOS = 15`, `vsim_ctl_atmos_t`.
- **Daemon:** store `ρ_ratio` + GE config; pass both into the motor model; write
  `pose.airspeed` (`‖v_rel‖`) and `pose.ge_factor`.
- **Physics:** `motor_model.cpp:32` thrust `*= ρ_ratio·GE(z_agl)`; reaction torque
  (`:50`) `*= ρ_ratio` only (GE is lift, not torque); drag `*= ρ_ratio` at
  `physics_core.cpp:30`. `MotorModel::update()` gains `rho_ratio` + `z_agl` inputs.
- **GCS/UI:** `sendAtmos`; "Air & Thrust" group in the World tab (`index.html:626`);
  `airSpd`/`geFac` labels + `#gAirSpd`/`#gGndEff` plots from the snapshot; HUD
  airspeed tape beside the ground-speed tape.
- **Exit gate:** `GE(0)=1+k_ge`, `GE(h_ref)=1+k_ge/2`, `GE(∞)→1`; thrust∝ρ; physics
  GE-climb + low-density hover-deficit tests.

## Phase 3 — Advanced sensor error models  ([05](05-sensor-error-models.md))

Largest phase; scope = **acc/gyr/mag only** (baro/GPS deferred with their sensors).

- **Proto:** `SET_SENSOR_ERR = 18` (per-sensor) + `SENSOR_CSV = 19` (chunked rows,
  mirror the `sendWorldMesh` transfer at `SimWorker.cpp:294`).
- **Daemon:** a `SensorErrModel` per sensor running the 8-stage pipeline
  (scale → temp-bias → vibration → noise → bias-walk → spikes → saturation →
  latency); shared seeded RNG; latency ring + CSV cursor. Replace/extend the
  `SET_NOISE` case (`main.cpp:388`); keep σ-only behaviour reproducible.
- **GCS:** `sendSensorErr(sensor, cfg)` + chunked `sendSensorCsv`. Keep `pushNoise`
  as a thin wrapper populating `sigma`/`bias_clip`/`enable` so nothing regresses.
- **UI:** replace the flat `m_sensorRow[3]` (`SimulatorWidget.cpp:947`) with the
  per-sensor collapsible panel from the mockup (`buildSensors()` ~app.js:1294,
  `SENS_COMPS` ~1279): Gaussian/CSV source toggle, the full stack params, file
  picker. Sensor enable stays a **live** toggle (mockup `live-toggle`); the rest
  is staged Apply.
- **Sub-sequence within the phase** (each independently testable): scale + sat +
  spikes + latency (pure functions) → vibration (needs motor-RPM coupling) →
  temp-bias (needs a temperature source) → CSV replay last.
- **Exit gate:** per-stage unit tests (scale, clamp, spike-rate≈p, vib spectral
  peak at f_blade, latency cross-correlation lag, temp linear drift); all-off ≡
  today's path (regression guard); CSV row-for-row replay.

## Phase 4 — Magnetic field  ([03](03-magnetic-field.md))

Small; slots into the Phase-3 synth pipeline.

- **Proto:** `SET_MAGFIELD = 16`, `vsim_ctl_magfield_t` (three human inputs;
  NED conversion lives **in the daemon** — single source of truth).
- **Daemon:** setter converts intensity/dec/inc → `B_world`; the mag synth (now the
  Phase-3 pipeline) projects `B_body = R_world2body·B_world` then adds noise.
- **GCS/UI:** `sendMagField`; "Magnetic Field" group. **NB the mockup puts this in
  the World tab** (`index.html:631`), *not* the sensor/vehicle panel that plan 03
  names — follow the mockup. `magVec` readout computed locally for feedback.
- **Exit gate:** N/E/D conversion vs known values (inc 90°→pure D; dec 90°,inc 0°→
  pure E); body projection at level ≡ B_world; 90° yaw rotates N↔E.

## Phase 5 — Power / battery  ([04](04-power-model.md))

- **Proto:** `SET_POWER = 17`, `vsim_ctl_power_t`; state rides the Phase-0
  `batt_*` pose fields.
- **Daemon:** `PowerModel` after the motor update — `P∝ω³+P_idle` → I → V-sag
  (prev-step V, no algebraic loop) → integrate mAh/SoC; reset (`main.cpp:224`) also
  resets SoC. Optional `couple_thrust` scales effective `max_omega` (default off).
- **GCS/UI:** `sendPower`; battery block in `SimHudWidget` (V/A/mAh + SoC bar).
- **Mockup caveat (D4):** the mockup exposes battery only as a **readout**, with no
  editable pack params. Treat the HUD/readout as the parity deliverable; the config
  spinboxes (cells/capacity/R/k_power) are **beyond parity** — decide scope before
  building them.
- **Exit gate:** zero-throttle I≈P_idle/V; V monotonically falls with SoC; V<V_oc
  whenever I>0; with coupling on, fixed throttle → altitude decays as pack drains.

---

## Cross-cutting divergences from the mockup (decide before building)

These are places the five per-feature plans and the actual mockup disagree, or
where the mockup under-specifies. Resolve each when its phase starts.

- **D1 — Pose-frame ABI.** The frame is version-locked (static_asserts) and three
  features extend it. Plan: one bump in **Phase 0** (recommended) vs. bump per
  phase. Recommended: Phase 0, no firmware impact (verified).
- **D2 — Apply model. ✅ DECIDED: staged per-group "Apply to Sim" button**
  (match the mockup, `data-apply` dirty tracking, `app.js:1382`). Only the sensor
  enable stays a live toggle (mockup `live-toggle`). This overrides the per-feature
  plans' "live per-control push" wording — every new sim-config group in Phases 1–5
  is staged behind a per-group Apply button. **Prerequisite:** `SimulatorWidget`
  has no dirty/Apply scaffolding yet (today's `pushNoise` is live per-control);
  build that scaffolding as part of Phase 1's UI and reuse it for every later
  phase. The existing live `pushNoise` either moves behind the new Apply button or
  stays live for back-compat — settle that when Phase 3 reworks the sensor panel.
- **D3 — Magnetic-field UI location.** Mockup = World tab; plan 03 = sensor panel.
  Follow the mockup (World tab).
- **D4 — Battery is readout-only in the mockup.** Plan 04's config UI is richer
  than parity. Default to readout-only for parity; gate config behind an explicit
  decision.
- **D5 — Sensor panel shape.** Today: flat 3-row `m_sensorRow[3]`. Mockup &
  Phase 3: per-sensor collapsible with the full stack. Only acc/gyr/mag (the
  mockup's baro/GPS rows stay deferred).

## Per-phase definition of done

1. Builds clean (firmware + GCS + `tools/vsim`); no static_assert regressions.
2. Daemon unit tests for the phase's math, run headless with a fixed seed.
3. One physics/behaviour test proving the headline effect (per the plan).
4. Manual SITL: the World-tab/sensor controls drive the live readouts/graphs.
5. Independently shippable — revertable without touching another phase.
</content>
</invoke>
