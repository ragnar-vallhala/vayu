# Simulator fidelity roadmap

Detailed, build-it-yourself implementation plans for the **large** simulator
features the UI mockup specifies that the SITL/sim does not implement yet. Scope
and prioritisation come from
[`../sim-mockup-parity-gaps.md`](../sim-mockup-parity-gaps.md);
this folder is where each Tier-1 subsystem gets a full plan an engineer can
implement from.

**Out of scope here (deferred deliberately):** GPS + GPS-glitch (Tier 1 #3) and
the barometer sensor (Tier 1 #4). Those are parked until position/altitude
estimation is actually exercised in the sim — they need an estimator consumer to
be meaningful, so planning them now would be speculative.

## The plans

| # | Plan | Subsystem | Touches |
|---|------|-----------|---------|
| 1 | [01-wind-turbulence.md](01-wind-turbulence.md) | Steady wind + gusts + Dryden-style turbulence | physics_core, proto, daemon, GCS World tab |
| 2 | [02-atmosphere-aero.md](02-atmosphere-aero.md) | Air density, ground effect, air-relative airspeed/drag | physics_core, proto, daemon, GCS World tab, HUD |
| 3 | [03-magnetic-field.md](03-magnetic-field.md) | Configurable field vector (intensity/declination/inclination) | proto, daemon, mag-sensor model, GCS sensor panel |
| 4 | [04-power-model.md](04-power-model.md) | Battery V/I/mAh from motor draw + sag | physics_core/motor, proto, daemon, pose frame, HUD |
| 5 | [05-sensor-error-models.md](05-sensor-error-models.md) | Scale/sat/spike/vibration/latency/temp-bias + CSV replay | proto, daemon sensor pipeline, GCS sensor panel |

## Shared design conventions

All five follow the same seam, so implementing one teaches the rest:

- **Protocol:** add a `VSIM_CTL_SET_*` opcode to the enum in
  `sim/vsim/include/vsim_proto.h` (next free value after `VSIM_CTL_SET_FAULTS = 13`),
  plus a `vsim_ctl_*_t` POD body struct (keep ≤ 256 B — the `vsim_ctl_frame_t.body`
  size). These plans reserve **14 = wind, 15 = atmosphere, 16 = magfield,
  17 = power-config, 18 = sensor-error** to avoid collisions if built in parallel.
- **Daemon:** add a `case` in the control dispatch `switch (cmd.subtype)` in
  `sim/vsim/src/main.cpp` (lines ~223–422); `memcpy` the body, push it into a
  setter on the controller/physics, log one line.
- **Physics:** environment forces sum into `force_b`/`torque_b` **before**
  `PhysicsCore::step()`, or as new terms inside `PhysicsCore::derive()`
  (`sim/vsim/src/physics_core.cpp` ~line 22) right where `linear_drag` is applied
  (line 30).
- **GCS send:** add a `send*()` to `navigator/src/vsim/SimWorker.{h,cpp}` mirroring
  `sendNoise`/`sendWorld` (build a `vsim_ctl_frame_t`, `::write(ctl_fd_, …)`).
- **GCS UI:** mirror the mockup ids in `navigator/src/ui/widgets/SimulatorWidget.cpp`;
  wire each control to a `push*()` slot (live-apply, like `pushNoise()` line ~937).
- **"Hot" vs "structural":** every feature here is a **hot** group in mockup terms
  (live-appliable while the sim runs) — none require a restart. Do not lock these
  behind `setMode()`.

## Suggested order

Same as the gap doc: **1 → 2** first (wind+atmosphere are the high-value physics
change and share the airspeed term), then **5** (sensor realism, biggest
sim-to-hardware transfer win), then **3** and **4**. Each plan is self-contained and
ships independently behind its own control message.
