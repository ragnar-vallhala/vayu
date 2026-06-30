# Postmortem: in-app autotune "violent rotation" on a rotated (rx=180) vehicle

**Symptom.** Loading the S500 vehicle and starting the in-app autotune, the
rig drone spun up violently in **roll** the instant a rollout began — "rotating
like a tap." Every eval reported `cost = 1e6 / DIVERGED`, so the tune never
converged. The vehicle's geometry had `rx=180` (the STL is modelled
upside-down, so the user rotates it 180° about body-X to display it right-side-up).

**Time spent:** ~5+ hours. Most of it was spent chasing false leads created by a
broken debug harness; the actual bug was small.

---

## What was actually wrong (two compounding bugs)

### 1. The real bug — the autotune never time-synced its sim
The NavLink v2 firmware has a **§10.5 command gate**: `router_command_gate`
(`src/comm/navlink_router.c`) returns `time_sync_is_synced() ? ACCEPTED : TEMPORARILY_REJECTED`,
and the generated dispatch runs it before **every** command handler. Until the
GCS disciplines the firmware clock (the TIME_SYNC handshake), `set_pid`,
`set_motor_geometry`, etc. are all rejected.

The in-app autotune spins up its **own isolated** `vsim_d + vayu_sitl` via the
C++ `SitlStack`, on a private link — and that link **never sent TIME_SYNC**
(only the main GCS link did, in `MainWindow`). So the autotune's
`set_motor_geometry` was silently rejected, and the firmware fell back to its
**compiled default mixer** `s_mix_roll = {-1,-1,+1,+1}` (`--++`).

### 2. Why the default mixer spun *this* vehicle
The firmware roll/pitch mix is `-sign(pos_y)` / `+sign(pos_x)` per motor, in the
**physics frame**. `rx=180` rotates the motor layout: `py` flips, thrust axis
flips to `-Z`. The correct mix for that rotated layout is `++--`, but the
rejected command left the firmware on its default `--++` — the exact inverse on
the roll axis → roll positive feedback → spin at zero setpoint.

A canonical (`rx=0`) vehicle never hit this: its physics frame *is* the default
layout, so the default mix is already correct even when the command is rejected.

---

## The fix

- **`SitlStack::start()` now performs the TIME_SYNC handshake** (one
  `encodeTimeSyncRequest`, `commanded_offset_ms=0`) before sending the
  geometry/PID commands — mirroring `sitl.py::sync_clock`. The gate opens, so
  `set_motor_geometry` is accepted and the **correct physics-frame mix** is
  applied.
- The firmware mixer is fed `physicsConfig()` (the physics frame) — the
  **original** behavior. No firmware, wire-protocol, or mixer-formula change was
  needed.
- Debug-tool fixes (python harness, not the app): `sitl.py::sync_clock`, and
  `protocol.py::ctl_geometry` now packs the motor `tau` field (see false alarm #6).

**Verification.** `navigator/tests/tst_sitl_stack` runs the app's *actual*
autotune objects (`SitlStack` + `runRollout`, the same ones the GUI's
`AutotuneWorker` drives) headlessly against the live geometry. Before: rollout
diverges / `nullopt`. After: holds level, arms, excites, scores **cost ~28**.
User confirmed the GUI autotune now tracks and converges.

---

## The false alarms (in order)

1. **Thrust-axis sign hunt.** Early on, assumed the motor thrust-axis sign was
   inverting roll; reverted a pile of "axis" edits. **Invalid:** the python
   harness was inert (its commands were time-sync-gated and rejected), so every
   test measured the *same fixed plant* regardless of the change.

2. **"The mix must be the un-rotated frame" (mixerConfig).** After getting the
   python harness to apply commands, reproduced the spin and concluded the
   firmware mix should use an **un-rotated (model)** layout, invariant to the
   cosmetic `rx=180`. Built a whole `mixerConfig()` / `mixerGeometry` path
   across GCS + harness. **Backwards** — it only looked right because of #6.

3. **Threading `axis_z` into the wire protocol + mixer** (`-sign(pos_y·axis_z)`).
   Passed on the user's vehicle but **broke the standard `az=-1` drone** on
   verification → reverted. (Symptom of reasoning from the same bad harness.)

4. **Wrong/stale binary, several times.** Real time lost to:
   - The app was being run from the `vayu-clean` worktree (clean baseline, none
     of the fixes), not the main build.
   - A rebuild doesn't update an **already-running** process (in-memory binary);
     had to fully quit + relaunch each time. (`/proc/PID/exe` showing `(deleted)`
     was the tell.)
   - The `[mix]` diagnostic was being wiped by a `m_tuneLog->clear()` placed
     right after it.
   - `strings` couldn't find the diagnostics because Qt `QStringLiteral` stores
     text as **UTF-16** (`strings -e l` showed them).

5. **"harness fail / pose 0.0" misread.** Looked like an arm failure or
   telemetry starvation. It was the drone **spinning on arm** — the pre-excite
   check aborted before the measured window, so the reported excitation-window
   pose read `0.0`.

6. **The harness that lied (root of the whole detour).** The python
   `ctl_geometry` packed each motor as 10 floats (`pos,axis,spin,k_thrust,
   k_moment,max_omega`) but the C `vsim_ctl_geometry_t` motor has a trailing
   **`tau`** field (11 floats). Omitting it **misaligned every motor after the
   first**, so vsim received a *scrambled* geometry — which happened to be
   stable with the *wrong* `--++` mix. That single missing field made the python
   harness "validate" the backwards fix (#2/#3) for hours. The C++
   `tst_sitl_stack` (the app's real code, correct struct) finally showed the
   truth: physics-frame mix `++--` is correct, and the only real bug was the
   missing TIME_SYNC.

---

## Lessons

- **The harness must be byte-identical to the firmware/sim it drives.** A single
  missing struct field (`tau`) silently scrambled the physics and sent the whole
  investigation the wrong way. Prefer the in-process C++ test (`tst_sitl_stack`)
  — it shares the real structs — as the ground truth over the Python reimpl.
- **A silent command gate is a debugging trap.** `set_*` returning
  `TEMPORARILY_REJECTED` with no surfaced error made an *inert* harness look like
  a *working* one running mysterious gains. The harness should assert on command
  ACKs.
- **Verify which binary is actually running.** `pgrep -x`, `/proc/PID/exe`
  (`(deleted)` = stale), and build-vs-process timestamps would have saved an hour.
