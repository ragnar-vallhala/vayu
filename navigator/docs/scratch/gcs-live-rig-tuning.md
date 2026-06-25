# Study: live-drone PID autotuning in a rig, driven by the GCS

Status: 🔬 study / feasibility. Next phase of the autotuner: reuse the SITL
tuning method to tune a **real** flight controller held in a physical test rig,
driven over the live GCS link. Builds on the autotune stack
(`software/src/autotune/`), the source state machine
([gcs-source-state-machine.md](../journal/shipped/gcs-source-state-machine.md)), and the
[time-sync](../../../navlink/docs/reference/messages/time_sync.md) work.

## Context & goal

Today the autotuner runs entirely in simulation: `AutotuneWorker` spawns an
isolated SITL (`vsim_d` + `vayu_sitl`), the optimizer proposes gains, and a
`runRollout()` excites the simulated craft and scores the response. The operator
then **applies** the winning gains to a real board over the live link
(`CMD_SET_PID`).

The goal of this study: close the loop on **hardware** — run the same
optimizer/cost/excitation against a **real drone in a rig**, scoring real
telemetry, so the gains are tuned on the actual airframe + motors + sensors
rather than the sim model.

## Feasibility verdict

**Feasible, and most of the stack is already reusable — but it needs one
enabling firmware command and a deliberate safety design.** The blocker is that
the GCS has no way to inject stick/setpoint excitation over the live link.

### Reusable as-is (backend-agnostic)
- **Optimizer** (`Optimizer.{h,cpp}`) and **Space** (`Space.{h,cpp}`) — pure
  numeric search + parameter bounds. No sim knowledge.
- **Cost** (`Cost.{h,cpp}`) — pure function of a telemetry window
  (setpoint-vs-measured IAE + overshoot + chatter). The fields it needs are
  exactly the real FC's `control_telemetry_t` (setpoints + measured + outputs),
  telemetered as `SYSTEM_ORIGIN_PID_ERROR` and surfaced as `ControlLoopData`.
- **AutotuneEngine** — the rollout is already a `std::function<...>`
  (`AutotuneEngine.h:28`), i.e. **the backend is pluggable**. The engine has no
  SITL coupling.
- **applyGains()** — `setPid`/`setGyroLpf` are plain NavLink commands that work
  over the live link unchanged.

### The seam to introduce
`runRollout()` is hard-typed to `SitlStack &` (no interface). To plug a live
backend, abstract the ~10 calls it makes into an interface (or template):

```
setPid, setGyroLpf, setRc, arm, disarm, waitLevel, clearSamples, snapshot   // reusable
reset(seed), setTestRig(tetherK)                                            // SITL-only
```

Introduce an `IExciteStack` interface implemented by both `SitlStack` (today)
and a new **`LiveRigStack`** (drives the real FC over the GCS link). The
SITL-only ops become no-ops / different strategies on the live backend (see
"Sim vs real" below).

## The enabling gap: GCS excitation over the link

**There is no command to inject RC sticks or rate/angle setpoints over the live
link.** In SITL the tuner writes `sim_rc_channels[]` via a PTY that replaces the
iBus receiver (`rc_task.c`, `VAYU_SIM` path); a real board only takes RC from the
physical receiver. Applying gains works (`CMD_SET_PID`), but **exciting** the
craft does not.

**Proposed firmware addition — `CMD_SET_RC_OVERRIDE`** (a new GCS→FC command):
- Feeds GCS-supplied channel values into the same `ibus_raw_data` path the real
  receiver uses (so the entire RC → flight-mode → controller cascade is
  exercised, exactly like the SITL excitation). Tuning in **angle mode** then
  exercises the angle + rate loops the cost scores.
- **Deadman/heartbeat:** the override is valid only for a short TTL (e.g. 200 ms)
  and must be refreshed; if GCS overrides stop (link drop, tuner aborts), the FC
  reverts to the physical receiver, which — with no live stick — trips the
  existing RC-loss failsafe → disarm. This makes "GCS goes away" fail safe by
  construction.
- Must coexist with the physical receiver as a **safety override channel**: the
  operator's TX arm switch / throttle and a hardware kill must always win.

This command is the one genuinely new, safety-critical piece. It deserves its
own focused design + review before any motors spin.

## Rig requirements

The "test rig" in SITL is **pure sim physics** (`physics_core.cpp` hard-pin or
soft critically-damped tether) — there is **no firmware rig mode**, and none is
needed. The real equivalent is **mechanical**:
- A **1-DOF or 3-DOF gimbal / tethered test stand** that frees rotation but
  constrains translation, so the craft can be excited in roll/pitch/(yaw)
  without flying away or needing altitude hold. The soft-tether sim model
  (`tetherK`) approximates a compliant rig; a stiff gimbal approximates the hard
  pin.
- **Prop guards / containment** and a **physical kill** (cut power) independent
  of the firmware.
- Tuning runs in **angle mode** so the firmware's 70° bank-angle failsafe
  (`angle_controller.c`, `MAX_ANGLE_CUTOFF`) catches a divergent excitation
  (it's suppressed in acro).

## LiveRigStack — the rollout backend

Maps each `IExciteStack` op onto the live link:

| Op | Live implementation |
| -- | ------------------- |
| `setPid`/`setGyroLpf` | `CommandCodec::encodeSetPid/GyroLpf` → `sendToFc` (queued to the engine). |
| `setRc(...)` | `CMD_SET_RC_OVERRIDE` frame at ~50 Hz with the deadman heartbeat. |
| `arm`/`disarm` | `CMD_ARM`/`CMD_DISARM` (software-arm latch); confirm via system-state telemetry. |
| `snapshot`/`clearSamples` | Buffer the `ControlLoopData` stream (the engine already decodes it); window it per excitation. |
| `waitLevel` | Poll measured attitude from telemetry until \|roll\|,\|pitch\| < ε (no instant reset — real settle). |
| `reset(seed)` | **No-op.** Replaced by "restabilize" — disarm/hold level between rollouts; no deterministic seed. |
| `setTestRig` | **No-op.** The mechanical rig is the constraint. |

## Sim vs real — how it changes the algorithm

| Property | SITL | Real rig | Consequence |
| -------- | ---- | -------- | ----------- |
| Reset | instant, deterministic (`reset(seed)`) | none — must restabilize | longer inter-rollout settle; no replay |
| Noise | seeded, reproducible | real, non-repeatable | rely on **more repeats + statistics**; can't average over seeds |
| Constraint | sim tether | mechanical gimbal | rig compliance affects the response (model the rig, or tune with it in-loop) |
| Iteration time | ~seconds, fast | seconds–slower over the link + settle | **smaller optimizer budget**; favor sample-efficient search |
| Telemetry rate | high | `ControlLoopData` ~18 Hz | coarser cost window — may need a higher tuning-telemetry rate during a run |
| Timing | local | over WiFi/serial (jitter) | use the **time-sync** offset to align the cost window to FC time; gate on link health |
| Failure | retry via reset | safety abort → auto-disarm | a diverged/aborted rollout returns `nullopt`/`kBig`; the loop must re-level before retry |

## Safety architecture

Firmware already provides strong backstops (verified): **bank-angle failsafe
(70°, angle mode)**, **RC-loss watchdog (1 s) → FAILSAFE**, **throttle failsafe**,
**motor cutoff on any non-ARMED state**, and **CMD_SET_PID NaN/range validation +
persist**. The live-tuning design must add a GCS-side safety envelope on top:

- **Deadman heartbeat** on `CMD_SET_RC_OVERRIDE` (above) — loss of GCS → revert
  to RX → RC-loss failsafe.
- **GCS e-stop**: a single operator action that disarms immediately, plus an
  overall run timeout and a per-rollout watchdog.
- **Throttle cap** during tuning (rig hover only); never command beyond a
  configured ceiling.
- **Divergence → auto-disarm**: when the cost detects >limit angle error, stop
  exciting and disarm (don't just score `kBig`).
- **Operator-in-the-loop arming**: the physical TX arm switch and a hardware
  power kill always override the software latch.
- A dedicated **safety review** of `CMD_SET_RC_OVERRIDE` before first motor spin.

## State-machine integration

Live tuning needs the GCS **in `Fc`** (live link, tx allowed) **and** running a
tuning loop — unlike the current `Autotune` state, which is an isolated SITL run
that doesn't feed the engine. Options:
1. **A new top-level state `RigTune`** — entered from `Fc`, tx-allowed, feed
   active (it reads live telemetry to score). Strict-single-source teardown
   disarms + stops the override on exit. Cleanest fit for the FSM.
2. **A sub-mode within `Fc`** — a background tuning task gated on
   `state()==Fc`. Less explicit, but no new state.

Recommend (1): it makes the safety-critical mode explicit in the one authority
that already gates tx and the pill, and reuses the teardown guarantees.

## Design decisions (round 2)

Operator/safety model and run protocol, decided 2026-06-15:

- **Arming stays on the physical RC.** To enter `RigTune` the craft must already
  be **armed via the RC arm switch** with **both sticks held bottom-left** (safe
  idle). Only then can the GCS trigger the mode. Arm/disarm authority is never
  taken by the GCS.
- **Decouple the iBus decode from the RC path.** Refactor the hardcoded iBus
  parse out of `rc_task` into a clean **RC-source abstraction** so a GCS-injected
  source can be fed in without iBus frames "infecting" the GCS RC packets (and
  vice-versa). Safety vs excitation channels route separately (see new problems).
- **Two exits:** the GCS can leave the mode, **or** the operator pulls **both
  sticks up-right** on the physical TX → FAILSAFE (hardware-side escape,
  independent of the GCS).
- **Trim telemetry during a run:** disable all non-essential telemetry; only the
  cost-relevant stream flows, at a much higher rate.
- **Pause clock discipline mid-eval:** stop time-sync corrections during an eval
  so the FC clock free-runs monotonically and the window is easy to serialize;
  the in-flight data carries a **GCS sub-millisecond timestamp**; **re-sync
  between runs** to stay wall-correct.
- **Tune angle and rate modes separately** — their gains can differ by mode.
- More rollouts per eval for a usable approximation under real noise.

## Additional problems found in review

Beyond the round-2 decisions, these need answers before motors spin:

1. **RC is per-purpose multiplexed, not all-or-nothing.** During the loop the
   **safety channels (arm switch, throttle, the up-right escape gesture) must
   keep coming from the physical RX**, while only the **excitation channels
   (roll/pitch/yaw, and the hover throttle) come from the GCS**. The RC-source
   refactor must define channel routing + precedence: **RX safety always vetoes**
   the GCS injection, and the escape gesture is evaluated on RX even while the
   GCS drives the excitation sticks.
2. **Throttle ownership.** Entering at bottom-left means min throttle (arm
   precondition met), but excitation needs hover thrust — so the **GCS injects
   hover throttle** during the loop. The deadman must revert *all* GCS channels
   to RX on loss, so throttle falls back to the operator's min → safe settle.
3. **Excitation timing over a jittery link.** Streaming ~50 Hz `CMD_SET_RC` is
   distorted by WiFi jitter (the chirp especially). **Strongly prefer FC-side
   parametric excitation**: the GCS sends a waveform descriptor (axis, step/chirp,
   amplitude, duration) and the FC plays it out at the loop rate, deterministically
   — jitter-immune, low-bandwidth, reproducible. Bigger firmware change, but it
   removes the link from the timing-critical path.
4. **Cost alignment should use the FC's already-paired (setpoint, measured).**
   `control_telemetry_t` carries `roll_angle_sp` *and* `roll_angle_curr` sampled
   **together on the FC clock**. Score from those — don't try to align a
   GCS-side injected setpoint against FC-measured across the link, which the
   latency jitter would smear. Pausing discipline keeps that FC timestamp a
   stable monotonic reference within the eval; the GCS sub-ms stamp is for
   serialization + post-run wall mapping, not for the cost alignment itself.
5. **`CMD_SET_PID` persists to SD on every call.** A search applies gains
   hundreds of times → SD wear + write latency in the loop. Need a **volatile
   apply** (RAM-only) during tuning, persisting only the final accepted gains.
6. **Restore known-good gains on abort/exit.** A diverging rollout leaves the
   last (bad) gains live; on failsafe/exit the FC must fall back to a safe
   baseline, not whatever the optimizer last tried.
7. **Cascade order + per-mode storage.** The rate loop is inner, angle is outer —
   **tune rate first, then angle** with rate fixed; tuning out of order is
   unstable. And "gains differ by mode" needs **per-flight-mode gain storage** in
   firmware (today `pid_config` is `gains[ctrl][axis]`, not per-mode) — a
   prerequisite for separate angle/rate-mode tunes.
8. **Non-stationary plant.** Over hundreds of real rollouts the **battery sags
   and motors/ESCs heat up**, so the plant the optimizer sees drifts — later
   rollouts aren't comparable to earlier ones. Monitor pack voltage, cap session
   length / re-baseline, and prefer sample-efficient search.
9. **High-rate stream bandwidth + loss.** `control_telemetry_t` is 74 B; serial
   @230400 is ~22.5 KiB/s, so a few-hundred-Hz stream is near the ceiling and
   UDP/WiFi can drop frames. Pick the max sustainable rate, prefer the
   **lossless serial** link for tuning, and **sequence-number + gap-detect** the
   samples — reject any window with a gap rather than mis-scoring it.
10. **Optimizer robustness.** Real noise + drift make the cost much noisier than
    SITL; the current SPSA/structured search may converge poorly. Consider a
    **noise-aware / sample-efficient** optimizer (e.g. Bayesian optimization) for
    the live budget.
11. **Probing unstable gains is by design.** The search *will* try bad gains →
    violent oscillation. Keep **conservative `Space` bounds**, a tight GCS-side
    divergence limit that **aborts + disarms fast** (ahead of the 70° firmware
    cutoff), and a soft-start.
12. **Re-level between rollouts (no reset).** Without `reset(seed)` the craft must
    settle on the gimbal between rollouts (recenter / brief level-hold or
    disarm) — slower and occasionally needs operator help; budget for it.
13. **Rig fidelity is a fundamental limit.** Gimbal friction/inertia + the
    constraint bias the response, so rig-tuned gains may not be optimal in free
    flight (same caveat as the sim, different bias). Yaw especially may be
    constrained/low-authority on a tether.
14. **Sub-ms GCS clock = `QElapsedTimer`** (nanosecond monotonic) for intra-eval
    stamping, mapped to wall time at eval boundaries (`currentMSecsSinceEpoch` is
    ms-only).

## Refined architecture (round 3 — decided)

The review collapsed the design into a clean split, **GCS = optimizer brain +
operator UI + study/persistence; FC = per-pass executor**:

- **Per-pass handshake (ACK'd, not streamed):** GCS sends the run plan (candidate
  gains + excitation descriptor) → operator physically stabilizes the craft on
  the rig and clicks **Start** in the GCS → FC applies the candidate **in RAM
  only (never SD)**, **generates the excitation itself at the loop rate**,
  captures the window, **computes the cost on the FC**, and sends back the cost +
  summary. GCS picks the next candidate. The inter-pass time naturally covers the
  round-trip while the operator re-levels.
- **No high-rate link stream.** Normal telemetry keeps flowing for operator
  awareness; the high-granularity sensor + control-loop trace is **logged to SD
  on the FC** for offline study, recoverable later. This logging mode is explicit
  and must not disturb normal flow.
- **Excitation is FC-internal**, overriding the target axis's *setpoint* during a
  pass; the **physical RC keeps arm + throttle + safety + the entry/exit
  gestures** (enter armed with both sticks bottom-left; exit by GCS, or both
  sticks up-right → FAILSAFE). The GCS does **not** stream RC — so link jitter is
  off the timing-critical path entirely.
- **Bank-angle cutoff disabled** during a pass (the craft is mechanically tied to
  the rig — no runaway), scoped to the mode and auto-restored on exit. `Space`
  bounds still constrain candidates; the bounded pass duration + operator gesture
  are the abort.
- **No intermediate persistence or commit.** Candidates run transiently; only an
  explicit GCS **Apply** (button + confirmation overlay) writes the chosen gains
  to SD and into the live loop. On exit the FC reverts RAM → committed gains.
- **Clock:** pause time-sync discipline during a pass so the FC clock (used for
  the cost window + SD log) is clean + monotonic; re-sync between passes for
  wall-correct logs. FC-side cost means **no GCS sub-ms timestamp is needed**.
- **Same SITL optimizer/cost**; angle and rate(acro) modes tuned as separate
  runs; optimizer selection is a later study if results disappoint.
- **Operator-managed**: battery/thermal pause-or-continue is the operator's call
  (not wired for automated decisions). Rig stability is the explicit goal;
  tethered (more-DOF, more-valid) tuning comes later, gated on rig success.

This resolves the round-2 problems #1–#4, #8–#14 by construction (no RC streaming,
no cross-clock alignment, no high-rate link, no intermediate SD writes).

## Final clarifications (round 4 — closes the open questions)

- **Sequencing:** this is a *study only*. Execution starts **after NavLink v2
  integration lands**; any new packets/features the autotuner needs are added to
  NavLink v2 **before** autotune execution begins. The reliable command/result
  exchange is therefore not a blocker to design now — it's the first execution
  step.
- **Cost lives only on the FC.** The GCS never computes cost; it only helps the
  operator *gauge* the run (plots/progress). No shared/duplicated cost to keep in
  sync.
- **Statistical acceptance:** run **multiple rollouts per candidate and average**
  the response of the *same* PID set until a **rig-stability criterion** is met —
  this absorbs real-world noise (mirrors the SITL `repeats`).
- **User-seeded start:** the operator provides the **initial gain seed** (per
  mode), so the search starts from a sane point rather than probing wild values.
- **Phased & pragmatic:** work with what exists; don't pre-build abstractions.
  The firmware pieces below are sequenced into the execution phases, not all up
  front.

## Still open — firmware deliverables (execution phase, post-NavLink-v2)

The architecture moves real work onto the FC; these are the new pieces, all
sequenced after NavLink v2 — none is a study blocker:

1. **Reliable plan/result/start exchange.** NavLink today is fire-and-forget. The
   tuning command + result handshake needs reliable (seq/ack/retry) delivery —
   **build it on the in-progress NavLink v2 codec**, not a parallel layer. This is
   **gated on the NavLink v2 work** (`vayu-navlink-v2` worktree): RX/uplink demux
   (their Phase 3) and the COMMAND migration (Phase 4) are prerequisites for new
   reliable uplink commands. Coordinate the rig-tune messages there.
2. **FC pass-executor subsystem** (new, explicit/gated firmware task): receive
   plan → transient RAM gain apply (atomic, no half-applied set) → wait for Start
   → run excitation → window + cost → SD log → report → revert-to-committed on
   exit. Must coexist with flight code without disturbing it.
3. **Port the cost + excitation generator to firmware** (step doublet / chirp at
   loop rate; `axisCost`/`yawRateCost`). Share one implementation with the GCS/SITL
   path so live and sim costs stay identical — don't fork them.
4. **Decoupled RC path** (your point): factor the iBus decode behind an
   RC-source abstraction with a clean **setpoint-override hook** for the
   excitation, so iBus frames and the injected excitation can't cross-contaminate;
   plus the entry preconditions gate (armed + sticks safe) and the scoped
   cutoff-disable that auto-reverts.
5. **Buffered async SD logging** at control-loop granularity that never stalls the
   loop; SD throughput bounds the log rate.
6. **FC real-time budget** check: control loop + excitation + cost accrual + SD
   log + normal telemetry within one pass must fit the schedule.

## Phased plan

1. **Refactor to a backend interface** (no behavior change): extract
   `IExciteStack`; `SitlStack` implements it; `runRollout()` takes the interface.
   Unit-testable with a mock stack.
2. **`CMD_SET_RC_OVERRIDE` firmware command** + deadman, behind a build/config
   flag. Bench-test on the board (no props): verify override drives the RC path,
   heartbeat-loss reverts to RX, arm switch overrides.
3. **`LiveRigStack`** over the link; dry-run the rollout **disarmed** (apply
   gains, send excitation, capture telemetry, compute cost) — no motors.
4. **GCS safety envelope** (e-stop, timeouts, throttle cap, divergence-disarm) +
   the `RigTune` FSM state.
5. **Props-on rig bring-up**: single-axis (roll only), conservative budget,
   operator hand on the kill. Compare converged gains to the sim's.
6. **Full** roll/pitch/(yaw), tune the budget/repeats for real-noise statistics.

## Open questions / risks

- **`CMD_SET_RC_OVERRIDE` is safety-critical** — it lets software move the sticks
  of an armed drone. Needs its own design + review; the deadman + RX-override +
  hardware kill are non-negotiable.
- **Rig fidelity**: a stiff gimbal hides thrust-tilt (sim notes this over-tunes);
  a compliant tether is more honest but adds rig dynamics into the response.
- **Telemetry rate / timing**: 18 Hz `ControlLoopData` may be too coarse for the
  cost window; consider a higher tuning-telemetry rate + time-sync alignment.
- **Non-determinism**: real noise means more repeats and a statistical
  acceptance test ("is B really better than A?") rather than a single rollout.
- **Optimizer budget**: real rollouts are slow; favor sample-efficient methods.

## Key references
- Autotune: `software/src/autotune/{AutotuneEngine,Optimizer,Space,Cost,Rollout,SitlStack,AutotuneWorker}.{h,cpp}`
- Excitation/RC path: `Rollout.cpp` (`exciteAxis`/`exciteOnce`), `SitlStack.cpp` (`setRc`/`rcWriterLoop`), firmware `src/comm/rc_task.c` (`VAYU_SIM` override), `src/comm/rc_safety.c`
- Commands: `software/src/protocol/CommandCodec.{h,cpp}`, firmware `include/comm/comm_types.h`, `src/comm/comm_processor.c`, `src/control/pid_config.c`
- Telemetry for cost: `control_telemetry_t` (`include/variables.h`) → `ControlLoopData` (`software/src/core/Types.h`)
- Sim rig physics: `tools/vsim/src/physics_core.cpp`, `tools/vsim/include/vsim_proto.h` (`vsim_ctl_testrig_t`)
- FSM: `software/src/core/SourceController.{h,cpp}`, `SourceState.h`

## Changelog

| Date       | Author          | Description                          |
| ---------- | --------------- | ------------------------------------ |
| 15/06/2026 | ragnar-vallhala | Initial feasibility study            |
