# Lockstep SITL — faster-than-realtime, deterministic simulation

## Context

The SITL stack is **two processes paced by the wall clock**, coupled by lossy
FIFOs:

- **vsim_d** (physics, `sim/vsim/`) steps at `imu_hz` and throttles with
  `std::this_thread::sleep_until(next)` (`sim/vsim/src/main.cpp:540`). It
  ingests PWM with a non-blocking, **latest-wins** `poll` (`main.cpp:210`) — no
  backpressure.
- **vayu_sitl** (firmware host, `sim/host/`) runs each vaios task as a
  detached pthread, each sleeping against `CLOCK_MONOTONIC` + `nanosleep`
  (`sim/host/src/host_vaios.c`).

Everything runs at **1× realtime**. An autotune rollout is ~5–10 s of sim time;
a `structured --yaw` sweep is dozens of rollouts = many minutes of waiting, and
the wall-clock pacing injects `dt` jitter that makes the cost noisy (this is what
motivated the work — see the soft-rate-loop analysis in the changelog,
`docs/journal/changelog/sitl-faster-than-realtime-and-rtos-on-host.md`).

**Goal:** run the sim as fast as the CPU allows (target **10–50×**), deterministically,
without changing flashed-firmware behaviour.

Design only — no code here. File:line references point into the live tree.

---

## 1. What already works in our favour

Physics integration is **already decoupled from the wall clock**:

- `host_imu_feeder.c:230` stamps every IMU sample with a **fixed sim-time
  increment** (`SYS_CLOCK_FREQ / SITL_IMU_FEED_HZ`). The estimator derives `dt`
  from those stamps, not from wall time — so faster sample arrival = faster sim
  time, with `dt` automatically correct.
- The estimator tasks are **event-driven**: they block on the IMU/input queue
  (`attitude_task.c:91`, `vertical_task.c:53`), so they already run exactly once
  per sample at any speed.
- The IMU and baro **feeders block on `read()`** (`host_imu_feeder.c`,
  `host_baro.c`) — they self-pace on FIFO data and need no change.

The hard part (correct `dt` under acceleration) is done. The blockers are the
*cadence* of the wall-clock-paced pieces.

---

## 2. The blockers

### 2a. Control-loop cadence is wall-clock with a fixed `dt` (the killer)
The inner/outer loops run on `task_delay_until` against `v_get_ticks()` (ms) and
integrate with a **constant** `dt`:

- `angle_rate_controller.c:196-205` — 1 kHz, `dt = INNER_LOOP_DT`, drains the IMU
  ring to the **freshest** sample, discarding intermediates.
- `angle_controller.c:181-191` — 250 Hz, `dt = OUTER_LOOP_DT`.

If we just delete vsim's pacing, IMU floods in (say 50 kHz) but the control loop
still wakes every 1 ms **wall**, advances **one** `INNER_LOOP_DT` step, and drops
the other 49 samples → physics races 50× ahead of control → total desync. **You
cannot accelerate by removing pacing alone.**

### 2b. HF timer stamps telemetry from the wall clock
`host_lifecycle.c:92` (`hf_timer_thread`) bumps the high-frequency counter from
`CLOCK_MONOTONIC`; telemetry timestamps would not match sim time under
acceleration.

### 2c. vsim has no backpressure
PWM ingest is latest-wins (`main.cpp:210`) and the loop is throttled only by
`sleep_until`. Remove the sleep and vsim outruns the firmware unboundedly.

---

## 3. Delay-caller audit (blast radius)

`v_delay` / `task_delay` / `task_delay_until` are the **vaios RTOS primitives**.
On hardware they are the real kernel; in SITL they are re-implemented in
**`host_vaios.c` only** — so converting them to a virtual clock changes **zero
flashed-firmware behaviour**. But the *same symbols* are shared across three
layers that need different treatment.

### Layer 1 — firmware RTOS tasks → make VIRTUAL
All funnel through the 3 `host_vaios.c` functions. Only the **SITL-live** tasks
(created in `host_lifecycle.c:180-196`) execute during a sim run:

| Site | Rate | SITL-live | Treat |
|---|---|---|---|
| `angle_rate_controller.c:205` | 1 kHz inner | yes | **virtual** (critical) |
| `angle_controller.c:191` | 250 Hz outer | yes | **virtual** |
| `actuator/motor.c:67` | motor_task loop | yes | virtual |
| `comm/comm_processor.c:159` | cmd processor | yes | virtual |
| `comm/channel.c:342` | TX wait | yes (flush/comm) | virtual |
| `sys/heartbeat.c:43,158,169` | heartbeat | no (not created in SITL) | HW only |
| `comm/telemetry_task.c:154`, `perf_telemetry.c:124`, `rc_task.c:162` | telem/perf/RC | no | HW only |
| `sensor/bmx160.c` (~20), `bme280.c` (~6) | driver init/cal waits | no (feeders replace) | HW only — real datasheet timing |

The live SITL delay set is small: the two control loops plus motor/comm. The big
`bmx160`/`bme280` blocks are HW driver code the host build does not run.

### Layer 2 — host-shim infrastructure → must STAY WALL-CLOCK
These link the **same `v_delay` symbol** but wait on non-sim events; a virtual
`v_delay` would **deadlock** them:

| Site | Waits for | Why wall |
|---|---|---|
| `host_main.c:36` `v_delay(1000)` | process-alive idle until SIGINT | not sim-bound |
| `host_rc_feeder.c:149,212` `v_delay(1/20)` | EAGAIN backoff + 50 Hz RC inject | reads harness RC over pty in wall time |
| `host_lifecycle.c:71,198` `v_delay(2/200)` | passthrough loop + one-shot boot settle | boot settle is one-time wall |
| `host_navhal.c:249,253` `nanosleep` | UART2 RX bytes from external GCS | real external I/O |
| `host_lifecycle.c:118` `nanosleep` | hf-timer | **deleted in Phase 1** |

**Required refactor:** add `host_wall_delay()` (raw `nanosleep`) and move the
Layer-2 sites to it, freeing `v_delay`/`task_delay_until` to become virtual for
Layer 1. This is the single most important prep step — without it the virtual
`v_delay` catches the host's own I/O loops and hangs.

### Layer 3 — the harness → must pace in SIM-TIME
The autotune harness paces excitation with wall-clock `time.sleep()`
(`tools/autotune/autotune.py` `_excite_once` → `hold`/`ret`/`settle`;
`tools/autotune/sitl.py` reader threads). Even with firmware + vsim fully
virtual, "hold doublet 1 s wall" at 30× becomes 30 s of sim → windows misalign.
The harness must block on **sim-tick / IMU-sample-count** advancement instead of
`time.sleep`. In true lockstep the harness *drives* the step count.

---

## 4. The plan (phased by priority)

### Phase 1 — single virtual clock (firmware host side) — DONE (branch `feat/sitl-lockstep-virtual-clock`)
Implemented in `sim/host/include/host_clock.h` + `host_vaios.c`
(`host_clock_advance_us` / `host_clock_now_us` / `host_clock_stop` /
`host_clock_set_driven` / `host_wall_delay_ms`). The IMU feeder advances the
clock + drives the HF counter (`host_imu_feeder.c`); the wall-clock
`hf_timer_thread` is deleted (`host_lifecycle.c`). Layer-2 carve-out done
(`host_main.c`, `host_rc_feeder.c`, boot-settle in `host_lifecycle.c` →
`host_wall_delay_ms`). Driver-less unit tests self-advance the clock (no feeder)
so `v_get_ticks()` still moves; the live stack flags `driven` at startup.
Verified: SITL host builds, unit tests 7/8 (the 1 failure is a pre-existing
VFS-overflow test, not clock-related), and a full vsim_d+vayu_sitl smoke run
shows the virtual clock advancing (`v_get_ticks` 1623→4617 ms) with no deadlock.

Original design notes:

Introduce `g_sim_now_us`, advanced **only** by the IMU feeder
(+`1e6 / SITL_IMU_FEED_HZ` per sample), guarded by a mutex + condvar.

- `v_get_ticks()` → `g_sim_now_us / 1000` (`host_vaios.c:44`).
- `v_delay` / `task_delay` / `task_delay_until` → block on the condvar until
  `g_sim_now_us` reaches the deadline (broadcast on every advance), replacing
  `nanosleep` (`host_vaios.c:54,64,71`).
- Delete `hf_timer_thread`; derive the HF counter from `g_sim_now_us`
  (`host_lifecycle.c:92`, `host_navhal.c:526` `hal_cycle_counter_get`).
- Prereq: the **Layer-2 carve-out** (§3) so host I/O loops stay on wall time.

Effect: "wait 1 ms" = "wait for 1 sim-ms = 1 IMU sample". The control loop
re-locks to physics at any wall speed; its fixed `dt = INNER_LOOP_DT` stays
correct (one tick is still one `INNER_LOOP_DT` of sim time). **No control-code
change** — the deliberate "drift-free periodic, not IMU-driven" design
(`angle_rate_controller.c:197`) is preserved, just driven by virtual time.

*Standalone-testable:* run a fixed-gain rollout, assert telemetry timestamps
advance at the sim rate independent of wall time, and attitude/rate traces are
bit-identical to a 1× run.

### Phase 2 — free-run + bounded backpressure (vsim side) — DONE (branch `feat/sitl-lockstep-virtual-clock`)
Implemented in `sim/vsim/src/main.cpp`: env `VSIM_LOCKSTEP=1` swaps the
`sleep_until` wall pacing for PWM-round-trip backpressure. A credit window
(`VSIM_LOCKSTEP_CREDIT`, **default 2** — see the validation finding below) lets
vsim run up to N IMU samples ahead of the last acknowledged PWM (primes the
firmware pipeline, bounds IMU-FIFO occupancy so no frames are dropped), then
blocks for fresh PWM with a 100 ms real-time watchdog so a non-producing
firmware can't wedge the sim. Default (env unset) keeps the original realtime
pacing.

**Determinism validation (key finding).**
`tools/autotune/validate_lockstep_determinism.py` runs a seeded, constant-input,
rig scenario and compares the per-frame attitude trajectory: two realtime runs
set the noise floor (thread jitter, ~0.2–0.4° max), then a credit sweep checks
lockstep against it. Result:

| credit | max angle Δ vs realtime | verdict |
|---|---|---|
| 16 | ~175° (vehicle tumbles) | FAIL |
| 8 | 7.5° | FAIL |
| 4 | 3.5° | FAIL |
| 3 | 2.4° | FAIL |
| **2** | **0.2–0.4° (within floor)** | **PASS** |
| 1 | — | arm fails (pipeline can't prime) |

The credit window is also **added control latency in sim-time** (vsim applies
PWM up to `credit` samples stale). The roll/pitch loop is *marginally stable*
(the low-`rate_kp` defect from the autotune analysis), so even ~4 ms of extra
latency tips it into a divergent tumble — while credit=2 tracks realtime to
within thread jitter. So the **faithful default is 2**, giving ~**3×** wall-clock
speedup for an armed marginal-plant rollout (the earlier "~45×" smoke was a
*disarmed*, benign case with no control loop to destabilise). A well-damped
plant tolerates a larger credit (more speed); the ceiling is plant-dependent.

Takeaway: the threaded run-ahead design has an inherent **speed↔fidelity
trade-off**. High-speed *and* faithful needs Phase 4 (single-step, zero added
latency). Phases 1–2 deliver a validated ~3× that preserves behaviour.

Original design notes:

Add `VSIM_CTL_SET_SPEED` (or a `VSIM_FREERUN` env). When set:

- Skip `sleep_until` (`main.cpp:540`).
- **Block for one PWM frame before each step** (a credit of 1–2 in-flight
  samples) instead of latest-wins. This closes the loop deterministically:
  emit IMU → firmware estimate→control→mix → PWM returns → step physics → repeat,
  as fast as the CPU allows. The credit window absorbs the firmware's
  cross-thread pipeline latency (estimator→control→motor are separate pthreads)
  without unbounded runahead.

Tune credit depth vs. pipeline depth so the controller is not always one step
stale (it already tolerates staleness — `angle_rate_controller.c:213` holds the
last sample).

### Phase 3 — sim-time harness pacing
Replace `time.sleep(hold)` in `tools/autotune/sitl.py` / `autotune.py` with a
wait on sim-tick advancement (query vsim tick over the ctl channel, or count IMU
samples seen on the telemetry stream). Required for the doublet/settle windows to
have correct **sim** duration under acceleration.

### Phase 4 — real vaios scheduler on host + in-process physics — ACHIEVED (branch `feat/sitl-lockstep-virtual-clock`)
Built the RTOS-on-host port and the single-threaded in-process stepper. Outcome
on `vayu_sitl_rtos` (opt-in `-DVAYU_SITL_RTOS_BUILD=ON`):
- **Real vaios scheduler runs on host** via a ucontext port (`host_rtos_port.c`);
  the kernel's idle task yields to the stepper → quiescence is the kernel's own
  ready list, no race. Boots to STANDBY (`eb4ce16`).
- **Single-threaded stepper** drives the firmware deterministically: inject IMU →
  SysTick+1 → run scheduler to idle (PWM written) → repeat (`36d59c2`).
- **In-process physics** (isolated `vsim_phys` lib) removes the two-process FIFO
  round-trip that was ~98% of wall time → **~57× realtime**, self-contained, and
  faithful (PWM read inline, never stale) (`34f586d`).
- **Deterministic**: two same-seed runs are **bit-identical** (IMU-input and
  estimator-attitude fingerprints match exactly).

So Phase 4 beats the Phase 1–2 trade-off outright: faithful **and** ~57× **and**
reproducible. Remaining polish: harness RC (arm/doublets) + wiring
`validate_lockstep_determinism.py` to `vayu_sitl_rtos`, then make it the default
SITL. Original investigation notes (the discrete-event-scheduler dead-end that
the real-RTOS approach replaced) follow.

### Phase 4 (superseded notes) — counter-barrier dead-end — INVESTIGATED, NOT LANDED
Goal: advance physics and the firmware one sim-sample in lockstep — feed one
IMU sample, let the estimator→control→motor pipeline fully propagate it (PWM
settled), then step physics — for zero added latency, bit-determinism, and full
CPU speed (no credit-window latency, no thread-handoff/watchdog overhead).

**Task/blocking structure (verified).** The 8 SITL-live tasks park each
iteration on one of two primitives:
- **semaphore** (`v_semaphore_take`): `attitude_task` (`_imu_attitude_sema`,
  posted by `imu_queue_attitude_push`), `vertical_estimator_task`
  (`_vert_input_sema`, posted by `attitude_task`).
- **virtual clock** (`v_delay`/`task_delay_until`): `angle_controller` (250 Hz),
  `angle_rate_controller` (1 kHz), `motor_task` (2 ms), `imu_telemetry_task`
  (6 ms), `flush_task` (1 ms), `comm_processor_task` (4 ms).
  (NB: the "loop blocks on …_wait()" comments at `angle_controller.c:286` /
  `angle_rate_controller.c:453` are **stale** — the code uses `task_delay_until`.)

**Why the naive barrier is wrong — the premature-settle race.** A parked-task
counter (`g_parked == g_tasks` ⇒ quiescent) does NOT work: when the feeder
advances the clock or posts the IMU semaphore and then checks the count, a *due*
task is still inside `pthread_cond_wait` and hasn't decremented the count yet, so
the feeder sees "all parked" and reads the next sample **before** the woken task
processed the current one. With the OVERWRITE queues that silently **drops the
sample → divergence** (intermittent, scheduler-dependent). Confirmed by reasoning
through the wake/reacquire ordering; a counter cannot distinguish "parked, work
done" from "posted, about to wake."

**What it actually requires.** A small **discrete-event scheduler** that knows
each waiter's wake-condition, so the feeder waits for *true* quiescence (no task
runnable):
- clock side: a priority queue of pending wake **targets**; on each advance, the
  due set (target ≤ now) is known and must drain before settling;
- semaphore side: per-sem **armed-token** accounting — a `give()` to a sem with a
  parked waiter marks that waiter runnable until it consumes;
- quiescence = no running task ∧ no due clock waiter ∧ no armed token.

N≈8 so the structures are tiny, but it is real synchronization work, not a
counter — a racy version is worse than none, so it was **reverted** rather than
landed.

**Better approach (chosen): run the REAL vaios scheduler on the host.** Rather
than hand-roll a cooperative scheduler in the shim (which only *approximates*
vaios's policy and re-solves quiescence), use the actual RTOS. SITL today
**stubs** the scheduler (`scheduler_init/start` no-ops, `task_create` → raw
pthread) and lets Linux interleave the 8 threads — that *is* the nondeterminism,
and it means SITL never tests the real scheduler. Running the real kernel gives
determinism **and** maximum fidelity, and **quiescence becomes free**: the
kernel's own ready list (`ready_bitmap`) says when nothing but the idle task is
runnable — no race to detect.

Feasibility — verified by probe:
- vaios has a clean **policy/mechanism split**: `kernel/task.c` (ready lists,
  `set_next_task`, `delayed_list`, block/wake) + `kernel/ipc.c` (real semaphores)
  are **portable C with zero ARM asm / no `CORTEX_M4` ifdefs**, and `task.c`
  **compiles clean against host headers** (only cosmetic 64-bit pointer-cast
  warnings in a stack guard). Only `portable/cortex-m4/port.c` is ARM (the PendSV
  context switch) — replaced by a host port.
- **Host port surface** (small): `init_task_stack`, `task_yield`,
  `scheduler_start`, `load_next_task_from_isr`, `v_port_cpu_relax` via
  **`ucontext`** (`makecontext`/`swapcontext`, one OS thread, cooperative →
  deterministic); critical sections `v_port_disable/enable_interrupts`; HW stubs
  `v_port_hw_*` (no-ops / wire to the virtual clock + host VFS + stderr); drive
  `SysTick_Handler` / `wake_up_delayed_tasks_isr` from the virtual clock.
- **Reconcile**: delete the colliding shims from `host_vaios.c` (`task_create`,
  `v_delay`, `task_delay`, `task_yield`, `v_semaphore_*`, `v_mutex_*`,
  `v_malloc/free`, `scheduler_*`, `v_init/start/system_init`) — the kernel
  provides them; `host_vaios.c` keeps only the true-host bits (print, panic,
  mem*, the port).

Staged build (opt-in `vayu_sitl_rtos` CMake target so the pthread SITL stays
intact until the RTOS variant is validated):
1. Host `ucontext` port + link real kernel (`task.c`/`ipc.c`/`vaios.c`/
   `syscalls.c`/`memory.c`/`utils.c`); get it to boot to STANDBY.
2. Drive the SysTick from the virtual clock; fold the IMU/baro/rc feeders into a
   single-threaded **stepper** that injects sensors + ticks + runs the scheduler
   to idle (PWM settled) per sample.
3. Validate with `tools/autotune/validate_lockstep_determinism.py` (must PASS
   *and* beat credit=2 on speed); then make it the default.

Until that lands, **Phases 1–2 at credit=2 are the validated, faithful path**
(~3×).

---

## 5. Risks & verification

- **Shared-symbol deadlock** (the main risk): if any host I/O loop keeps the
  virtual `v_delay`, it hangs. Mitigation: the Layer-2 carve-out, plus a grep
  gate that no `sim/host` infra file calls the virtual `v_delay`.
- **Condvar correctness:** advance `g_sim_now_us` **after** pushing the IMU
  sample (consumers must see data at the new time), and **broadcast** so all
  waiting tasks re-evaluate their deadlines.
- **Hidden wall-clock waits:** confirm no SITL-live firmware thread busy-waits on
  the wall clock outside the delay primitives (comm/flush/motor use `v_delay` —
  covered).
- **Determinism check:** a fixed seed + fixed gains must produce a bit-identical
  telemetry trace at 1× and at N×. This doubles as the regression that the
  refactor preserved behaviour, and directly attacks the noisy-cost problem in
  the autotune analysis.
- **HW untouched:** all changes live in `sim/host` + `sim/vsim`; the
  flashed firmware (`src/`) is not modified.

## 6. Payoff
- **Speed:** rollouts become CPU-bound; a structured sweep drops from minutes to
  seconds (~10–50×).
- **Determinism:** removing wall-clock pacing kills the FIFO-burst `dt` jitter
  the code already fights (`host_imu_feeder.c:227`, the pre-excitation-transient
  retries in `Rollout.cpp` / `autotune.py`), making `eval(x)` reproducible.
