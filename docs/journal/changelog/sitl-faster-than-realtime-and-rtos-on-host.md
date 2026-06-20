# SITL — faster-than-realtime & the real RTOS on host

Deep-dive changelog for the host SITL execution model: slaving firmware timing
to a virtual sim clock, pacing the sim faster than realtime, and ultimately
running the **real vaios scheduler on the host** with physics in-process —
deterministic and ~60× realtime. Newest first.

Design + rationale: [`docs/plans/sitl-lockstep-sim.md`](../../plans/sitl-lockstep-sim.md).
System view + build/run commands: [`ARCHITECTURE.md`](../../../ARCHITECTURE.md)
(§ "SITL execution model", § "Building and running"). Motivating analysis:
[`AUTOTUNE-ROLLPITCH-ANALYSIS.md`](../../../AUTOTUNE-ROLLPITCH-ANALYSIS.md).

---

## 2026-06-20 — Faster-than-realtime, deterministic SITL

**Why.** Autotune and log analysis both flagged a non-tracking roll/pitch inner
rate loop; iterating on it meant waiting for SITL at 1× realtime, with run-to-run
noise (wall-clock `dt` jitter) muddying the cost. Goal: run the *real firmware*
faster than realtime, deterministically, without sacrificing fidelity.

The work landed in phases on `feat/sitl-lockstep-virtual-clock`. The legacy
pthread SITL (`vayu_sitl`) stays the default and untouched in behaviour
throughout; the new capabilities are opt-in.

### Added

- **Virtual sim clock (Phase 1).** Firmware timing (`v_get_ticks`, control-loop
  cadence, telemetry timestamps) is slaved to **sim time** — one tick per IMU
  sample — instead of the wall clock. The IMU feeder advances a mutex/condvar
  clock; the wall-clock `hf_timer_thread` is gone. Driver-less unit tests
  self-advance the clock so `v_get_ticks()` still progresses. **Delay-caller
  audit:** host-infrastructure loops that wait on external/wall events
  (`host_main` idle, RC/serial backoff, boot settle) moved to a new
  `host_wall_delay_ms()` so they never block on the virtual clock.
  (`host_clock.h`, `host_vaios.c`, `host_imu_feeder.c`, `host_lifecycle.c`.)

- **vsim lockstep pacing (Phase 2, opt-in `VSIM_LOCKSTEP=1`).** `vsim_d` swaps
  its realtime `sleep_until` for PWM-round-trip backpressure: it runs as fast as
  the firmware returns PWM, bounded by a credit window (`VSIM_LOCKSTEP_CREDIT`,
  default 2). Default (unset) is unchanged realtime. (`tools/vsim/src/main.cpp`.)

- **Real vaios scheduler on host (Phase 4, opt-in `-DVAYU_SITL_RTOS_BUILD=ON`).**
  A new `vayu_sitl_rtos` target runs the **actual RTOS scheduler** (kernel
  `task.c`/`ipc.c`/`memory.c`/`vaios.c`) via a **`ucontext` port**
  (`host_rtos_port.c`) instead of free pthreads — so SITL now exercises the real
  scheduler (priorities, delays, semaphores), not a pthread approximation. The
  kernel's idle task yields back to the stepper, so quiescence is the kernel's
  own ready list (no race). Boots to STANDBY; `host_vaios.c` compiles out the
  collision set under `-DVAYU_SITL_RTOS`; `HEAP_SIZE` is `#ifndef`-guarded so the
  host can pass a large heap (target build unchanged).

- **Single-threaded, in-process stepper (Phase 4).** One thread drives the
  firmware deterministically: step physics → inject IMU → SysTick +1 → run the
  scheduler to idle (firmware writes PWM) → read PWM back inline. Measurement
  showed the two-process FIFO round-trip was ~98% of wall time, so vsim's physics
  (`SimController`) is **linked in-process** (isolated `vsim_phys` C++ lib,
  `vsim_inproc.cpp`) — no FIFO, no second process. Result: **~60× realtime**,
  self-contained, and **faithful** (PWM read inline, never stale).

- **Determinism validators.**
  `tools/sim_host/validate_rtos_determinism.py` runs `vayu_sitl_rtos` across
  seeds and asserts same-seed runs are **bit-identical** while different seeds
  differ — currently PASS (~60×). `tools/autotune/validate_lockstep_determinism.py`
  covers backend A (realtime-vs-lockstep credit sweep).

- **Docs:** the lockstep plan, the autotune soft-rate-loop root-cause analysis,
  and the ARCHITECTURE.md SITL-execution-model + build-reference sections.

### Changed / fixed

- Determinism validation of backend A found that the lockstep credit window adds
  control latency in sim-time; a *marginal* roll/pitch loop tumbles at credit ≥ 3
  while **credit = 2 tracks realtime within thread jitter** — so the default
  credit was set to 2 (faithful). High-speed *and* faithful needs backend B.
- The earlier RTOS non-determinism was traced to an **uninitialised IMU sample
  struct** in the stepper (garbage in fields `packImu` doesn't fill); zero-init
  fixed it. The scheduler + firmware are themselves deterministic.

### Trade-off summary

| | speed | fidelity | determinism |
|---|---|---|---|
| backend A, realtime | 1× | full | — |
| backend A, lockstep credit=2 | ~3× | faithful | — |
| **backend B (`vayu_sitl_rtos`)** | **~60×** | **faithful** | **bit-exact** |

### Not yet

Harness RC (arm/doublets) + wiring `vayu_sitl_rtos` into the autotune harness for
full tracking-quality runs; then making backend B the default SITL.
