# SITL consolidation — one in-process RTOS engine

**Status:** design (Phase 5, follows `docs/plans/sitl-lockstep-sim.md`).
**Goal:** collapse the two SITL paths into ONE faithful, fast, deterministic,
single-binary engine built on the in-process RTOS stepper (`vayu_sitl_rtos`),
and retire the decoupled `vsim_d` + legacy pthread host.

This doc covers **#11** (give the RTOS core the interactive surface the GCS
needs) and the contract for **#12** (migrate the GCS + headless SDK) and **#13**
(parity audit + deletion). #9 (armed deadlock) and #10 (sim-cadence `disturb`
scenario) are done.

---

## 1. Why consolidate

The physics core (`vsim::SimController`) is ALREADY shared by both paths. What
diverges is the **firmware host** and the **coupling**:

| | Decoupled (today's GCS) | In-process RTOS (`vayu_sitl_rtos`) |
|---|---|---|
| Firmware host | legacy pthread (`host_vaios.c`), free-running threads | ucontext RTOS port (`host_rtos_port.c`), 1 thread, cooperative |
| Physics | separate `vsim_d` process | linked in-process (`vsim_inproc.cpp`) |
| Coupling | 5 FIFOs (PWM/IMU/POSE/CTL/BARO), ~98% of wall time is round-trip | inline calls, zero round-trip |
| Determinism | no (pthread races + wall-clock) | yes (single thread, sim-clock, bit-identical, ASLR-independent) |
| Pacing | wall-clock (vsim_d), optional lockstep credit window | self-paced free-run (~40–60× realtime) |
| Pose / live config | yes (vsim_d + SimWorker) | **no** |

The RTOS core is the better foundation on every axis except it lacks the
**interactive surface** the GCS depends on. #11 adds that surface.

## 2. The decoupled surface we must replicate (factual map)

*Firmware in the GCS path:* `vayu_sitl_start()` is linked into Navigator and
boots the (pthread) RTOS threads in-process. `vsim_d` is a *separate* process.
PWM: firmware → `/tmp/vsim_pwm` → vsim_d → physics. IMU: vsim_d →
`/tmp/vsim_imu` → firmware feeder. SimWorker reads `/tmp/vsim_pose` and writes
`/tmp/vsim_ctl`. Per-instance isolation via `VSIM_FIFO_SUFFIX="_nav<pid>"`.

*vsim_d pacing* (`sim/vsim/src/main.cpp`): 8 kHz physics / 1 kHz IMU / 60 Hz
pose; `sleep_until(steady_clock)` wall-clock pace; optional `VSIM_LOCKSTEP`
(credit window, default 2) paces on PWM round-trip instead.

*Pose frame* (`vsim_pose_frame_t`, proto v3, 128 B): tick, pos_w[3],
quat_wxyz[4], vel_w[3], omega_b[3], motor_omega[4], motor_duty[4], + v3 fidelity
block (wind/airspeed/ge/battery). The RTOS stepper has all of this from
`g_ctl.state()` / `g_ctl.motorOmegas()` / `g_ctl.windWorld()`.

*Config surface SimWorker sends* (11 `VSIM_CTL_*`): RESET, SET_TESTRIG,
SET_GEOMETRY, SET_WORLD, CLEAR_OBSTACLES, ADD_OBSTACLE, SET_RATES,
SET_WORLD_MESH, CLEAR_WORLD_MESH, SET_FAULTS, SET_NOISE, SET_WIND.
`SimController` already has setters for most (`setMotorParams`/`setDroneParams`,
`setNoise`, `setObstacles`, `setWorldMesh`, `setTestRig`, `setWind`); vsim_d's
dispatch switch (main.cpp ~L313–537) is the translation reference.

## 3. Design — #11: interactive surface on the RTOS core

### 3.1 Factor the step loop into a reusable engine
`host_rtos_main.c` currently hardcodes scenarios in `main()`. Extract the loop
into `host_rtos_engine.{c,h}` exposing:

```
void rtos_engine_boot(uint32_t seed);            // v_system_init + vayu_sitl_start + scheduler_start + geometry/env
void rtos_engine_set_rc(int r,int p,int t,int y,int arm);   // existing set_rc()
void rtos_engine_step(control_telemetry_t* ct, int* got);   // existing step_once()
void rtos_engine_get_pose(vsim_pose_frame_t* out);          // NEW: snapshot g_ctl.state()
```

`main()` (scenarios) and the GCS worker both call this engine — one step loop,
no duplication. Headless behavior is unchanged (free-run, deterministic).

### 3.2 Realtime pacing (the key new piece)
Add an opt-in wall-clock pacer around `rtos_engine_step`, mirroring vsim_d:

```
rtos_engine_run(mode)   // mode = FREE_RUN (default; headless/CI/autotune)
                        //      = REALTIME (sleep_until next 1 ms boundary)
```

REALTIME uses `clock_nanosleep(TIMER_ABSTIME)` on a monotonic `next += 1ms`,
with catch-up-skip when behind (vsim_d L657–662 pattern). Sim-clock determinism
is untouched in FREE_RUN; REALTIME only gates how fast wall-time advances.

### 3.3 Pose publishing
After each step, pack `vsim_pose_frame_t` from `g_ctl`. Two sinks, selectable:
- **In-process (preferred end-state):** double-buffered snapshot + a
  thread-safe `rtos_engine_get_pose()` the GCS reads directly — NO pose FIFO.
- **FIFO (compat shim):** write the suffix-aware `/tmp/vsim_pose` so an
  *unmodified* SimWorker can attach during migration. Drop on EAGAIN
  (non-blocking, lossy — same rule as the #9 UART2 fix).

### 3.4 Config surface
Expose in-process C functions mirroring the 11 `VSIM_CTL_*` bodies, reusing the
`vsim_proto.h` structs as args, each a thin wrapper over the `SimController`
setter (port vsim_d's dispatch switch). No FIFO in the end-state; an optional
ctl-FIFO drain shim can bridge an unmodified SimWorker during migration.

### 3.5 Firmware-host / build
The GCS today links `libvayu_sitl_core` with the **pthread** port. The RTOS
engine needs the **ucontext** port + `vsim_inproc` + the engine. Produce a
`libvayu_sitl_rtos_core` (RTOS port, `VAYU_SITL_RTOS=1`) that Navigator links
instead. The firmware then runs as ONE cooperative worker thread (the stepper),
not N pthreads — simpler and deterministic. RC and config come from the GUI
thread via the engine API (lock or SPSC handoff to the stepper thread).

### 3.6 Hermetic tmp
Standalone binary should honor `VSIM_FIFO_SUFFIX` (or skip the vestigial PWM
FIFO write entirely in the RTOS path) so concurrent/abandoned runs can't share
`/tmp` state — closes the stale-FIFO contamination noted in #10.

## 4. Staged tasks
- **#11a** engine refactor (`host_rtos_engine`), scenarios call it — no behavior change.
- **#11b** realtime pacing mode + a `--realtime` standalone smoke test.
- **#11c** pose snapshot + getter (+ optional FIFO shim), validated against a vsim_d pose dump.
- **#11d** config C API mirroring the 11 ctl messages (+ optional ctl-FIFO shim).
- **#11e** `libvayu_sitl_rtos_core` build target.
- **#12** point SimWorker at the engine (get_pose + config calls); migrate `vayu_headless`. Keep FIFO shims until proven.
- **#13** parity audit (world mesh, obstacles, faults, baro, multi-instance), then delete `vsim_d` + pthread host.

## 5. Open decisions (need user input)
1. **GCS pose/config transport:** direct in-process API (cleanest, more
   SimWorker change) vs keep the pose/ctl FIFOs as the interface (smaller
   SimWorker diff, retains 2 FIFOs)? Recommend: in-process API, FIFO shims only
   during migration.
2. **Firmware host in the GCS:** commit to the ucontext RTOS port (full
   determinism, the stated goal) vs interim "pthread firmware + in-process
   physics" (smaller, drops the FIFO but keeps pthread nondeterminism)?
   Recommend: ucontext (it's the whole point).
3. **Realtime fidelity in the GCS:** is 1 ms `clock_nanosleep` pacing on a
   single worker thread acceptable for the live render, or does the GUI need the
   old 8 kHz physics / 60 Hz pose split? (The stepper is 1 kHz; pose can decimate
   to 60 Hz for the renderer.)

## 6. Risks
- Linking the ucontext port into a Qt app: symbol/heap ownership
  (`_heap_start`, `systick_count`, `scheduler_start`) must not clash with the
  pthread core — hence a *separate* lib + a build that picks one.
- Single-thread cooperative firmware inside the GUI: a blocking syscall in the
  stepper stalls the whole sim (cf. #9). All engine I/O must be non-blocking.
- Pose getter races: the GUI thread reads while the stepper writes — needs a
  seqlock/double-buffer, not a naive shared struct.
