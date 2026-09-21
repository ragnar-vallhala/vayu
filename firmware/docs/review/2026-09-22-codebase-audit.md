# Codebase correctness audit — 2026-09-22

Scope: firmware (`src/`, `include/`, `linker.ld`, `CMakeLists.txt`) and sim
(`sim/host`, `sim/vsim`), reviewed in six chunks at HEAD `a03de00`. Tests and
submodules (vaios, navlink, vtest) were read only where a finding needed them.
Read-only review: nothing was changed.

Items marked **✔** were re-checked against the source by hand. The rest are
reviewer findings with named code paths that have not yet been re-verified
line by line.

## Critical — fix before the next flight

1. **✔ SACK repair length overflows the xfer stack** —
   `src/comm/xfer/navlink_xfer.c:311`. `want` is clamped to the file size, not
   to `XFER_CHUNK_MAX`; `tick_download` then reads it into `uint8_t
   buf[XFER_CHUNK_MAX]` (`:427`). One `XFER_SACK` with `miss_len=65535` during a
   download writes ~64 KB over the stack → HardFault. SACK is not a command, so
   the TIME_SYNC gate does not cover it.
   Fix: `if (want > s->chunk_size) want = s->chunk_size;`
2. **✔ Cancel-calibration disarms in flight.** `navlink_router.c:330` lets
   `which==0xFF` skip the STANDBY/FAILSAFE gate; with no calibration task,
   `comm_processor.c:146` calls `system_state_set(STANDBY)`, which `state.c:61,63`
   allows from ARMED and IN_AIR. A stale cancel pressed in the air cuts motors
   (and also pulls the FC out of FAILSAFE).
   Fix: only restore state when the current state is CALIBRATING.
3. **✔ No watchdog, no motor-command staleness cutoff.** No IWDG is started in
   firmware or vaios; a panic/HardFault halts with the timers still emitting the
   last PWM duty. `src/actuator/motor.c:57` replays `prev_motor_outputs`
   indefinitely when the queue is empty.
   Fix: start the IWDG and kick it from the rate loop; zero the PWM compare
   registers in `v_panic`/HardFault; stamp motor commands and output zero after
   20–50 ms without one.

## High

### Control

4. **✔ Height-hold hand-back can latch a climb** —
   `src/control/height_controller.c:146` with `:45`. An in-flight engage sets
   `base = clamp(stick, 0.30, 0.75)`; `handback_arm` caps at `base`, not hover
   (0.38). Engage at a punched-up stick → runaway guard trips → the pilot is
   handed 0.75 and stick-down cannot descend until the stick rises back to 0.75.
   `handback` is not cleared on switch-centre either. Same class as the
   2026-09-04 ceiling climb.
   Fix: cap the hand-back at `hover_ref`; seed in-flight `base` from
   `min(stick, hover_ref)`; LAND always uses `hover_ref`.
5. **✔ Stick tilt command exceeds the bank cutoff** — `include/variables.h:225,229`
   (`*_ANGLE_TARGET_MAX` 100°) vs `:239` (`MAX_ANGLE_CUTOFF` 70°). ~89% stick
   commands 70°: the recovery trips and re-trips in the air, and in ARMED (not
   yet IN_AIR) it goes to FAILSAFE.
   Fix: target max ~45–55°, below `MAX_ANGLE_RECOVER`.
6. **✔ Rate I-terms zeroed in flight below 0.3 throttle** —
   `src/control/angle_rate_controller.c:514` (`RATE_PID_INTEGRATE_THROTTLE` 0.3,
   `:379`). Hover is 0.38, so descents, chops and auto-land wipe the integrator
   every tick; the known left-thrust bias then rolls the craft. The comment at
   `:376` still cites the X3's 0.55 hover.
   Fix: gate with `state != SYSTEM_STATE_IN_AIR`.
7. **One NaN idles all motors** — `src/control/mixer.c:430`, `pid.c:74`.
   `m_clamp` passes NaN; any NaN axis makes every motor NaN → idle floor. A NaN
   gyro sample poisons `d_filtered`/`prev_meas`/gyro LPF state until re-arm.
   Fix: zero non-finite `w[a]` in the mixer; in `v_pid_update` return 0 and
   reset filter state on non-finite input.
8. **Motor geometry accepted while armed** — `mixer.c:258-264`,
   `navlink_router.c:400-416`. `mixer_set_geometry` rewrites `B`, `Bpinv`, idle
   floor and airmode in place while the 1 kHz loop reads them; a failed
   `m_mat4_inv` is ignored (new `B` + stale `Bpinv`); the layout is persisted to
   `pid.bin` and NaN positions are applied live.
   Fix: reject unless disarmed; build into a temporary and copy on success;
   reject non-finite positions.

### Comm

9. **Software-arm latch defeats the RC kill switch** — `src/comm/rc_safety.c`
   (`rc_arm_engaged` = switch OR `g_sw_arm_request`), `navlink_router.c:298-322`.
   After GCS `CMD_ARM`, arm-switch-down no longer disarms; `CMD_DISARM` ACKs OK
   even when the switch holds the vehicle armed; the latch survives an arm
   timeout and can arm the vehicle later.
   Fix: clear the latch on timeout, on any switch-down edge and on every disarm.
10. **Protected-file delete bypass** — `src/comm/xfer/fs_query.c:84-133`
    (`same_file`). FatFS ends an SFN at any byte ≤ `' '` and treats a trailing
    `/` as the last segment, so `"0:pid.bin x"` or `"0:cal.bin/"` fails the
    basename compare but deletes the protected file.
    Fix: normalise (cut at first char ≤ `' '`, strip trailing separators) before
    comparing.
11. **Upload bypasses delete policy and arm state** —
    `src/comm/xfer/providers/file_provider.c:43-82`. Offset-0 upload runs
    `fs_owner_truncate` on any path (`pid.bin`, `cal.bin`, rings), armed or not.
    Fix: same path policy plus a disarmed-only check in `xfer_on_open`.

### Sensors / estimation

12. **Dead baro/ToF knocks out the IMU** — `src/driver/bmx160.c:1025-1030`,
    `i2c_manager.c:228-233`. Each secondary-device failure calls
    `i2c_manager_unstick()`, leaving PB8/PB9 as GPIO until the 50-tick watchdog.
    `*_is_present()` is never cleared, so a loose ToF gives a ~50 ms gyro gap
    every ~45 ms in flight.
    Fix: drop the extra unstick, restart the fast read immediately, clear
    `present` after N NACKs.
13. **✔ `bmx160_init()` return ignored** — `src/main.c:87`. On chip-ID/config
    failure `bmx160_ready_sema` stays NULL; `v_semaphore_take(NULL)` returns
    immediately so `imu_read` (prio 2) busy-spins and starves comm/motor/
    attitude; scales stay 0.
    Fix: create semaphores before any early return; check the result and latch
    a fault that blocks arming.
14. **✔ Estimator point-samples the decimation window** —
    `src/est/attitude_task.c:127-144`. The latest gyro/accel sample is multiplied
    by the whole 8-sample `dt_sum`. The comment says this is deliberate for
    latency, but the upstream LPFs (~330 Hz gyro, ~165 Hz accel) leave prop
    energy near multiples of 250 Hz, which folds to near-DC: attitude wander and
    phantom vertical acceleration. May be part of the vibration bias the 3rd
    vertical-bias state absorbs.
    Fix: feed the window mean (Σω·dt, Σa·dt); costs ~dt/2 latency.
15. **✔ "Estimator degraded" is mag-only and forces FAILSAFE** —
    `src/est/sensor_fusion.c:40-87` with `bmx160.c:1467`
    (`estimator_mark_sample(!_is_mag_invalid)`). 100 ms of mag rejects sends a
    flying craft to FAILSAFE though roll/pitch don't need mag, while a NaN
    attitude never raises the flag. `ekf.c:275-282,366-377` accepts NaN input
    (a NaN accel passes the trust gate) and nothing resets the EKF at runtime.
    Fix: base `degraded` on gyro/accel validity and filter health; treat mag loss
    as skip-the-mag-update; reject non-finite inputs and re-init on non-finite
    `q`/`trace(P)`.

## Medium

### Storage / sys

- **✔ `fs_sync_call` timeout leaves a live request** —
  `src/storage/fs_owner.c:619-633`. After the 2 s wait the FS task still runs
  the request, writing into the caller's returned stack frame and pushing a
  stale result the next caller pops as its own.
  Fix: sequence-tag requests and drop mismatches; read into an FS-owned buffer.
- **Save retry reorders** — `fs_owner.c:307-319`. A failed save is requeued
  behind newer saves, so an older `pid.bin` overwrites a newer one.
- **Non-atomic persistent saves** — `fs_owner.c:292-304`, `pid_config.c:86-99`,
  `bmx160.c:231-250`. Truncate-then-write with no temp file or CRC; power loss
  mid-save wipes the tune/calibration.
  Fix: write a temp file, sync, rename; add a CRC32 header.
- **`system_state_set` not atomic** — `src/sys/state.c:51-68`. Six-plus tasks at
  prio 0/1 read-check-write; a preempted rc_ibus can store ARMED over a
  FAILSAFE set in between. Fix: critical section.
- **No build-time RAM bound** — `linker.ld:62-66`, `vaios_app_config.h:75`. The
  `_heap_start + HEAP_SIZE ≤ 0x20018000` rule omits the MSP that grows down from
  the same top; current headroom for main stack + nested ISR frames is ~12.9 KB,
  and ~2.6 KB more `.bss` cuts into the nominal 10 KB. `MAIN_STACK_SIZE` is
  unused. Fix: `--defsym` + linker `ASSERT`.
- **Blackbox ring restarts at 0 every boot** — `fs_owner.c:227-229,253-263`. The
  previous flight's newest (crash) records are the first overwritten; no
  header/seq marks the boundary.
- **`s_wa_pending` increment after push** — `fs_owner.c:831-835`. The FS task
  can commit and clamp the decrement first; pending sticks at 1 and the upload
  never reports DONE.
- **HSL seq resets on an unreadable header** — `imu_hs_log.c:790-799`. Old
  higher-seq frames then sort after new data.
- **`hover_store.c:32-39` calls `vfs_*` directly** from the vertical task,
  breaking the single-FS-owner rule (`fs_owner.c:490-497` documents this
  corrupting FatFS on hardware).

### Sensors / calibration

- **✔ PMU "normal" codes wrong** — `include/driver/bmx160.h:30-32` use
  `0x02<<n`; the datasheet encodes normal as `01`. `bmx160_verify_pmu` can never
  pass (~1.1 s wasted at boot; result ignored).
- **Online gyro-bias tracker never runs** — `bmx160.c:1388-1392` tests stillness
  on the gyro *before* bias subtraction; any raw bias > 0.2 dps fails it. Also
  gated on state rather than `_calib_active`.
- **BMM150 left in normal mode** — `bmx160.c:472-473` (verify on hardware); mag
  likely refreshes at ~10 Hz while `_mag_fresh` fires on repeats. The comment
  says forced mode; `BMX_MAG_ODR` is unused.
- **Mag fit committed on poor coverage** — `src/calib/calib_engine.c:129-133`.
  At the 20 s cap the fit is saved regardless of per-axis coverage.

### Estimation / DSP

- **No baro staleness tracking** — `src/est/vertical_task.c:127-136`. A stalled
  BME280 keeps `valid` true; `accel_unhealthy` can't latch; altitude and climb
  rate free-integrate while height hold trusts them.
- **ToF velocity compared at the wrong time** —
  `src/est/vertical_estimator.c:160-177`. The backward difference describes the
  previous interval (~24 ms old + ~30 ms ranging) but is compared with the
  current climb rate, biasing the estimate during takeoff.
- **Notch slots swap between peaks** — `src/dsp/notch_bank.c:444-453`. Slot *i*
  takes the *i*-th strongest peak, so similar peaks swap and notches jump,
  kicking the D term.

### Comm

- **Publish-before-fill races** — `navlink_xfer.c:174-189` (`state` set before
  `provider`; a slice expiry calls a NULL `provider->open`) and
  `fs_query.c:137-194` (`active` before `opened`/`path`).
- **Comm-task teardown races xfer emission** — `navlink_xfer.c:246-257,221-225`.
  DONE/ABORT frees the session while `tick_download` is in `provider->read`;
  the cursor-snapshot guard itself is a non-atomic compare-and-store.
- **TX reservation incomplete** — `navlink_xfer_tx.c:88-121`,
  `navlink_xfer.c:404-408,555-557`. Open ACK and terminal upload ACK go through
  the capped `write_channel`, once, return ignored; dropped under congestion
  with no idempotent recovery.
- **`XFER_OPEN.arg[32]` not NUL-terminated** — `navlink_router.c:219-220`, then
  used as a C string by the providers.

## Sim (SITL)

- **✔ Duplicated, drifted arm logic** — `sim/host/src/host_rtos_engine.c:127-135`,
  `host_rc_feeder.c:196-209` vs `firmware/src/comm/rc_task.c:81-97`. Disarms only
  from ARMED/FAILSAFE (not IN_AIR, so the kill switch is ignored after takeoff);
  skips `arm_preconditions_met`, throttle failsafe, deadband and the RC-loss
  watchdog. SITL does not model real arming.
  Fix: run `rc_task` in SITL via its `VAYU_SIM` path.
- **Fabricated hover RC on port failure** — `host_rc_feeder.c:196-208`. Arm
  2000, throttle 1300, `is_failsafe=false`; RC-loss failsafe is untestable.
- **IMU fed at 1 kHz vs firmware's 2 kHz** — `host_rtos_engine.c:53,143` vs
  `variables.h:292`. `ATTITUDE_DECIM=8` then runs the EKF at 125 Hz / 8 ms in
  SITL. May contribute to the SITL pitch limit cycle.
- **0.3 s accel ground-hold after any contact** — `sim_controller.cpp:79-84`,
  including liftoff; blanks the first 300 ms of climb and landing impacts.
- **Motors silently dead if the PWM FIFO fails to open** — `host_navhal.c:188-193`
  updates `pwm_latest[]` only after the FIFO opens.
- **Task-context collision** — `host_rtos_port.c:58,89-91` indexes by
  `task_id % 32` while ids only increase; ~23 calibrations in one process
  overwrite a live context.
- **Threads bypass the single-threaded stepper** — the rc_feeder and uart2 RX
  pthreads call `system_state_*`, `spsc_write` and the packet callback
  concurrently with firmware tasks; critical sections are counters only.
- **Pose FIFO serves stale ground truth** — `host_rtos_main.c:659,724` opens
  `O_RDWR`; a full pipe keeps the oldest ~7.6 s.

## Low (abridged)

- `navlink_xfer.c:216` `offset + len` wrap marks an upload complete early;
  `:194-200` `len` 248–255 stalls the upload.
- `channel.c:136` casts baud to `uint16_t` (115200 → 49664); `:271` comment says
  DMA2_Stream6, code uses Stream7; the buffer comment no longer matches the cap.
- `mixer.c:419` yaw desaturation can reverse yaw rather than shrink it.
- `sysid.c:83-98` restart writes `s_mode` before `s_amp` while active.
- `timer_callbacks.c:43-45` uint32 `last_call` vs uint64 counter wraps after
  ~5 days.
- `sys_utils.c:36,84,142` time-sync double/uint64 read-modify-write without a
  critical section.
- `boot.c:43-50` SD check is a stub; clock-check FAILSAFE clears on switch-down.
- `esc.c:57-61` NaN throttle → CCR 0 (mixer already guards).
- `bme280.c:306-313`, VL53L0X: DMA ISR can tear the sample copy; ToF status code
  not filtered.
- `calib_ellipsoid.c:199` `w<=0` passes NaN; loaded calibration not bounds- or
  finite-checked.
- `bme280.c:126-127` `dig_H4`/`dig_H5` not sign-extended; `bmx160.c:439`
  `dig_xyz1` missing `& 0x7F`.
- `host_vfs.c` `vfs_write` count underflows past the cap;
  `rtos_engine_enable_serial_rc` not idempotent; `trimesh_bvh.h:278-295`
  unchecked offsets.
- Complementary filter (dead while `SF_FILTER_USED==SF_EKF`): wrong
  tilt-compensation terms and no yaw unwrap (`sensor_fusion.c:133-188`).

## Comment/code mismatches

- `ekf.h:453` says roll/pitch are seeded from the first accel sample; `ekf.c:373`
  starts from identity.
- `ekf.c:285`, `sensor_fusion.c:119`, `vertical_estimator.h:220` call specific
  force the "gravity vector"; `(0,0,-1)` is called world-down (it is up in NED).
- `maths_interface.h:420-421` `to_radians`/`to_degrees` don't parenthesise the
  argument.
- `gyro_notch.c:173` says it sees raw gyro; it gets post-LPF, post-deadband gyro.
- Mag remap comment `[-Y, X, Z]` vs code `(-Y, -X, -Z)` (`bmx160.c:1195-1198`).
- `BMX160_INT_STATUS_1_ADDR` is 0x1B; datasheet says 0x1D (`bmx160.h:67-68`).
- `fs_owner.c:56` says payload 84 B; it is 116 B.
- `angle_rate_controller.c:595` says airmode DISABLED is the default; code uses
  RP.
- `variables.h` HSL_FILE_SIZE comment: "every arm rewinds to offset 0" and
  "~22 min" are no longer true (~13 min with all streams).
- `sys/clock.h`, `clock.c`, the F10 doc: `vayu_clock_hz()` is derived from RCC
  registers with a hard-coded 8 MHz HSE, not measured; a wrong crystal is
  undetectable.
- Sim: `VSIM_IMU_FRAME_BYTES` says 76 B (real payload 88 B); pose tick "8 kHz"
  counts 1 ms steps; groundClamp also zeroes yaw; `Mat3::inverse` returns plain
  identity; gust is sinusoidal, not 1-cos.

## Checked and correct

- IMU config: `bmx160_init` now writes ACC_CONF 0x2C, ACC_RANGE 0x0C (±16 g),
  GYR_CONF 0x2C, GYR_RANGE 0x00 (±2000 dps); scale factors derive from the same
  codes; `_Static_assert`s lock them. Axis remap is a proper rotation.
- MEKF: Hamilton product, right-error Φ, accel H, Joseph form + symmetrise; mag
  update sign is equivalent. Mahony, RBJ notch, radix-2 FFT/rfft, 3rd-order
  vertical gains correct. dt clamped to [1e-4, 0.1] s; no double promotion.
- Control: derivative on measurement, D-LPF α, `sat_excess` anti-windup,
  integrator reset on arm edge, recovery throttle override applied last.
- HSL SPSC rings are safe (producers ≥1 above the FS task); all SD paths are
  8.3; rate_ctl stack is 1536.
- Sim: quaternion kinematics, gravity +Z NED, specific force = a − g, yaw
  reaction-torque and r×F signs, baro ISA inverse, ESC band, seqlock fences,
  88 B IMU layout asserts.

## Suggested fix order

1. Critical 1–3 (a few lines each).
2. 4–6 — the height-hold and low-throttle class that caused the 2026-09-04 flip.
3. Armed gates on geometry, upload and delete (8, 10, 11).
4. Degraded-flag / NaN handling / baro staleness (7, 15, baro item).
5. SITL arm-logic drift and IMU rate, so SITL results can be trusted for the
   rest.
