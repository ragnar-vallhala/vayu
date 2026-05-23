# Roadmap — Vayu firmware in Renode ↔ Gazebo SITL

Closed-loop SITL: same ARM ELF that flashes to the Nucleo-F401RE runs in
Renode, exchanging sensor and actuator data with Gazebo Harmonic for
physics. No firmware carve-outs beyond a single `VAYU_SIM` task swap.

## Current state

| Phase | Component                                                | Status                                |
|-------|----------------------------------------------------------|---------------------------------------|
| P1    | Firmware boots in Renode, DMA-driven SDIO works          | done                                  |
| P2    | iBus RC injection (`usart6_mock.py` + `ibus_inject.py`)  | done                                  |
| P3    | Gazebo Harmonic + `vayu_quad.sdf` world                  | done                                  |
| P4a   | Gazebo IMU → FIFO → vayu (`gz_imu_to_vayu.py`)           | code present, unverified end-to-end   |
| P4b   | vayu TIM1 PWM → FIFO → Gazebo motors                     | Gazebo half done; Renode half sketch  |
| P5L   | IMU sysbus peripheral + `bmx160_sim.c` sim task          | code present, unverified end-to-end   |
| P5H   | Full I²C + BMX160 register-level mock                    | not started                           |

## Milestones

### M1 — Open-loop sensor path (P5L + P4a verification)

Get a Gazebo IMU sample to surface inside vayu's sensor fusion telemetry.
No motors yet.

- Bring up `vayu.resc`, confirm `imu_inject_mock.py` peripheral
  initializes its FIFO at `/tmp/vayu_imu.fifo`.
- Run `gz_imu_to_vayu.py` with the Gazebo world running; confirm IMU
  message count climbs.
- Confirm `bmx160_sim_task` is built under `VAYU_SIM=ON` and
  `imu_queue_telemetry_push` carries Gazebo data — read via UART2
  telemetry log at `/tmp/vayu_uart2.log`.

**Exit criterion:** tilt the model in Gazebo (rotate `base_link` via
`gz service`), see vayu's attitude estimate respond in telemetry.

### M2 — Motor path (P4b Renode-side)

The missing half: TIM1 CCR1..CCR4 writes → `/tmp/vayu_pwm.fifo`.

- Add a `Python.PythonPeripheral` (mirroring `usart6_mock.py`) at TIM1's
  base (`0x40010000`) covering CCR1..CCR4 (`0x34`..`0x40`).
- Watchpoint hooks (as sketched in `vayu_pwm_to_gz.py`'s docstring) would
  work but reopen the FIFO every write — kills performance at 400 Hz ×
  4 motors. A peripheral keeps the fd open.
- Verify against `hal_pwm_init` to confirm ARR for the F401 @ 84 MHz,
  400 Hz PWM. The "210000" in the sketch is a guess; `readelf` or a
  debug breakpoint will tell us the actual value.

**Exit criterion:** arm vayu via iBus injection, watch
`vayu_pwm_to_gz.py` print non-zero duties and rotors spin in Gazebo.

### M3 — Loop closure smoke test

- Run all four processes (gz sim, renode, IMU bridge, PWM bridge)
  together. Manual throttle via iBus. Use **`gz sim -s -r
  --headless-rendering`** (no GUI window) — the GUI tries DRI2/EGL and
  wedges on this host's hybrid NVIDIA/AMD driver stack.

**Exit criterion:** throttle stick → rotors spin → IMU sees acceleration
→ vayu's `angle_controller` reacts. No oscillation or divergence within
5 s.

**Status:** plumbing verified, control chain blocked on a firmware bug.

*Plumbing (verified):*
- Gazebo headless ↔ Renode coexist for >10 min, no GPU lockup.
- `gz_imu_to_vayu.py` feeds IMU at 200 Hz (gravity ≈ +9.81 m/s² Z).
- `pwm_extract` peripheral → `/tmp/vayu_pwm.fifo` → `vayu_pwm_to_gz.py`
  → `gz.msgs.Actuators` exercised end-to-end (rotors visibly spin in
  Gazebo when motor outputs are forced from inside `motor_task`).
- iBus arming sequence (`ibus_inject.py --source=arm`) drives vayu
  through STANDBY → ARMED; verified by reading
  `_system_current_status` via `sysbus ReadWord 0x200006de` → `0x0010`
  (ARMED).

*Fixes needed for the sim build to reach STANDBY at all:*
- `src/sys/boot.c` — skip the `hal_clock_get_sysclk() == SYS_CLOCK_FREQ`
  check under `VAYU_SIM`. Renode's RCC model doesn't track the firmware's
  PLL config, so it always reports HSI (16 MHz) and the boot check would
  fail, putting vayu in FAILSAFE.
- `src/comm/channel.c` — force the UART2 TX polling path under
  `VAYU_SIM`. Renode's `STM32_UART` doesn't honor CR3.DMAT (telemetry
  goes nowhere on the DMA path). Polling works; throughput is reduced
  and Renode's `FileBackend` buffers heavily but the data does flow.

*Fixes landed for the cascaded controller chain to run under SIM:*

- `src/main.c:72-76` — bumped `angle_controller_task` /
  `angle_rate_controller_task` / `motor_task` stack sizes to 8 KB /
  8 KB / 4 KB. With the previous 2 KB / 5 KB / 2 KB sizes, the PID
  state + telemetry struct overflowed and clobbered an `spsc_fifo_t`'s
  `buffer` field to the kernel's stack-poison pattern (0xCFCFCFCF) —
  observable as hundreds of "non-existing peripheral at 0xCFCFxxxx"
  warnings from inside `memcpy` (PC 0x800FD24..0x800FD6A). The bump
  makes the warnings vanish and the PID loops iterate.
- `src/sensor/bmx160.c` — under `VAYU_SIM`, initialize
  `_bmx_orientation.q` to identity. The hardware init path at line
  ~227 was the only place that set it, but `bmx160_init` returns early
  under sim, so the quaternion started at (0,0,0,0). The mahony filter
  then divides by zero norm and emits NaN attitudes, which trip
  `MAX_ANGLE_CUTOFF` and push vayu into FAILSAFE before it can arm.
  (Aside: `bmx160_sim.c` doesn't run sensor fusion at all right now —
  it just feeds the raw queues. Phase 5 Heavy fixes that. For now,
  attitude stays at its initial value of zero, which is fine for the
  M3 hover test.)

*Latent USART6 mock byte-drop bug (worked around, not fixed):*

`tools/sim_renode/usart6_mock.py` mangles incoming iBus frames so that
~half of each 32-byte packet's bytes never reach the firmware's DMA
buffer. The parsed channel values cycle between aligned (correct) and
shifted (looks like RC failsafe), enough to occasionally arm but also
enough to spuriously trip STANDBY→FAILSAFE. The workaround for the M3
test is `volatile sim_rc_channels[14]` + `sim_rc_enabled` in
`src/comm/rc_task.c` — when set, `rc_ibus_task` reads channels
directly from these globals and skips iBus parsing entirely.
`tools/sim_renode/sim_rc_inject.py` is the host-side companion that
writes them through Renode's monitor `sysbus` commands.

*Open issue — Renode time-source wedges:*

After several seconds of running with all four processes (Gazebo, Renode,
both bridges, iBus or sim_rc), Renode's local time source frequently
enters `State: WaitingForReportBack` and **stops advancing virtual
time** even though host time keeps moving and the CPU is not halted.
Closed-loop motor commands stop reaching `pwm_extract` at that point.
Hypothesis: one of the Python peripheral mocks
(usart6/sdmmc/imu_inject/pwm_extract) is not returning from a
`LocalTimeSource.ExecuteInNearestSyncedState` callback under some
condition. Reproduce by running the full stack and querying
`machine GetTimeSourceInfo` repeatedly — virtual time pins at a fixed
value, host time keeps climbing. Not pursued further this session.

*Plumbing validation (kept in source as a sim-only override):*

Earlier in the session, an `#ifdef VAYU_SIM` override in
`motor_task` that bypassed the controller and walked
`motor_outputs.m{1-4}` through a slow ramp **produced visible rotor
spin in Gazebo** (`vel_rad_s=[0 420 426 432]` from
`vayu_pwm_to_gz.py`). That confirms the *entire* downstream pipeline
(`motor_task` → ESC → `hal_pwm_set_duty_cycle` →
`pwm_extract_mock` → `/tmp/vayu_pwm.fifo` → bridge → Gazebo
Actuators) is functional. The override has been reverted; what's
missing is just the cascaded controller chain delivering non-zero
motor commands when armed.

*Debug counters left in the firmware (gated by `VAYU_SIM`):*

| Symbol | Address (rebuild may shift) | Meaning |
|---|---|---|
| `sim_rc_enabled` | 0x20001a30 | 1 = use `sim_rc_channels` |
| `sim_rc_channels[14]` | 0x20000550 | direct RC input bypass |
| `dbg_rc_iter` | 0x20001a34 | rc_task sim_rc branch hits |
| `dbg_motor_iter` | varies | motor_task while-body entries |
| `dbg_motor_ready_seen` | varies | motor_ready was true |
| `dbg_motor_armed_seen` | varies | state was ARMED |
| `dbg_motor_m1_post` | varies | motor_outputs.m1 (last) |
| `dbg_ang_ctrl_iter` | varies | angle_controller iterations |
| `dbg_ang_pop_ok` | varies | successful rc_queue pops |
| `dbg_ang_target_throttle` | varies | normalized ch3 throttle |
| `dbg_ang_rc_ch2` | varies | last ch[2] seen |
| `dbg_rate_ctrl_iter` | varies | rate_controller iterations |
| `dbg_rate_ang_pop_ok` | varies | successful angle pops |
| `dbg_rate_target_throttle` | varies | last target_throttle |
| `dbg_rate_m1` | varies | last computed m1 |

Refresh addresses with `arm-none-eabi-readelf -s build/main | grep -E
'^.{18}OBJECT.*dbg_'`.

*Next steps when M3 is picked up again:*

1. **Renode-wedge investigation** — instrument each Python peripheral
   with a heartbeat log inside its time-sync callback to find which
   mock is hanging. Likely candidate: `usart6_mock.py`'s
   `_pump_and_reschedule`.
2. **USART6 mock byte-drop** — `tools/sim_renode/usart6_mock.py`
   loses ~half of each iBus frame. Likely a chunk-size or
   `total_rx_bytes` arithmetic bug; the right fix is probably to
   write whole bytes-objects with `sb.WriteBytes` instead of a
   per-char loop, after which `sim_rc_enabled` can be deleted.
3. **Replace the `bmx160_sim.c` shim with the mahony filter call**
   so the controller actually closes against the IMU.

### M4 — Time sync (likely needed before M3 will be stable)

Renode runs at "real time" by default; Gazebo defaults to RTF 1.0 but
slips under load. Drift means the control loop sees stale IMU and vice
versa.

- **Option A (simple):** pin both to RTF 1.0 and accept jitter. Probably
  works for hover, fails for aggressive maneuvers.
- **Option B (lockstep):** drive Gazebo with `--iterations` per Renode
  quantum boundary via `gz service /world/.../control` step. Higher
  fidelity, more plumbing.

Recommend A first; escalate to B only if M3 is unstable from sync
drift.

### M5 — Frame & unit audit

One-time check that is cheap to do now and expensive to debug later:

- BMX160 body-frame axes vs Gazebo `base_link` axes (sign + axis
  mapping).
- Gyro: vayu expects dps; bridge converts rad/s — already done.
- Mag: Tesla → µT — already done.
- Accel sign: vayu's sensor fusion expects +Z up with gravity reading
  +9.81 m/s² when stationary — confirm Gazebo's IMU plugin matches.
- ESC throttle 0..1 → rotor velocity scale (`MAX_ROT_VEL_RAD_S = 800`)
  must match the `MulticopterMotorModel` config in `vayu_quad.sdf`.

### M6 — Scenarios

- **Idle** (no throttle): no drift, attitude stays level.
- **Hover:** throttle to 0.5, vayu holds attitude against Gazebo's
  small numerical jitter.
- **Step input:** roll command via iBus → drone tilts → controller
  returns to level.
- **Disturbance:** nudge the model in Gazebo, vayu rejects.

### M7 — Phase 5 Heavy (stretch, post-loop)

Replace `bmx160_sim.c` task with a real I²C BMX160 register mock so the
firmware runs the same `bmx160_initiate_read` path on hardware and in
sim. Removes the last `VAYU_SIM` carve-out from the sensor path.
Multi-day; only worth doing once M3–M6 are green.

## Suggested order

Start with **M1** — smallest verifiable thing, unblocks M3, surfaces any
P5L bugs latent since the deferred verification.

## References

- `tools/sim_renode/README.md` — Phase 1, 2, 5L details
- `tools/sim_gazebo/README.md` — Phase 3 install & world details
- `tools/sim_gazebo/gz_imu_to_vayu.py` — Phase 4a
- `tools/sim_gazebo/vayu_pwm_to_gz.py` — Phase 4b (Gazebo half)
