# Vayu FC — SITL Stub Inventory

**What this is:** a per-component report of exactly which parts of the Vayu
flight-controller firmware are *stubbed* (replaced by host shims) when the FC is
built for software-in-the-loop simulation, what mechanism each stub uses, and at
what data rate it moves data. Generated 2026-06-23 from
`tools/sim_host/` against the current tree.

## The seam, in one sentence

In SITL the **real firmware control/estimation/comm code is compiled and run
verbatim**; only the *hardware edges* are stubbed — sensors in, actuators out,
RTOS primitives, clock, transport, and persistence. The honest contract
([sitl-seam-contract]) is: the FC sees **only what its sensors tell it (raw IMU,
raw baro, RC) and acts only through PWM out + telemetry**. No privileged state
crosses into the firmware.

### What is REAL (compiled from `src/`, not stubbed)

From `tools/sim_host/CMakeLists.txt` `VAYU_SOURCES`:

- Estimators: `attitude_task.c` (EKF/Mahony), `ekf.c`, `sensor_fusion.c`,
  `vertical_estimator.c` + `vertical_task.c`, `flight_phase.c`.
- Control: `angle_controller.c`, `angle_rate_controller.c`, `pid.c`,
  `pid_config.c`, `flight_mode.c`, `control_buffer.c`, `lpf.c`.
- Actuator math: `motor.c`, `esc.c` (the mixer + ESC throttle mapping run for real).
- Comm/telemetry chain: `channel.c`, `serializer.c`, `comm_processor.c`,
  `navlink_router.c`, `navlink_tx.c`, `telemetry_task.c`, `perf_packet.c`.
- Sensor *logic*: `imu_buffer.c`, `bme280.c` (altitude derivation/getters),
  `rc_buffer.c`, `rc_safety.c` (arm gate + watchdog).
- Sys: `state.c`, `assert.c`, `logger.c`, `log_text.c`, `math_utils.c`,
  `sys_utils.c`.

Everything below is the part that is **NOT** real — the stubs.

---

## Stub inventory

All shims live in `tools/sim_host/src/`. FIFOs/pty paths are suffixed with
`$VSIM_FIFO_SUFFIX` so concurrent runs don't cross-feed.

### 1. IMU sensor — `host_imu_feeder.c`

| | |
|---|---|
| Replaces | The BMX160 I2C IMU driver (`src/sensor/bmx160.c`, *not* compiled) |
| Mechanism | Dedicated pthread reads framed `vsim_imu_frame_t` from FIFO `/tmp/vsim_imu$SUFFIX` produced by `vsim_d`; pushes **raw** samples into the firmware's three IMU queues: `imu_queue_control_push`, `imu_queue_telemetry_push`, `imu_queue_attitude_push` |
| Wire frame | 16 B `vsim_hdr_t` + 88 B payload (`VSIM_IMU_PAYLOAD_BYTES`, 22 floats: acc/gyr/mag triplets + raw/compensated/fusion mirrors + temp). Gyro is **deg/s on the wire**, accel m/s², mag µT |
| **Data rate** | **1000 Hz** (`SITL_IMU_FEED_HZ`, must track `vsim_d`'s `kImuHz = 1000`) |
| Time model | Each sample is stamped with a **fixed** sim-time increment (`SYS_CLOCK_FREQ/1000` cycles), *not* wall-clock — this is the lockstep clock source. After queuing it calls `host_clock_advance_us(1000)` (advances virtual time 1 ms/sample) and bumps the 10 kHz HF timestamp counter by its per-sample share |
| Fidelity notes | Injects **raw IMU only** — the firmware's own `attitude_task`/EKF estimates attitude (a past version ran a host-side Mahony and bypassed the estimator; that violation is fixed). **Known gap:** firmware `IMU_SAMPLE_FREQ_HZ = 2000` but vsim emits 1000 Hz |
| RTOS variant | `host_imu_feeder_open/pump` do a single synchronous read for the cooperative stepper instead of running the thread |

### 2. Barometer — `host_baro.c`

| | |
|---|---|
| Replaces | The I2C transport for the BME280 (real `bme280.c` *logic* is kept) |
| Mechanism | pthread reads framed `vsim_baro_frame_t` from `/tmp/vsim_baro$SUFFIX`, then calls the **real** `bme280_publish(pressure_pa, temperature_c, humidity_rh)` — the FC derives altitude and emits BARO telemetry through its normal path. Also defines no-op `i2c_manager_write/read/write_read` stubs so `bme280.c`'s HW entry points link |
| Wire frame | 16 B header + 12 B payload (3 floats: static pressure Pa, air temp °C, RH %) |
| **Data rate** | Driven by `vsim_d`'s baro emit (modelled physical pressure); consumed as fast as frames arrive. (FC re-emits BARO telemetry at ~10 Hz, see §9) |
| Fidelity notes | No host-side altitude — physical reading in, FC math runs. Mirrors the IMU hand-over pattern |

### 3. RC / radio input — `host_rc_feeder.c`

| | |
|---|---|
| Replaces | iBus/PPM RX on UART6 (`src/comm/rc_task.c` parser path) |
| Mechanism | pthread reads **CSV lines** (`roll,pitch,throttle,yaw,SwA,…` in µs) from the serial port at `$VAYU_UART_RC_PATH` (default `/dev/ttyUSB0`, or a pty driven by the harness). Parses to `ibus_data_t.channels[0..13]`, runs the **real** arm/disarm state machine (`apply_arm_logic`, mirrors `rc_task`), pushes to `rc_queue_control_push` + `rc_queue_telemetry_push` |
| Fallback | If the port is absent, synthesizes a hover frame (roll/pitch/yaw 1500, throttle 1300, SwA 2000=arm) and keeps retrying; on >500 ms silence emits a failsafe frame |
| **Data rate** | ~**50 Hz** when fed by `sim_bridge.ino` / harness; ≥2 Hz required or the feeder goes failsafe. Synthetic-fallback pushes paced by 20 ms real-time retry |
| Time model | Uses `host_wall_delay_ms` (**real** time, not sim time) for serial I/O backoff — it waits on an external device, so it must not block the virtual clock |

### 4. Motor / PWM output — `host_navhal.c` (PWM HAL)

| | |
|---|---|
| Replaces | STM32 TIM1 4-channel PWM peripheral (`hal_pwm_*`) |
| Mechanism | `hal_pwm_set_duty_cycle` writes a `vsim_pwm_frame_t` snapshot of **all four** motor duties to FIFO `/tmp/vsim_pwm$SUFFIX` (opened `O_RDWR\|O_NONBLOCK` so writes never block); `vsim_d` does latest-wins. `host_pwm_get_latest()` lets the in-process RTOS stepper read duties back without a FIFO round-trip |
| ESC band strip | The firmware's `esc_set_throttle` maps motor 0..1 → 1..2 ms pulse on a 2.5 ms (400 Hz) period = duty 0.4..0.8. The shim **strips that band** back to a linear 0..1 motor command: `cmd = (duty-0.4)/(0.8-0.4)`, because there is no real ESC in sim |
| Wire frame | 16 B header + 16 B payload (4 floats, duty 0..1) |
| **Data rate** | One frame **per `hal_pwm_set_duty_cycle` call** = once per motor per control iteration; in practice paced by the rate loop (~1 kHz control / motor_task) |

### 5. Telemetry transport (UART2, FC→GCS) — `host_navhal.c` (UART HAL)

| | |
|---|---|
| Replaces | UART2 DMA TX/RX for the GCS link |
| Mechanism | Two modes. **(a) In-process iface:** when Navigator registers a `vsim_iface_t`, `hal_uart_write_dma` hands bytes to `iface->on_uart2_bytes` callback. **(b) Legacy/standalone:** opens a **pty** (`posix_openpt`, raw mode), advertises the slave path to `/tmp/vayu_uart2_pty$SUFFIX`, and tees a raw copy to `/tmp/vayu_uart2.log$SUFFIX`. A reader thread on the pty master delivers GCS→FC bytes one at a time via `uart2_packet_recv_callback` + `hal_uart_read_char` (emulating the per-byte RX IRQ) |
| DMA fakery | `hal_uart_write_dma` is synchronous, so it immediately calls `dma_tx_complete_callback()` to clear `channel.c`'s busy flag (no real TX-complete IRQ). `hal_uart_init_dma_rx` / idle-callback / `hal_uart_dma_rx_index` are no-ops (RC uses the bypass, not DMA-RX) |
| Baud | `UART_BAUDRATE = 230400` configured but not rate-limiting on host (pty is memory-speed) |
| **Data rate** | The *content* is produced by the real `telemetry_task` at ~166 Hz tick (`v_delay(6)`), emitting per-message: ImuCompressed 25 Hz, Motor/PIDerr 18 Hz, log 15 Hz, Attitude/RC/Baro/Vertical 10 Hz, Status 2 Hz, full-state + cost + heartbeat ~1 Hz |

### 6. CRC peripheral — `host_navhal.c` (CRC HAL)

| | |
|---|---|
| Replaces | STM32 hardware CRC unit |
| Mechanism | `hal_crc_compute` reimplements the **STM32 CRC32 variant** in software (poly `0x04C11DB7`, init `0xFFFFFFFF`, MSB-first, no reflection, no final XOR) so SITL packets pass the GCS's checksum gate (which rejects all packets failing it) |
| Data rate | Per outgoing/incoming packet (call-driven) |

### 7. Clock / cycle counter / timers — `host_navhal.c` + `host_clock.h`

| | |
|---|---|
| Replaces | SYSCLK config, DWT cycle counter, TIM-based HF timestamp |
| Mechanism | `hal_clock_get_sysclk` returns fixed **84 MHz** (AHB 84, APB1 42, APB2 84); `hal_cycle_counter_cycles_per_us` = 84. `hal_cycle_counter_get` reads `CLOCK_MONOTONIC` **anchored at process start** (scaled to 84 MHz) so the firmware's "first sample, small now" assumption holds and doesn't trip the angle-cutoff failsafe. `hal_timer_*` are no-ops |
| Virtual sim clock | The vaios delay family blocks on a **virtual clock** advanced only by the IMU feeder (one 1 ms step/sample) — lockstep, enables faster-than-realtime. `host_wall_delay_ms` is the escape hatch for host infrastructure that must pace on real time (process-alive loop, serial backoff) |
| HF timestamp | 10 kHz (`HIGH_FREQ_TIMER_FREQ`) counter driven off the virtual clock by the IMU feeder, replacing the wall-clock timer ISR |

### 8. GPIO / interrupts — `host_navhal.c`

| | |
|---|---|
| Replaces | STM32 GPIO + NVIC |
| Mechanism | **All no-ops / return HAL_OK.** `hal_gpio_*`, `hal_interrupt_*`, `hal_disable/enable_global_interrupts` (both legacy and new names). Also no-op stubs for `calibration_task` and `bmx160_calib_request_cancel` (calibration flow lives in the uncompiled `bmx160.c`) |
| Data rate | n/a |

### 9. RTOS kernel (vaios) — `host_vaios.c` (default pthread build)

| | |
|---|---|
| Replaces | The vaios scheduler, tasks, IPC, heap |
| Mechanism | `task_create` → **one detached pthread per task** (priority ignored, min 64 KB stack). `v_semaphore_*` → POSIX `sem_t`; `v_mutex_*` → `pthread_mutex_t`; `v_malloc/v_free` → libc; `v_panic` → abort; `scheduler_*`/`v_init`/`v_start` → no-ops. `v_delay`/`task_delay`/`task_delay_until` block on the **virtual clock**; `v_get_ticks` reads it (1 tick = 1 ms) |
| Limitation | Tasks are `while(1)` detached pthreads with no clean stop — `vayu_sitl_start()` is once-per-process; `vayu_sitl_stop()` only quiesces (no fresh IMU → loops idle) |
| RTOS variant | The opt-in `vayu_sitl_rtos` build compiles these **out** (`-DVAYU_SITL_RTOS`) and runs the **real vaios scheduler** on a ucontext port (`host_rtos_port.c`/`host_rtos_main.c`) for maximal fidelity + determinism |

### 10. Flash / SD persistence (VFS) — `host_vfs.c`

| | |
|---|---|
| Replaces | SD-card-backed vaios VFS (PID tune `0:pid.bin`, IMU calibration) |
| Mechanism | Small in-memory file table (8 files × 4 KB) **mirrored to real files** under `$VAYU_VFS_DIR` (default `/tmp/vayu_vfs/`): loaded on open, flushed on write/close/sync — so a `CMD_SET_PID` tune survives a restart, matching the real save→reboot→reload contract |
| Data rate | Call-driven (config persistence only) |

### 11. CPU port globals — `host_port.c`

| | |
|---|---|
| Replaces | Cortex-M critical-section primitives |
| Mechanism | Defines `host_critical_mutex` (pthread mutex) + `critical_nesting`. `CORTEX_M4=1` is defined only to satisfy `vaios/task.h`'s arch `#error`; the ARM bits are shimmed |

---

## Producer side (`vsim_d`) emit rates — for reference

The physics daemon (`tools/vsim/src/main.cpp`, single thread, three rates) is the
*producer* the sensor stubs read from:

- **8000 Hz** — physics RK4 tick + drain PWM FIFO + drain ctl FIFO (`kPhysicsHz`)
- **1000 Hz** — emit IMU frame (`kImuHz`, drives the firmware loop & virtual clock)
- **60 Hz** — emit pose/ground-truth frame (`kPoseHz`) — consumed by the
  harness/Pilot, **never** by the FC
- Baro frames emitted on the baro FIFO (physical pressure)
- All runtime-tunable via `VSIM_CTL_SET_RATES`

Wire protocol: `tools/vsim/include/vsim_proto.h`, `VSIM_PROTO_VERSION 3`,
little-endian, 16 B `vsim_hdr_t` (magic/version/type/payload_bytes/seq_no) on
every frame. Rebuild both `vsim_d` and the SITL on any wire change.

## Quick map: HW edge → stub file

| HW edge | Stub file | Transport | Rate |
|---|---|---|---|
| IMU (BMX160) | `host_imu_feeder.c` | FIFO `/tmp/vsim_imu` in | 1000 Hz |
| Baro (BME280) | `host_baro.c` → `bme280_publish` | FIFO `/tmp/vsim_baro` in | physical, FC re-emits ~10 Hz |
| RC (iBus/PPM) | `host_rc_feeder.c` | serial/pty CSV in | ~50 Hz |
| Motors (PWM TIM1) | `host_navhal.c` | FIFO `/tmp/vsim_pwm` out | per control iter (~1 kHz) |
| GCS link (UART2) | `host_navhal.c` | iface callback or pty | content ~166 Hz tick |
| CRC unit | `host_navhal.c` | software CRC32 | per packet |
| Clock/DWT/timers | `host_navhal.c` + `host_clock.h` | virtual sim clock | 1 ms/IMU sample |
| GPIO/NVIC | `host_navhal.c` | no-ops | n/a |
| RTOS (vaios) | `host_vaios.c` | pthreads / POSIX | n/a |
| Flash/SD (VFS) | `host_vfs.c` | disk-backed `/tmp/vayu_vfs` | call-driven |
| CPU critical sect. | `host_port.c` | pthread mutex | n/a |

## Related
- [[sitl-architecture]] · [[sitl-seam-contract]] · [[sitl-test-harness]]
- `docs/plans/sitl-lockstep-sim.md` (virtual clock / lockstep)
- `software/headless-sdk/docs/FINDINGS.md` (open estimator/rate-fidelity gaps)
