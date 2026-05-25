# In-app simulator

Replaces the Gazebo / Python-bridge SITL with a single-binary C++/OpenGL
simulator embedded inside the Navigator GCS. The vayu firmware now
runs as a static library linked into the same process as the GCS, with
a shared in-process channel (`vsim_iface_t`) carrying motor PWM out of
the firmware and IMU samples in.

This document is the v1 design write-up + operator's manual: what
exists, how it fits together, how to run it, how to read the logs it
produces, and what's known-broken / next to do.

---

## Why we threw Gazebo out

Three iterations of Gazebo Harmonic + Python bridges + `MulticopterMotorModel`
fought us repeatedly with:

- bullet-featherstone contact instabilities (random 2400 dps gyro
  spikes from a body resting on the ground),
- an X3 Fuel model never designed for arbitrary base-link wrench
  application,
- CoM/skid pendulum interactions, NaN propagation through wrench
  messages, and physics that didn't match the rigid-body equations the
  PID was tuned against.

Each fix surfaced the next mismatch. ArduPilot's own default SITL is a
~1500-line C++ rigid-body integrator with no physics engine for the
same reason. We took that path.

---

## Architecture

```
                  ┌──────────────────────────────────────────┐
                  │  Navigator (single executable)           │
                  │                                          │
   ┌── RC ────────┼─→ vayu firmware (libvayu_sitl_core.a)    │
   │ /dev/ttyUSB0 │     - vaios tasks on pthreads            │
   │              │     - angle / rate / motor / telemetry   │
   │              │                                          │
   │              │     ↓ PWM           ↑ IMU                │
   │              │   ┌────── vsim_iface_t ──────┐           │
   │              │   │ float motor_duty[4]      │           │
   │              │   │ uint8  imu_frame[76]     │           │
   │              │   │ mutex+cond, uart2 cb     │           │
   │              │   └──────────────────────────┘           │
   │              │     ↑ PWM           ↓ IMU                │
   │              │                                          │
   │              │  vsim::SimWorker (QThread, 1 kHz)        │
   │              │   - PhysicsCore (6-DOF RK4 in NED)       │
   │              │   - MotorModel  (per-rotor 1st order)    │
   │              │   - SensorModels (IMU + mag + noise)     │
   │              │                                          │
   │              │  vsim::SimRendererWidget (QOpenGLWidget) │
   │              │   - real GL 3.3 core, VAO/VBO/shaders    │
   │              │                                          │
   │              │  SimulatorWidget                         │
   │              │   - Start/Stop, log dir, motor monitor   │
   │              │   - dataReceived(QByteArray) → existing  │
   │              │     DroneProtocol parser                 │
   └──────────────┴──────────────────────────────────────────┘
```

### Process / thread model

There is exactly one process. Inside it:

| Thread                  | Owner                  | Cadence | Job                                  |
|------------------------ |------------------------|---------|---------------------------------------|
| Qt main / GUI           | QCoreApplication       | event   | UI + signal/slot dispatch             |
| `SimWorker`             | QThread                | 1 kHz   | physics + sensor synth + iface IO     |
| firmware vaios tasks    | `task_create` pthreads | varies  | PID, motor task, telemetry, …         |
| `host_imu_feeder`       | pthread                | event   | cond_wait on iface, push to imu queue |
| `host_rc_feeder`        | pthread                | ~50 Hz  | reads RC from `/dev/ttyUSB0`          |
| `hf_timer_thread`       | pthread                | 1 kHz   | drives `_time_stamp_high_freq` from clock_monotonic |

No `/tmp/` FIFOs, no Python, no Gazebo, no ROS.

### Coordinate frame

Everything in `vsim/` is **NED** (+X north, +Y east, +Z down,
gravity = +9.81 in +Z) to match what the BMX160 driver expects on
real hardware. The bridge code in the old `tools/sim_gazebo/`
applied an FLU→NED swap; the in-app sim models in NED natively so
no transform is needed at the sensor boundary.

### What's outside the GCS process

- The standalone `vayu_sitl` binary still builds (legacy mode, uses
  `/tmp/vayu_pwm.fifo` + `/tmp/vayu_imu.fifo` + the UART2 pty). Useful
  only for offline testing without the GCS — there's no producer for
  the IMU FIFO in that mode unless you wire one externally.
- The RC link is still serial — `/dev/ttyUSB0` from the `sim_bridge.ino`
  microcontroller. The iface does **not** intercept RC. Override the
  device path with `VAYU_UART_RC_PATH=/dev/...`.

---

## File map

### New (in this work)

| File | Purpose |
|------|---------|
| `software/src/vsim/VsimTypes.h`         | Vec3/Quat aliases, `RigidBodyState`, `DroneParams`, `MotorParams`, `ImuSample` |
| `software/src/vsim/PhysicsCore.{h,cpp}` | 6-DOF RK4 integrator, ground clamp |
| `software/src/vsim/MotorModel.{h,cpp}`  | Per-rotor first-order spin-up + thrust/torque |
| `software/src/vsim/SensorModels.{h,cpp}` | IMU/mag/temp with Gaussian noise + clipped bias walk |
| `software/src/vsim/SimController.{h,cpp}` | composes the above into one `tick(duty,dt) → ImuSample` |
| `software/src/vsim/SimWorker.{h,cpp}`   | QThread driver, iface IO, snapshot signal |
| `software/src/vsim/SimRendererWidget.{h,cpp}` | OpenGL 3.3 core renderer (drone + grid + axes) |
| `tools/sim_host/include/vsim_iface.h`   | Shared firmware↔host iface (motor duty, IMU sample, UART2 cb, lifecycle API) |
| `tools/sim_host/src/vsim_iface.c`       | iface impl + global pointer |
| `tools/sim_host/src/host_lifecycle.c`   | `vayu_sitl_start/stop/set_passthrough`, hf timer thread |
| `tools/sim_log_to_csv.py`               | decode `sim-*.bin` → per-stream CSVs |
| `tools/sim_log_plot.py`                 | PID tuning analysis report (PDF + text) |
| `docs/in-app-sim.md`                    | this file |

### Modified

| File | Change |
|------|--------|
| `software/CMakeLists.txt`                  | adds `vsim/*` sources, links `vayu_sitl_core` via add_subdirectory |
| `software/src/ui/main/MainWindow.cpp`      | wires `SimulatorWidget::dataReceived` → `DroneProtocol::processData` |
| `software/src/ui/widgets/SimulatorWidget.{h,cpp}` | rewritten around in-app sim; Gazebo / IMU bridge / PWM bridge / vayu_sitl QProcess rows are gone |
| `tools/sim_host/CMakeLists.txt`            | builds `libvayu_sitl_core.a` + thin `vayu_sitl` binary; exports include dirs PUBLIC |
| `tools/sim_host/src/host_main.c`           | slimmed to a 30-line wrapper that calls `vayu_sitl_start(NULL)` |
| `tools/sim_host/src/host_navhal.c`         | PWM writes go to iface when set (else FIFO); UART2 bytes go to iface callback when set (else pty + raw log) |
| `tools/sim_host/src/host_imu_feeder.c`     | iface cond_wait when iface is set; FIFO read otherwise |
| `src/control/angle_rate_controller.c`      | `MOTOR_IDLE_FLOOR` clamp + PID authority ramp (kept from earlier work) |
| `include/variables.h`                      | `MOTOR_IDLE_FLOOR`, `PID_FULL_AUTHORITY_THROTTLE` constants |

### Orphaned (kept on disk, no longer wired up)

- `tools/sim_gazebo/` — `vayu_quad.sdf`, `gz_imu_to_vayu.py`,
  `vayu_pwm_to_gz.py`, `README.md`. Nothing in the active code path
  references these anymore. Safe to `git rm -rf tools/sim_gazebo/`
  when ready; left around for now so we have something to diff
  against if the in-app sim regresses.

---

## How to build

```bash
# Navigator (in-app sim + GCS, one binary):
cd software
cmake -B build -S .
cmake --build build
# produces software/build/Navigator and (transitively) software/build/vayu_sitl/libvayu_sitl_core.a

# Standalone vayu_sitl (legacy FIFO mode, optional):
cd ..
cmake -S tools/sim_host -B build_sitl
cmake --build build_sitl
# produces build_sitl/vayu_sitl + build_sitl/libvayu_sitl_core.a
```

`-fno-strict-aliasing -Wall -Wextra -O2 -g` everywhere. C11 for the
firmware-side code, C++17 for the GCS.

---

## How to run

1. Connect the RC receiver MCU (`sim_bridge.ino`) to a USB port. The
   feeder defaults to `/dev/ttyUSB0`; export `VAYU_UART_RC_PATH` if it
   landed somewhere else. Without RC the firmware falls back to a
   synthetic hover frame and never arms — useful for some bring-up
   testing but boring.
2. Launch the GCS: `./software/build/Navigator`.
3. Navigate to the **Simulator** page (the home screen lists it).
4. (Optional) edit the **Log dir** field. Defaults to `<repo>/logs/`.
5. Click **Start** (top-right or in-card). The first click:
   - opens `<log dir>/sim-YYYY-MM-DD_HH-MM-SS.bin` for raw UART2 capture,
   - boots the firmware via `vayu_sitl_start(&iface)` — this spawns all
     vaios task pthreads inside the GCS process,
   - starts the `SimWorker` physics thread.
6. Watch the GL panel. The drone should sit flat on the ground at
   spawn (z = -0.05 m in NED — barely above the ground plane). Arm
   it with your transmitter (SwA up + low throttle), then push
   throttle past ~55% to lift off.
7. The existing telemetry panels (Attitude, IMU, Log, Motor, Control
   Loop, Packet Analyzer) light up off `SimulatorWidget::dataReceived`
   exactly as if a real flight controller were plugged in over USB.

### Hard limit: start once per process

`vayu_sitl_start` can be called at most ONCE per Navigator process.
The firmware's vaios "tasks" are detached pthreads with `while(1)`
loops; they have no clean stop hook. Clicking **Stop** pauses the
`SimWorker` (no fresh IMU samples) and closes the log file, but the
firmware pthreads keep running. **Restart Navigator for a true reset.**

---

## Log analysis

Every run writes one raw-bytes `.bin` to the log directory.

### sim_log_to_csv.py — per-stream CSVs

```
./tools/sim_log_to_csv.py logs/sim-2026-05-26_00-01-04.bin
```

Produces `logs/sim-2026-05-26_00-01-04_csv/` with:

| CSV | Columns |
|-----|---------|
| `heartbeat.csv` | `t_ms,t_s` |
| `imu.csv`       | `t_ms,t_s,kind,ax,ay,az,gx_dps,gy_dps,gz_dps,mx,my,mz,temp` (reconstructed from FULL keyframes + fp16 deltas) |
| `attitude.csv`  | `t_ms,t_s,roll_deg,pitch_deg,yaw_deg` |
| `rc.csv`        | `t_ms,t_s,ch0_us..chN_us` |
| `motor.csv`     | `t_ms,t_s,m1,m2,m3,m4` (0..1 linear duty, ESC pulse band already stripped) |
| `state.csv`     | `t_ms,t_s,state_value,state_name` |
| `pid.csv`       | 20 cols: `t_ms,t_s` + the 18 floats from `control_telemetry_t` |
| `log.csv`       | `t_ms,t_s,text` |
| `unknown_status.csv` | `t_ms,t_s,origin,payload_hex` — anything we didn't decode |

CRC32 validation uses the STM32 hardware variant (poly 0x04C11DB7,
init 0xFFFFFFFF, no input/output reflection, no final XOR) — same as
the firmware and the GCS itself.

### sim_log_plot.py — PID tuning report

```
./tools/sim_log_plot.py                  # newest log in logs/
./tools/sim_log_plot.py <log.bin>
./tools/sim_log_plot.py --show           # also open interactively
./tools/sim_log_plot.py --no-pdf         # text summary only
```

Writes `<log_stem>.pdf` next to the .bin and prints:

```
States:     STANDBY → ARMED → STANDBY → ARMED → FAILSAFE → STANDBY
Armed for:  38.38 s  (across 2 windows)

PID tracking quality (armed window only):
  axis    angle RMS   angle peak   rate RMS    rate peak
  roll         3.03        30.11      64.04       185.09
  ...

Motor saturation while armed: 35.4% (...)
Loop dt (ms): outer mean=2.08  ...
PID output rail-hit rate (|out| > 0.99) while armed: ...
Tuning hints: ...
```

The figure has six rows:

1. State timeline (colored backdrop continues through every plot)
2. Throttle + 4 motors + saturation marker strip
3. Per-axis angle setpoint (dashed) vs current (solid)
4. Per-axis rate setpoint vs gyro
5. Per-axis PID output with ±1 rails marked
6. Loop dt + raw gyro

Look for:
- **Sustained offset between setpoint and current** → P gain too low.
- **Oscillation around setpoint** → P too high, missing D.
- **Motor saturation > ~10%** + low PID rail-hit → mixing is clipping;
  reduce rate Kp (the script's tuning hint surfaces this).
- **PID rail-hit > a few %** → output limit `OUT_MAX` may be too low,
  or the rate setpoint is unrealistic.

---

## Limitations / known issues

1. **No clean Stop semantics for the firmware.** Restart Navigator
   to reset state. See "Hard limit" above.
2. **Yaw PID disabled** (`DEAFULT_YAW_ANGLE_RATE_KP = 0` in
   `include/variables.h`). Yaw drifts under sim conditions without
   working mag correction; controlled but unbounded. Yaw is excluded
   from the angle FAILSAFE check on purpose for this reason.
3. **PID gains are not tuned for the in-app sim's drone parameters.**
   Default vsim params (`mass=1.0`, `Ixx=Iyy=0.012`, arm 0.13/0.22 m,
   `k_thrust=1.522e-5`, `max_omega=1200`) produce a twitchier airframe
   than what the SIM-build PID gains were calibrated against. Latest
   log shows 35% motor saturation. Tuning is task #29 in the next
   session.
4. **Standalone binary lost its IMU producer.** The standalone
   `vayu_sitl` still opens the IMU FIFO but no external bridge writes
   to it anymore. Either write a new bridge, run only via Navigator,
   or strip the FIFO code from `host_navhal.c` later.
5. **`tools/sim_gazebo/` is orphaned but not deleted.** No active code
   references it. `git rm -rf tools/sim_gazebo/` when sure.

---

## Implementation notes / gotchas

### Timestamp fix

`get_timestamp_unix()` in `src/utils/utils.c` reads
`_time_stamp_high_freq / 10`, where `_time_stamp_high_freq` is bumped
by an ISR at 10 kHz on real hardware. In the host build there's no
ISR. `host_lifecycle.c::hf_timer_thread` runs at ~1 kHz, computes
`elapsed_us / 100` from CLOCK_MONOTONIC since boot, and `catches`
the counter up by repeated `increment_high_freq_timer()` calls. The
counter is bursty (jumps by ~10 per wake) but `get_timestamp_unix()`
divides by 10 to produce ms, so the burstiness is invisible at the
wire layer.

**First version had an unsigned-underflow bug** in the `tv_nsec`
subtraction: when `now.tv_nsec < origin.tv_nsec` the cast to
`uint64_t` produced ~1.8×10¹⁹, the inner loop bumped the counter
that many times, and downstream packets stamped `t = 213,214,113 ms`
(60 hours). Fixed in the second pass with signed subtraction + borrow.

### Motor mixing convention

`MotorParams::pos_b` + `MotorParams::spin` match the vayu firmware's
mixing in `angle_rate_controller.c`:
M1=FR (front-right), M2=RR, M3=RL, M4=FL; M1+M3 CCW, M2+M4 CW. Don't
reorder without also updating `MotorModel.cpp`'s spin signs.

### UART2 callback threading

`host_navhal::uart2_write_bytes` invokes the iface callback on
whichever firmware thread produced the bytes (telemetry task,
flush task, vayu_log path). The trampoline in `SimulatorWidget.cpp`
uses `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` so the
Qt-side dispatch always happens on the GUI thread. Don't touch
widgets from the C callback directly.

### NaN motor outputs

Caught + clamped in `angle_rate_controller.c` after the
anti-saturation block. `motor.m != motor.m` is the portable NaN
check we use (avoids needing `math.h`'s `isnan` in firmware code).

### Idle motor floor

`MOTOR_IDLE_FLOOR = 0.005` while armed. Matches real-ESC behavior
with `MOTOR_STOP=false` (props keep spinning slowly when armed) and
in SITL serves as the "armed-and-alive" signal — bridge's gravity
cancellation logic (now removed but useful pattern) keyed off this.

---

## What's next

- **Tune** drone params + PID gains for the new physics
  (task #29). Start by dropping `DEAFULT_*_ANGLE_RATE_KP` by 1.5–2×
  per the analysis report's hint.
- Optionally implement a per-tick Lua/Python hook in `SimWorker` for
  scripted maneuvers (step inputs for system ID, doublets for tuning).
- Once tuning settles, consider a more realistic drone parameter set
  matching specific real airframe inertias.
- Delete the orphaned `tools/sim_gazebo/` tree.

---

*Session log:* this document captures the state at the end of the
**2026-05-25 → 2026-05-26 in-app-sim session**. See the
corresponding commit for the changeset.
