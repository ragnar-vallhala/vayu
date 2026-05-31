# Roadmap — firmware ↔ GCS sim integration

**Goal:** a stable, reproducible SITL loop you can **arm and fly** inside
the Navigator GCS: `vsim_d` physics ↔ firmware (`vayu_sitl` /
`libvayu_sitl_core`) ↔ Navigator (telemetry, render, RC, commands).

**Architecture:** Navigator ⇄ firmware over UART2 NavLink (telemetry +
commands); firmware ⇄ `vsim_d` over `/tmp/vsim_{pwm,imu,pose}` FIFOs;
Navigator drives RC via a pty (`$VAYU_UART_RC_PATH`) and configures the
airframe via the `vsim_d` ctl channel. See
[`../../build/HANDOFF.md`](../../build/HANDOFF.md).

Status: ⬜ todo · 🟡 in progress · ✅ done · ⛔ blocked

## Done (this integration pass)
- ✅ Firmware ↔ GCS **bidirectional NavLink** over the UART2 pty: telemetry
  out (all packet types, heartbeat ~1 Hz) + commands in (raw pty, RX reader
  thread, `comm_processor_task`). `vayu_sitl` reaches STANDDBY.
- ✅ SITL harness committed (`tools/sim_host`, `d97b80d`); rebuilds against
  the GCS instance's `vsim_proto.h` (HANDOFF §5.5).
- ✅ **GCS software-arm** (`CMD_ARM`/`CMD_DISARM`): a `g_sw_arm_request` latch
  OR'd with the physical arm switch via `rc_arm_engaged()`, so a 4-channel
  USB-HID stick (no arm channel) can arm from the GCS. Preconditions
  (throttle <1100, RC healthy, estimator OK) still enforced. fw side tested
  (`test_software_arm_latch`). **GCS side wired**: `MainToolbar` ARM button →
  `MainWindow::onArmClicked` sends `CMD_ARM`/`CMD_DISARM`; button enabled on
  connect and its label (ARM/DISARM) follows the FC state from telemetry.
  **Link verified end-to-end** (`test_arm_command_wire`): the exact frame the
  Navigator emits is run through `deserializer_feed` → `comm_processor_dispatch`
  (extracted from the task loop for testability), flipping `g_sw_arm_request`;
  a bad-CRC frame is dropped and leaves the latch untouched.

## In progress / next
| # | Item | Owner | Status | Notes |
|---|------|-------|--------|-------|
| 1 | **Per-instance FIFO paths** (`VSIM_FIFO_SUFFIX`) | fw shim ✅ / GCS ⬜ | 🟡 | HANDOFF §5.1. **fw side done**: `host_navhal` + `host_imu_feeder` append `$VSIM_FIFO_SUFFIX` to `/tmp/vsim_{pwm,imu}` and `/tmp/vayu_uart2_{pty,log}` (verified: unset = legacy paths; `=_pid42` → all suffixed). **GCS side pending** — see contract below. |
| 2 | **RC timing vs COMM-RC-002** | firmware | ⬜ | HANDOFF §5.4 — `RcBridge` is 50 Hz (20 ms); confirm it never false-trips the 100 ms `rc_loss` / 1.0 s failsafe (incl. jitter). |
| 3 | **Arm → fly bring-up** (single clean instance) | integration | ⛔→#1 | HANDOFF §5.2 — with valid IMU + software-arm + throttle, validate stable hover. |
| 4 | **Control tuning vs airframe MoI/mass** | firmware (`src/control`) | ⬜ | HANDOFF §5.2 — if it tumbles when armed, retune rate/angle PIDs against the configured inertia. |
| 5 | RC calibration wizard | GCS | ⬜ | HANDOFF §5.3 — auto-detect axes/direction; ends manual-mapping friction. |
| 6 | Verify `MAX_ANGLE_CUTOFF` (45→70) provenance | firmware | ⬜ | Long-dangling uncommitted tuning of the angle failsafe; confirm intended before committing. |
| 7 | **Wire ARM button → `CMD_ARM`** | GCS | ✅ | USB-HID gives only 4 channels (no arm switch). fw accepts `CMD_ARM`/`CMD_DISARM`; `MainToolbar` button now sends them via `MainWindow::onArmClicked`, enabled on connect, label follows FC telemetry state. |

## Contract: GCS software-arm (`CMD_ARM` / `CMD_DISARM`) — DONE
The firmware accepts two NavLink commands over the UART2 command channel.
`MainWindow::onArmClicked` sends a `PACKET_TYPE_COMMAND` (0x3) framed exactly
like the `CMD_CALIBRATE_IMU` path (`CalibrationWidget.cpp:269`):
- **payload** = `cmd_id` as 2 little-endian bytes; **no args** (argc absent,
  `length == 2`).
- **`CMD_ARM = 0x0002`** when disarmed, **`CMD_DISARM = 0x0003`** when armed
  (toggle on `m_armed`, which tracks the FC state from `statusReceived`).

Firmware behavior: `CMD_ARM` sets the latch; on the **next RC frame** the arm
preconditions are checked (throttle <1100, RC link healthy, estimator OK) and
STANDBY→ARMED follows. `CMD_DISARM` clears the latch → STANDBY. UI flow:
**lower throttle, then ARM** (arming with throttle up trips FAILSAFE, by
design). The button label follows telemetry (ARMED/IN_AIR/FAILSAFE → "DISARM"),
not the last click, so a failed arm doesn't lie about the state.

## Contract: `VSIM_FIFO_SUFFIX` (per-instance isolation) — GCS side TODO
The firmware shim appends `$VSIM_FIFO_SUFFIX` (default empty) to its base
`/tmp` paths. For one Navigator+firmware+`vsim_d` triple to stay isolated,
**`SimWorker` must**, before `vayu_sitl_start`:
1. choose a unique suffix (e.g. `"_" + getpid()`),
2. `setenv("VSIM_FIFO_SUFFIX", suffix, 1)` so the in-process firmware shim
   picks it up, and
3. launch `vsim_d` so it opens the **same** suffixed FIFOs
   (`/tmp/vsim_pwm$SUFFIX`, `/tmp/vsim_imu$SUFFIX`, `/tmp/vsim_pose$SUFFIX`)
   — via a `vsim_d` arg/env (daemon change, GCS scope).

Until all three agree, keep to **one** Navigator (HANDOFF §5.1 mitigations
— singleton + NaN guards — still apply).

## Critical path
#1 (clean IMU) → #3 (arm→fly) → #4 (tuning if divergent). #2 in parallel.
