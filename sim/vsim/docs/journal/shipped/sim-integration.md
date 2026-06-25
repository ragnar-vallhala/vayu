# Roadmap — firmware ↔ GCS sim integration

> **Largely shipped.** The headline goal — a stable **arm → fly** SITL loop
> (clean takeoff, altitude-hold, waypoint) — is delivered via the **headless SDK**
> (`navigator/headless-sdk/`) and the daemonised in-app sim. Living record:
> `docs/changelog/gcs-in-app-simulator-and-world-collision.md` and the headless
> SDK's own `README.md`/`PLAN.md`. The referenced `build/HANDOFF.md` no longer
> exists, and the in-process `vayu_sitl_start` / `vsim_iface` UART2-callback notes
> below describe the pre-daemon embedding (since replaced by `vsim_d` spawned by
> `SimWorker`). The completed items are collapsed; only the **genuinely-open**
> items (#5 RC-calibration wizard, #6 `MAX_ANGLE_CUTOFF` provenance) remain live.

**Goal (delivered):** a stable, reproducible SITL loop you can **arm and fly**:
`vsim_d` physics ↔ firmware ↔ Navigator (telemetry, render, RC, commands).
The current wire is the standalone `vsim_d` daemon over
`/tmp/vsim_{pwm,imu,pose,ctl}` FIFOs spawned by `navigator/src/vsim/SimWorker`.

Status: ⬜ todo · 🟡 in progress · ✅ done · ⛔ blocked

## Open items

| # | Item | Owner | Status | Notes |
|---|------|-------|--------|-------|
| 5 | RC calibration wizard | GCS | ⬜ | auto-detect axes/direction; ends manual-mapping friction. |
| 6 | Verify `MAX_ANGLE_CUTOFF` (45→70) provenance | firmware | ⬜ | Long-dangling uncommitted tuning of the angle failsafe; confirm intended before committing. |

<details>
<summary>Completed integration pass (historical — references the pre-daemon in-process embedding and the removed build/HANDOFF.md)</summary>

## Done (this integration pass)
- ✅ Firmware ↔ GCS **bidirectional NavLink** over the UART2 pty: telemetry
  out (all packet types, heartbeat ~1 Hz) + commands in (raw pty, RX reader
  thread, `comm_processor_task`). `vayu_sitl` reaches STANDDBY.
- ✅ SITL harness committed (`sim/host`, `d97b80d`); rebuilds against
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
| 1 | **Per-instance FIFO paths** (`VSIM_FIFO_SUFFIX`) | fw shim ✅ / vsim_d ✅ / GCS ✅ | ✅ | HANDOFF §5.1. **fw side**: `host_navhal` + `host_imu_feeder` append `$VSIM_FIFO_SUFFIX` to `/tmp/vsim_{pwm,imu}` + `/tmp/vayu_uart2_{pty,log}`. **vsim_d**: `suffixed()` now applies it to all four FIFOs + the singleton lock (`main.cpp`). **GCS**: `SimulatorWidget::startInAppSim` publishes `_nav<pid>` (respecting an externally-set value) before `vayu_sitl_start` + spawning `vsim_d`; `SimWorker` opens the suffixed pose/ctl. Verified: unset → base paths; `=_smoke` → all suffixed, base untouched. |
| 2 | **RC timing vs COMM-RC-002** | firmware | ⬜ | HANDOFF §5.4 — `RcBridge` is 50 Hz (20 ms); confirm it never false-trips the 100 ms `rc_loss` / 1.0 s failsafe (incl. jitter). |
| 3 | **Arm → fly bring-up** (single clean instance) | integration | 🟡 (#1 unblocked) | HANDOFF §5.2 — with valid IMU + software-arm + throttle, validate stable hover. **Restart-freeze fixed**: on the 2nd Start the pose FIFO file is a leftover, so `SimWorker::openFifos` returned before `vsim_d` opened it as a writer; the read loop got `read()==0` (writer-less EOF), mistook it for "daemon died", and `killDaemon()`'d the just-spawned `vsim_d` — leaving zero `vsim_d`, frozen pose, and the imu feeder stuck in `reopening` (frozen IMU, σ=0). Read loop now tolerates startup EOF until the producer connects (5 s grace). **Operational gotcha**: in-app SITL telemetry reaches the GCS via the `vsim_iface` UART2 callback (no serial needed), but the ARM button writes to `m_serial` (the "Connect" QSerialPort). The SITL firmware's command RX is its **pty** (`cat /tmp/vayu_uart2_pty` → `/dev/pts/N`), driven by `host_navhal`'s `uart2_rx_thread` → `uart2_packet_recv_callback`. So you must point "Connect" at that pty — connecting to `ttyUSB0` (or leaving it unconnected) means `CMD_ARM` never reaches the firmware. UX fix worth doing: auto-connect / route the ARM command through the iface so the two channels are unified. |
| 4 | **Control tuning vs airframe MoI/mass** | firmware (`src/control`) | ⬜ | HANDOFF §5.2 — if it tumbles when armed, retune rate/angle PIDs against the configured inertia. |
| 7 | **Wire ARM button → `CMD_ARM`** | GCS | ✅ | USB-HID gives only 4 channels (no arm switch). fw accepts `CMD_ARM`/`CMD_DISARM`; `MainToolbar` button now sends them via `MainWindow::onArmClicked`, enabled on connect, label follows FC telemetry state. |

(Items 5 and 6 are the remaining open work — promoted to the "Open items" table above.)

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

## Contract: `VSIM_FIFO_SUFFIX` (per-instance isolation) — DONE
All three components now agree on the suffix, sourced from one env var
(default empty == legacy shared paths):
1. **GCS** — `SimulatorWidget::startInAppSim` publishes the suffix
   (`_nav<pid>`, or an externally-set `VSIM_FIFO_SUFFIX`) **before**
   `vayu_sitl_start` (the firmware shim caches paths on first use) and before
   `SimWorker` spawns `vsim_d` (inherits it via `environ`).
2. **firmware shim** — `host_navhal` / `host_imu_feeder` `path_suffixed()`
   append it to `/tmp/vsim_{pwm,imu}` + `/tmp/vayu_uart2_{pty,log}`.
3. **`vsim_d`** — `suffixed()` appends it to all four FIFOs
   (`/tmp/vsim_{pwm,imu,pose,ctl}$SUFFIX`) **and** the singleton lock
   (`/tmp/vsim_d.lock$SUFFIX`, so instances don't fight over one lock).
4. **`SimWorker`** — opens the suffixed `pose`/`ctl` FIFOs.

This was the root of frozen IMU + 0 Hz attitude: a firmware launched with a
suffix waited on a FIFO `vsim_d` (base paths) never fed. With all three
coordinated, multiple Navigators can now run isolated rather than colliding on
the globals.

## Critical path
#1 (clean IMU) → #3 (arm→fly) → #4 (tuning if divergent). #2 in parallel.

</details>
