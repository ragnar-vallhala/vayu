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

## In progress / next
| # | Item | Owner | Status | Notes |
|---|------|-------|--------|-------|
| 1 | **Per-instance FIFO paths** (`VSIM_FIFO_SUFFIX`) | fw shim + GCS `SimWorker` | 🟡 | HANDOFF §5.1 — the NaN-attitude root cause is colliding global `/tmp/vsim_*` paths. fw side reads `$VSIM_FIFO_SUFFIX`; GCS sets it (per-pid). |
| 2 | **RC timing vs COMM-RC-002** | firmware | ⬜ | HANDOFF §5.4 — `RcBridge` is 50 Hz (20 ms); confirm it never false-trips the 100 ms `rc_loss` / 1.0 s failsafe (incl. jitter). |
| 3 | **Arm → fly bring-up** (single clean instance) | integration | ⛔→#1 | HANDOFF §5.2 — with valid IMU + software-arm + throttle, validate stable hover. |
| 4 | **Control tuning vs airframe MoI/mass** | firmware (`src/control`) | ⬜ | HANDOFF §5.2 — if it tumbles when armed, retune rate/angle PIDs against the configured inertia. |
| 5 | RC calibration wizard | GCS | ⬜ | HANDOFF §5.3 — auto-detect axes/direction; ends manual-mapping friction. |
| 6 | Verify `MAX_ANGLE_CUTOFF` (45→70) provenance | firmware | ⬜ | Long-dangling uncommitted tuning of the angle failsafe; confirm intended before committing. |

## Critical path
#1 (clean IMU) → #3 (arm→fly) → #4 (tuning if divergent). #2 in parallel.
