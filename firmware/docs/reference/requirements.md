# Vayu — Firmware Requirements

Status: living document. Defines what the vayu flight-control firmware
must do, by module, with bidirectional traceability to source and tests.
Companion to [`coding-guidelines.md`](coding-guidelines.md).

Adapted from a PX4-grade requirements template. Borrows the
template's three-level structure, MOD-SUB-NNN ID format, verification
methods, and traceability discipline; replaces the template's coarse
HAL/RTOS/AP/SYS taxonomy with vayu's actual nine-module decomposition.

Seven sections:

1. **Requirement structure** — levels, ID format, fields, verification.
2. **Module taxonomy** — the nine module prefixes and what each covers.
3. **System requirements (SYS)** — vehicle-level behaviour.
4. **Module requirements** — HLR + LLR per module, with reserved ID ranges.
5. **Traceability** — `@implements` / `@verifies` tags + CI gate.
6. **Audit snapshot** — reality vs. spec drift and the cleanup backlog
   captured from the 2026-05-26 code audit.
7. **Maintenance rules** — how the doc stays honest.

The GCS-side companion lives at
[`navigator/docs/requirements.md`](../../../navigator/docs/reference/requirements.md);
SYS-level requirements that span firmware ↔ GCS are owned here, with the
GCS doc referring back to them.

Cross-references:
- [`sensor-fusion/`](sensor-fusion/) — attitude estimator math.
- [`navlink messages`](../../../navlink/docs/reference/messages/) — wire format authority.
- [`state-machine/`](state-machine/) — system state diagrams.
- [`coordinate_ref.md`](coordinate_ref.md) — NED conventions.
- [`gcs-in-app-simulator-and-world-collision.md`](../../../navigator/docs/journal/changelog/gcs-in-app-simulator-and-world-collision.md) — SITL design (`vsim_d` daemon).

---

## 1. Requirement structure

### 1.1 Levels

| Level | Scope                                                                  | Audience           |
|-------|------------------------------------------------------------------------|--------------------|
| SYS   | What the vehicle must do from the operator's point of view. Cross-module. | All stakeholders. |
| HLR   | What a module must do to satisfy its parent SYS or sibling HLR.       | Module owner.      |
| LLR   | How a module is internally structured (algorithms, data flow, timing). | Implementer.       |

Every requirement has exactly five fields:

- **ID** — `MOD-SUB-NNN`, unique, never reused after deletion.
- **Title** — short noun phrase.
- **Statement** — single testable claim. Imperative voice ("shall …").
- **Rationale** — *why*. The reason a future maintainer can use to judge edge cases.
- **Parent** — the higher-level requirement(s) this satisfies. Use `(top level)` for SYS root entries.
- **Verification** — one or more of: Test · Analysis · Inspection · Demonstration.

### 1.2 ID format

```
MOD-SUB-NNN
    MOD  : module prefix (SYS | HAL | VOS | SNS | EST | CTRL | ACT | COMM | LOG)
    SUB  : 3-letter sub-area (e.g. IMU, SCHED, RATE, MIX, ARM, …)
    NNN  : 3-digit number. Range 001–099 for HLR, 101–199 for LLR.

Examples: SYS-SAFE-001 · HAL-IMU-001 · CTRL-RATE-101 · COMM-RC-002
```

**Rules:**

- A deleted ID is never reused — leave the row with `❌ (dropped: <reason>)`.
- New IDs go at the end of their sub-area block.
- HLR range is `001–099`; LLR range is `101–199`. Sub-areas under a
  module never collide because the SUB token differentiates them.

### 1.3 Verification methods

| Method        | Definition                                                                          | Typical venue                                         |
|---------------|-------------------------------------------------------------------------------------|-------------------------------------------------------|
| Test          | Automated unit, integration, or HIL/SITL test that pass/fails on a numeric assertion. | host unit (`navtest`), Renode emulation, SITL bridge.|
| Analysis      | Static analysis, formal proof, hand calculation with documented assumptions.        | `clang-tidy`, `cppcheck` (MISRA addon), CBMC, hand notes. |
| Inspection    | Code review against an explicit checklist; reviewer signs off.                      | PR review with checklist comment.                     |
| Demonstration | Observed behaviour in a defined scenario, usually a flight test or full-system bench. | Test flight; bench rig.                              |

A requirement may name **multiple** methods (e.g. `Test (SITL) + Demonstration (flight test)`).

---

## 2. Module taxonomy

Nine prefixes, mapped to the actual source layout:

| Prefix | Domain                                       | Source location(s)                                              | Vendor / owned |
|--------|----------------------------------------------|-----------------------------------------------------------------|----------------|
| `SYS`  | Vehicle-level / cross-cutting behaviour.     | `src/sys/`, top-level integration.                              | owned          |
| `HAL`  | Hardware Abstraction Layer (drivers, MCU peripherals). | `extern/vaios/extern/NavHAL/`                          | vendored       |
| `VOS`  | RTOS — scheduler, IPC, memory, time.         | `extern/vaios/kernel/`, `extern/vaios/portable/cortex-m4/`      | vendored       |
| `SNS`  | Sensor drivers + sample buffering.            | `src/sensor/` (BMX160, IMU buffer, I2C manager)                 | owned          |
| `EST`  | State estimation (attitude / position).      | `src/est/` (sensor_fusion, lpf)                                 | owned          |
| `CTRL` | Control loops + mixing + PID.                | `src/control/` (controllers, PID core, control buffer)         | owned          |
| `ACT`  | Actuator output (motors, ESCs).              | `src/actuator/` (esc, motor)                                    | owned          |
| `COMM` | Communications — RC ingest + telemetry tx/rx. | `src/comm/`, `include/comm/`                                   | owned          |
| `LOG`  | On-device logging subsystem.                  | `src/logger/`                                                   | owned          |

**Sub-area conventions** (the `SUB` token in `MOD-SUB-NNN`):

| Module | Sub-areas (illustrative)                                                            |
|--------|-------------------------------------------------------------------------------------|
| SYS    | SAFE (safety), TIM (timing), CAL (calibration), PWR (power), STATE (state machine), TEL (telemetry contract), HMI (annunciation) |
| HAL    | I2C, SPI, UART, PWM, CRC, DMA, GPIO, TIM, IRQ, FLASH                                |
| VOS    | SCHED, IPC, MEM, ISR, TIME, WD (watchdog)                                           |
| SNS    | IMU, MAG, BARO, BUF (buffering)                                                     |
| EST    | MAH (Mahony filter), BIAS, COV (convergence), ALT (vertical estimate), PHS (flight phase) |
| CTRL   | RATE (inner loop), ANGLE (outer loop), MIX (motor mix), PID, ARM (arming logic), SID (system-ID) |
| ACT    | MOT (motor), ESC (PWM/DShot), FAIL (failsafe outputs)                                |
| COMM   | RC, TEL (telemetry), PKT (packet layer), CRC, HB (heartbeat), XFER (bulk transfer), FS (filesystem nav), SYNC (time sync) |
| LOG    | TXT (in-memory text log), RATE (max rate), PERSIST (future SD/flash), OWN (fs owner), XFER (bulk transfer), FS (filesystem nav) |

**Vendor scope clause.** `HAL` and `VOS` are vendored (vaios + navhal
submodules). We treat their requirements as *contracts we depend on*,
not as code we ship. When a HAL/VOS HLR is needed we record it here for
visibility, but verification points at the vendor repo's own tests
(`extern/vaios/tests/`, `extern/vaios/extern/NavHAL/tests/`). The CI
trace check is allowed to find HAL/VOS requirements implemented only in
vendored code.

---

## 3. System requirements (SYS)

Vehicle-level requirements. Each has a SYS-level parent of `(top level)`.

> **Scope note.** Vayu currently does manual stabilised flight (rate
> and angle control modes) with no autonomous navigation, no GPS, no
> mission planner, and no autopilot in the PX4 sense. SYS-level
> requirements reflect that. Future autonomy items (RTL, mission,
> position hold) are listed but marked `❌ deferred — out of scope`
> until the firmware grows those subsystems; their IDs are reserved.

### 3.1 Safety (SYS-SAFE)

| ID            | Title                                                                                                                                       | Statement                                                                                                                                            | Parent       | Verification                       |
|---------------|---------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|------------------------------------|
| SYS-SAFE-001  | Emergency disarm                                                                                                                            | The vehicle shall command zero motor output within 200 ms of a kill-switch assertion on RC or telemetry.                                              | (top level)  | Test (SITL) + Demonstration        |
| SYS-SAFE-002  | RC loss behaviour                                                                                                                           | On loss of valid RC frames for > 1.0 s, the vehicle shall transition to the FAILSAFE state and command zero motor output. No autonomous RTL/LAND. **✅ `rc_watchdog_step()` raises FAILSAFE after `RC_LOSS_TIMEOUT_MS` (1.0 s) without a valid frame; motor task commands zero on FAILSAFE (Phase 2a). | (top level)  | Test (SITL) + Demonstration        |
| SYS-SAFE-003  | Sensor-fault failsafe                                                                                                                       | If the attitude estimator flags its state as degraded for > 100 ms, the vehicle shall transition to FAILSAFE. ✅ `estimator_safety_step()` requests FAILSAFE when `estimator_is_degraded()` persists in a flight state (Phase 2b). | (top level)  | Test (SITL with fault injection)   |
| SYS-SAFE-004  | Max attitude failsafe                                                                                                                       | If absolute roll or pitch exceeds `MAX_ANGLE_CUTOFF` (currently 70° per `variables.h:176`) the vehicle shall transition to FAILSAFE within one rate-loop iteration. Yaw is excluded from this check by design (Mahony yaw drift immunity). | (top level)  | Test (SITL)                        |
| SYS-SAFE-005  | Arming preconditions                                                                                                                        | The vehicle shall refuse to transition from STANDBY to ARMED unless: estimator converged, RC valid, calibration current, throttle stick at minimum. **✅ `arm_preconditions_met()` gates STANDBY→ARMED on RC valid + estimator not degraded + throttle minimum (Phase 2d). Calibration-freshness gate still TODO. | (top level)  | Test (unit + SITL)                 |
| SYS-SAFE-006  | State-transition validation                                                                                                                 | All system-state transitions shall be validated against a static allowed-transitions table; out-of-order transitions shall be rejected and logged.    | (top level)  | Test (unit) + Inspection           |
| SYS-SAFE-007  | Time-sync command gate                                                                                                                       | Until the GCS time-sync handshake has applied at least one clock correction, the firmware shall start UNSYNCHRONISED and TEMPORARILY_REJECT every ack-requiring command (spec §10.5); the synced flag shall be set only when a clock offset is actually applied. | (top level)  | Test (unit + SITL) + Inspection    |

### 3.1.1 State machine (SYS-STATE)

The current implementation uses a **bitfield-encoded** state enum
(`include/sys/state.h`). Values are powers of two so multiple states
could in principle be ORed together, but the live state is a single
value; the bitfield representation is exploited by the telemetry layer
for compact transmission.

| ID            | Title                              | Statement                                                                                                                                                                       | Parent         | Verification          |
|---------------|------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------|-----------------------|
| SYS-STATE-001 | State enumeration                  | The vehicle shall maintain a single current state from `{UNINITIALIZED, INIT, STANDBY, PREARM, ARMED, IN_AIR, FAILSAFE, TERMINATED, CALIBRATING}` encoded as a bitfield in `sys_state_t`. | (top level)    | Inspection            |
| SYS-STATE-002 | Read accessor                      | A non-blocking accessor shall return the current state in O(1) time and shall be callable from any task or ISR context.                                                          | (top level)    | Inspection + Test     |
| SYS-STATE-003 | Boot start state                   | On power-on the state shall be `INIT`; on successful completion of boot self-tests (clock + SD + sensors) it shall transition to `STANDBY`, else to `FAILSAFE`.                  | SYS-TIM-001    | Test (bench)          |

### 3.2 Timing (SYS-TIM)

| ID           | Title              | Statement                                                                                                                  | Parent       | Verification          |
|--------------|--------------------|----------------------------------------------------------------------------------------------------------------------------|--------------|-----------------------|
| SYS-TIM-001  | Cold-boot time     | The vehicle shall be ready to arm within 5 s of power-on under nominal sensor conditions.                                  | (top level)  | Test (bench)          |
| SYS-TIM-002  | Rate-loop closure  | The rate-control loop shall close at 1 kHz ±5 % steady-state on the target MCU. **🟡 gap** — currently driven by `v_delay(1)` in `angle_rate_controller.c:334`, not by IMU-sample arrival; effective cadence depends on scheduler jitter. | (top level)  | Test (target log)     |
| SYS-TIM-003  | Outer-loop closure | The angle-control loop shall close at 500 Hz ±10 % steady-state. Currently `v_delay(2)` ⇒ ~500 Hz target.                  | (top level)  | Test (target log)     |
| SYS-TIM-004  | System clock       | The system clock shall be 84 MHz on the target MCU (STM32F401RE @ HSE 8 MHz, PLL M=8 N=336 P=4 Q=7).                       | (top level)  | Inspection + Test     |
| SYS-TIM-005  | High-frequency timer | TIM5 shall fire at 10 kHz and dispatch up to 4 registered callbacks for sub-millisecond periodic work.                   | (top level)  | Inspection + Test     |
| SYS-TIM-006  | Disciplined wall clock | The vehicle shall maintain a Unix-epoch-millisecond wall clock equal to monotonic high-frequency ticks plus a GCS-supplied offset, strictly non-decreasing across corrections, with a 64-bit offset path for cold-start deviations exceeding int32 ms (~24.8 days). | (top level)  | Test (unit, monotonicity under step + slew) |
| SYS-TIM-106  | Slew-limited clock discipline | Clock corrections within `TIME_SYNC_STEP_MS` (200 ms) shall slew toward target at ≤ 50 ms/s (τ 2 s); larger corrections shall step; slew rate shall stay below the 1000 ms/s tick rate to guarantee monotonicity. | SYS-TIM-006  | Test (unit) + Inspection |

### 3.3 Control modes (SYS-CTRL)

| ID            | Title                       | Statement                                                                                                                              | Parent       | Verification                        |
|---------------|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------|--------------|-------------------------------------|
| SYS-CTRL-001  | Manual rate mode (acro)     | The vehicle shall expose a manual rate (acro) mode where stick deflection commands body-rate setpoints scaled to ±360 °/s by default. | (top level)  | Demonstration + Test (SITL)         |
| SYS-CTRL-002  | Stabilised angle mode       | The vehicle shall expose a stabilised mode where roll/pitch sticks command angle setpoints clipped to ±45° by default; yaw is rate.   | (top level)  | Demonstration + Test (SITL)         |
| SYS-CTRL-003  | Mode selection via RC       | Mode shall be selectable via a configured 3-position switch channel on the RC link.                                                   | (top level)  | Test (unit) + Demonstration         |
| SYS-CTRL-004  | GCS flight-mode override     | The firmware shall accept a GCS command (`CMD_SET_FLIGHT_MODE`) that sets a sticky angle/acro override taking priority over the RC mode switch until released, and shall publish the effective mode and its source (RC vs GCS) as telemetry. | SYS-CTRL-003 | Test (unit + SITL)                  |

### 3.4 Telemetry contract (SYS-TEL)

Authoritative wire spec lives in [`navlink messages`](../../../navlink/docs/reference/messages/);
these SYS entries pin the cadence and reliability contract.

| ID           | Title                              | Statement                                                                                                                            | Parent       | Verification         |
|--------------|------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|--------------|----------------------|
| SYS-TEL-001  | Heartbeat cadence                  | The vehicle shall emit a heartbeat packet at ≥ 2 Hz whenever a telemetry transport is connected.                                     | (top level)  | Test (SITL + target) |
| SYS-TEL-002  | IMU stream                         | The vehicle shall emit IMU samples (full or delta-compressed) at ≥ 100 Hz when armed.                                                | (top level)  | Test (SITL + target) |
| SYS-TEL-003  | System-state announcement          | The vehicle shall emit a system-state packet on every state transition and at ≥ 1 Hz steady-state.                                   | (top level)  | Test (SITL)          |
| SYS-TEL-004  | Wire framing                       | All telemetry frames shall use the `0x56 \| type<<4\|ver \| len \| dev_id \| ts[4] \| payload \| CRC32[4]` framing in `docs/telemetry/`. | (top level)  | Inspection           |

### 3.5 Calibration (SYS-CAL)

| ID           | Title                       | Statement                                                                                                                          | Parent       | Verification         |
|--------------|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------|--------------|----------------------|
| SYS-CAL-001  | Gyro bias calibration       | The vehicle shall support a stationary, stillness-gated gyro bias calibration triggered from the GCS, completing within 10 s.       | (top level)  | Test (SITL + bench)  |
| SYS-CAL-002  | Accel calibration           | The vehicle shall support a pose-tolerant accelerometer calibration over ~12 held orientations (6 faces + 6 edges/corners) with per-pose progress feedback to the GCS. | (top level)  | Test (SITL + bench)  |
| SYS-CAL-003  | Magnetometer axis coverage  | The vehicle shall support a magnetometer calibration that reports per-axis sphere coverage to the GCS.                              | (top level)  | Test (SITL + bench)  |
| SYS-CAL-004  | Calibration persistence     | Calibration constants shall survive a power cycle.                                                                                  | (top level)  | Test (bench)         |

### 3.6 Power (SYS-PWR) — *placeholder, not yet implemented*

| ID           | Title                  | Statement                                                                                              | Parent       | Status                                          |
|--------------|------------------------|--------------------------------------------------------------------------------------------------------|--------------|-------------------------------------------------|
| SYS-PWR-001  | Battery monitor input  | The vehicle shall provide a battery voltage reading at ≥ 1 Hz to the telemetry stream.                | (top level)  | ❌ deferred — no battery monitor in current HW. |
| SYS-PWR-002  | Low-battery failsafe   | When battery voltage falls below a configured threshold, the vehicle shall transition to FAILSAFE.    | (top level)  | ❌ deferred — depends on SYS-PWR-001.            |

### 3.7 Human-machine interface (SYS-HMI)

Onboard pilot-facing annunciation (LEDs + buzzer) of the live system
state and boot-fault codes, independent of any GCS link.

| ID           | Title                  | Statement                                                                                                                                                                                          | Parent         | Verification                          |
|--------------|------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------|---------------------------------------|
| SYS-HMI-001  | Visual state annunciation | The vehicle shall continuously annunciate the current system state on onboard LEDs/buzzer, refreshing at ≥ 2 Hz (period clamped ≥ 250 ms), clearing all indicators on each transition, with a distinct per-state pattern. | SYS-STATE-001  | Demonstration (bench) + Inspection    |
| SYS-HMI-101  | Boot-fault annunciation codes | In FAILSAFE the annunciator shall blink red + buzzer and additionally show solid blue for a system-clock-check failure and solid green for an SD-card-check failure, decoded from boot-check flags. | SYS-HMI-001    | Demonstration (bench, fault injection) |

---

## 4. Module requirements

Each module reserves an ID space, defines its scope, and lists seed HLR
and LLR entries. The remainder are added as code matures — IDs are
allocated in order, never reused.

### 4.1 HAL — Hardware Abstraction Layer

**Owner.** Vendored (`extern/vaios/extern/NavHAL/`). Vayu consumes the
v1 API via `navhal.h`.

**Scope.** Memory-mapped register access, MCU peripheral drivers
(I2C/SPI/UART/PWM/CRC/DMA/GPIO/TIM/IRQ/FLASH), bounded interrupt
service routines, monotonic clock.

**Reserved IDs.** `HAL-*-001..099` (HLR), `HAL-*-101..199` (LLR).

#### 4.1.1 High-Level Requirements (HAL-HLR)

| ID            | Title                  | Statement                                                                                                                                                                                 | Parent              | Verification                          |
|---------------|------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------|---------------------------------------|
| HAL-IMU-001   | ❌ (dropped: HAL does not ship an IMU driver. Sensor-rate contract belongs in `SNS-IMU-001`. Dropped during 2026-05-27 cleanup.) | — | — | — |
| HAL-IMU-002   | ❌ (dropped: per-sample validity is a sensor-layer concern, owned by `SNS-IMU-002`. HAL only owes byte-level transport correctness via `HAL-I2C-001`.) | — | — | — |
| HAL-I2C-001   | I²C transport          | The HAL shall expose a fast-mode I²C 1.0 master (≥ 400 kHz SCL) with synchronous read/write/write-then-read primitives, ISR-driven DMA reads, and explicit bus-recovery hooks (manual SCL clocking, peripheral reinit). | SYS-TIM-002         | Test (unit, target HIL)               |
| HAL-DMA-001   | I²C DMA RX             | The HAL shall expose DMA-driven I²C reads on Stream 0, Channel 1, configured `Memory-increment, single-buffer, VERY_HIGH priority`, with a completion-ISR callback that hands control back to the caller. | SYS-TIM-002         | Inspection + Test (target HIL)        |
| HAL-PWM-001   | ESC output sync        | All configured ESC outputs shall update synchronously at a rate ∈ [400 Hz, 8 kHz]. Vayu currently uses 400 Hz on TIM1 channels 1–4.                                                       | SYS-TIM-002         | Test (target HIL, logic analyser)     |
| HAL-CRC-001   | Hardware CRC32         | The HAL shall expose a CRC32 unit configured to match the GCS-side polynomial 0x04C11DB7, MSB-first, init 0xFFFFFFFF.                                                                      | SYS-TEL-004         | Test (unit + vector compare with GCS) |
| HAL-TIME-001  | Monotonic time         | The HAL shall expose a monotonic microsecond counter with drift ≤ ±50 ppm over the operating temperature range.                                                                            | SYS-TIM-002         | Analysis (datasheet) + Test (24 h)    |
| HAL-API-001   | Host-stub portability  | The HAL public API shall be implementable on a POSIX host (`sim/host/`) such that upper layers compile unchanged for SITL.                                                          | (process)           | Inspection + Test (host build CI)     |

#### 4.1.2 Low-Level Requirements (HAL-LLR)

| ID            | Title                   | Statement                                                                                                                           | Parent           | Verification                |
|---------------|-------------------------|-------------------------------------------------------------------------------------------------------------------------------------|------------------|-----------------------------|
| HAL-IMU-101   | ❌ (dropped: BMX160 lives on I²C, not SPI; sensor-specific init contract belongs in `SNS-BMX-101/102` and that row already covers it accurately. Verbatim copy from the PX4-grade sample template that assumed an ICM-42688 SPI sensor.) | — | — | — |
| HAL-IMU-102   | ❌ (dropped: vayu does not use a DRDY interrupt and does not read from a FIFO. The driver uses a DMA-completion semaphore handshake on I²C — see `SNS-BMX-105`. The DRDY+FIFO pattern was inherited from the sample template and does not apply.) | — | — | — |
| HAL-API-101   | Header-only API         | All HAL public symbols shall be declared in `navhal.h` (umbrella) with no transitive include of MCU vendor headers in clients.       | HAL-API-001      | Analysis (grep in CI)        |

### 4.2 VOS — vaios RTOS

**Owner.** Vendored (`extern/vaios/kernel/`).

**Scope.** Task scheduling, IPC (queues/semaphores), memory pools,
ISR-safe API subset, timing, watchdog.

**Reserved IDs.** `VOS-*-001..099`, `VOS-*-101..199`.

#### 4.2.1 VOS-HLR

| ID             | Title                       | Statement                                                                                                  | Parent           | Verification                  |
|----------------|-----------------------------|------------------------------------------------------------------------------------------------------------|------------------|-------------------------------|
| VOS-SCHED-001  | Preemptive priority schedule | The scheduler shall implement fixed-priority preemptive scheduling with at least 8 priority levels.        | SYS-TIM-002      | Test (host port + target stress) |
| VOS-SCHED-002  | Context switch latency      | Worst-case context-switch latency on Cortex-M4 shall not exceed 10 µs.                                     | SYS-TIM-002      | Test (target, GPIO-toggle WCET) |
| VOS-IPC-001    | Message queue API           | The RTOS shall provide a thread-safe bounded-blocking queue API supporting fixed-size items.               | (process)        | Test (unit + target stress)    |
| VOS-MEM-001    | No dynamic allocation after init | After the scheduler is started, no RTOS API call shall perform dynamic memory allocation.             | (process)        | Analysis (heap trace) + Inspection |
| VOS-ISR-001    | ISR-safe API subset         | A defined subset of RTOS APIs shall be callable from interrupt context, documented in the API reference.   | SYS-TIM-002      | Inspection + Test (target)     |
| VOS-WD-001     | Task watchdog               | The RTOS shall provide a per-task software watchdog API that triggers a callback if a task fails to check in within its declared period. | SYS-SAFE-002 | Test (unit + target fault injection) |

#### 4.2.2 VOS-LLR

| ID             | Title                  | Statement                                                                                                          | Parent           | Verification          |
|----------------|------------------------|--------------------------------------------------------------------------------------------------------------------|------------------|-----------------------|
| VOS-IPC-101    | Queue post-from-ISR    | `vaios_queue_post_isr()` shall complete in constant time, never block, and signal whether a context switch is required on ISR exit. | VOS-IPC-001, VOS-ISR-001 | Test (unit) + Analysis (asm review) |
| VOS-MEM-101    | Static task creation   | Task control blocks and stacks shall be supplied from statically declared storage by the caller.                   | VOS-MEM-001      | Inspection            |
| VOS-WD-101     | Watchdog tick source   | The task watchdog shall be evaluated from the systick ISR at 1 kHz and shall not perform any operation that may block. | VOS-WD-001    | Inspection + Test     |

### 4.3 SNS — Sensor drivers

**Scope.** `src/sensor/` (BMX160 driver, IMU buffer, I2C manager —
`drivers/` folded in per Phase 4 R2.6). Concerned with raw sample acquisition + sample buffering,
*not* with fusion or control.

**Reserved IDs.** `SNS-*-001..099`, `SNS-*-101..199`.

**Sub-areas.** BMX (driver), IMU (gyr+acc samples), MAG (magnetometer),
BUF (buffering), CAL (online calibration), I2C (bus manager).

#### 4.3.1 SNS-HLR

| ID            | Title                          | Statement                                                                                                                                                                                       | Parent              | Verification                  |
|---------------|--------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------|-------------------------------|
| SNS-IMU-001   | IMU sample availability        | The IMU subsystem shall deliver synchronised gyroscope + accelerometer samples to the estimator at a configurable rate. Native rate: 1600 Hz BMX160 internal ODR; effective sample rate observed by the rate-control task is gated by the read state machine — see SNS-BMX-104. | HAL-I2C-001, HAL-DMA-001 | Test (SITL + target)        |
| SNS-IMU-002   | Sample validity                | Each IMU sample posted to the estimator queue shall carry a validity flag, false if any of: I2C read error, range over-saturation, or magnetometer disturbance rejection (mag only) occurred.   | SYS-SAFE-003        | Test (unit, fault injection)  |
| SNS-MAG-001   | Mag sample availability        | The magnetometer subsystem shall deliver mag samples at ≥ 100 Hz when present and healthy. Currently effective ≈ 123 Hz (interleaved 1 in 13 fast cycles); BMX160 internal mag ODR is 50 Hz.    | SYS-CAL-003         | Test (SITL + target)          |
| SNS-MAG-002   | Mag disturbance rejection      | The mag subsystem shall flag a sample invalid if any of: rhall ∉ [50, 30000], post-compensation magnitude ∉ [20 µT, 80 µT], or step-to-step disturbance dot-product < 0.75 × ‖m‖².              | SYS-SAFE-003        | Test (unit, fault injection)  |
| SNS-CAL-001   | Persistent calibration store   | Accel offset + 3×3 (scale + cross-axis misalignment, `acc_soft_iron`), gyro offset, and mag hard-iron + soft-iron constants shall persist across power cycles, stored in `0:cal.bin` (1 KB on SD, format v3; a mismatched magic/version/size is rejected on load and compiled-in identity defaults are kept). | SYS-CAL-004         | Test (bench, power-cycle)     |
| SNS-CAL-002   | Online gyro bias estimator     | While not in CALIBRATING and not in motion (gyro norm < 0.2 °/s, acc-jerk-equivalent < 0.2 m/s² for ≥ 200 consecutive samples), the driver shall update gyro bias with α = 0.0034, clamped to ±5 °/s per axis. | (process)           | Test (unit + bench)           |
| SNS-BUF-001   | IMU buffer SPSC ring           | IMU samples and attitude estimates shall flow through SPSC ring buffers with `OVERWRITE` policy on full; capacities: main IMU ring 11, telemetry queue 11, control queue 11, calibration queue 11, calibration-telemetry queue 2. | (process)           | Inspection + Test (unit)      |
| SNS-BUF-002   | Drop accounting                | The IMU buffer shall expose a drop counter incremented on every overwrite, surfaced through telemetry or the LOG channel.                                                                       | (process)           | Test (unit, fault injection)  | ✅ `imu_buffer_drop_count()` increments on each OVERWRITE; emitted in the HEALTH status (Phase 3 SLOG). |
| SNS-I2C-001   | I2C bus contract               | The I2C manager shall provide thread-safe synchronous read/write and ISR-driven async read APIs, recovering from a stuck bus via the 9-clock bit-bang procedure after ≥ 100 consecutive acquire failures. | HAL-I2C-001         | Test (unit + target, fault injection) |
| SNS-BARO-001  | Baro acquisition and publish   | The BME280 (I2C 0x76) shall be read over the shared single-owner IMU DMA loop at ~15 Hz and publish compensated pressure (Pa), temperature (°C), humidity (%RH) and derived altitude; an absent/mis-wired sensor (chip id ≠ 0x60) shall be detected at init and baro reads disabled. | SNS-IMU-001         | Test (SITL + bench)           |

#### 4.3.2 SNS-LLR

| ID            | Title                       | Statement                                                                                                                                                                                       | Parent           | Verification           |
|---------------|-----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|------------------------|
| SNS-BMX-101   | Init sequence               | On `bmx160_init()` the driver shall: read CHIP_ID and verify == 0xD8; issue soft-reset; transition accel + gyro + mag PMUs to NORMAL with up to 5 retries each, polling PMU_STATUS between retries. | SNS-IMU-001      | Test (unit w/ I2C mock + target HIL) |
| SNS-BMX-102   | Sensor ranges               | Accelerometer shall run at ODR 1600 Hz, ±8 g, OSR4; gyroscope at ODR 1600 Hz, ±1000 dps, OSR4; magnetometer at internal 50 Hz ODR.                                                              | SNS-IMU-001      | Inspection             |
| SNS-BMX-103   | Scale factors               | Acc scale = 8 × 9.80665 / 32768 m/s² per LSB; gyr scale = 1000 / 32768 dps per LSB. Computed at init, applied on every sample.                                                                  | SNS-IMU-001      | Inspection + Test (unit) |
| SNS-BMX-104   | Read state machine          | The driver shall interleave reads as: every cycle = gyro+accel; every 13th cycle = magnetometer; the cycle following a mag cycle = temperature. Effective accel/gyro emission rate ≥ native ODR / state-machine overhead. | SNS-IMU-001      | Inspection + Test (target) |
| SNS-BMX-105   | DMA + semaphore handshake   | Sensor reads shall be issued asynchronously via I2C DMA; the BMX160 task shall block on a semaphore given by the DMA completion ISR, with a 50 ms watchdog that triggers I2C unstick + driver reinit. | SNS-I2C-001      | Test (target, fault injection) |
| SNS-BMX-106   | Axis-frame remap            | Driver shall present samples in body frame per `docs/coordinate_ref.md`. Raw → body remap: acc = (−ax, ay, −az), gyr = (−gx, gy, −gz), mag = (−my, −mx, −mz). Documented in a comment block at the top of `bmx160.c`. | (process)        | Inspection             |
| SNS-CAL-101   | Accel ellipsoid calibration | Accel calibration shall accept ~12 prompted-but-pose-tolerant static holds (6 faces + 6 edges/corners; stillness-gated, with pose acceptance/coverage gating specified by SNS-CAL-104), fit a magnitude-only ellipsoid (`calib_fit_ellipsoid`) to recover offset + a full 3×3 matrix, rescale so corrected ‖a‖ = g, and commit only on a successful fit. | SYS-CAL-002      | Test (bench + host unit)  |
| SNS-CAL-102   | Gyro bias calibration       | Gyro calibration shall average `gyr_raw` only while the board is still (gyro + |a| stillness gate), over `GYRO_CAL_STILL_SAMPLES`; it shall time out (keeping the prior offset) if the board never settles and reject the window if its variance exceeds `GYRO_CAL_VAR_MAX`. | SYS-CAL-001      | Test (bench + host unit)  |
| SNS-CAL-103   | Mag free-rotation calibration | Mag calibration shall accumulate compensated samples over ≥ 20 s of free rotation, fit an online least-squares ellipsoid (`calib_fit_ellipsoid`) to recover the hard-iron offset + soft-iron 3×3, track per-axis raw-span coverage for GCS feedback, and commit only on a successful fit. | SYS-CAL-003      | Test (bench)           |
| SNS-MAG-101   | Trim-data compensation       | The driver shall read BMM150 trim data (regs 0x5D..0x70) once at init and apply per-axis temperature-compensated formulas before delivering mag samples.                                       | SNS-MAG-001      | Inspection + Test (unit) |
| SNS-I2C-101   | Bus-acquire timeout          | I2C mutex acquire shall use a 5 ms timeout; on timeout the call shall return `HAL_ERR_TIMEOUT` and shall not block other callers.                                                              | SNS-I2C-001      | Test (unit + target)   |
| SNS-I2C-102   | Unstick procedure            | Bus recovery shall toggle SCL 9 times manually, issue one START/STOP, and re-init the HAL I2C peripheral. Triggered after ≥ 100 consecutive `bus busy` failures.                              | SNS-I2C-001      | Test (target, fault injection) |
| SNS-LPF-101   | First-order LPF              | Sensor channels shall be filtered by a per-channel first-order IIR `y[n] = α·x[n] + (1−α)·y[n−1]` with α = 0.34 (accel), α = 0.51 (gyro), α = 0.0034 (gyro-bias estimator).                  | (process)        | Test (unit, step response) |
| SNS-CAL-104   | Accel pose-coverage gate    | Accel calibration shall accept a still hold only if it advances orientation coverage: a FACE hold must have one body axis dominate (max unit component ≥ `ACCEL_POSE_FACE_DOMINANCE`) and land on a signed axis no prior face used; an EDGE/corner hold must share gravity (second-largest component ≥ `ACCEL_POSE_EDGE_MIN_SECOND`) and sit ≥ ~30° (dot < `ACCEL_POSE_MIN_SEP_COS`) from every banked direction. Duplicate/wrong-shape holds shall be rejected and re-prompted. Matching is geometric (mount-agnostic). | SNS-CAL-101      | Test (host unit) + bench  |
| SNS-BARO-101  | Datasheet compensation      | The driver shall apply Bosch BME280 fixed-point temperature / 64-bit pressure / humidity compensation using factory calibration read once at init. | SNS-BARO-001     | Test (host unit, datasheet reference vectors) |
| SNS-BARO-102  | Altitude derivation         | The driver shall derive altitude from compensated pressure via the ISA formula `h = 44330·(1−(p/p0)^0.190295)` against a settable QNH sea-level reference. | SNS-BARO-001     | Test (host unit)          |

### 4.4 EST — State estimation

**Scope.** Attitude estimator (`src/est/sensor_fusion.c`, Mahony +
complementary filters per `docs/sensor_fusion/`; error-state EKF in
`src/est/ekf.c`), shared linear-algebra/quaternion kernel
(`include/maths/linalg.h`), low-pass filters (`src/est/lpf.c`).
Position estimation is **out of scope** until a GPS / OF / range sensor
arrives.

**Reserved IDs.** `EST-*-001..099`, `EST-*-101..199`.

**Sub-areas.** MAH (Mahony filter), COMP (complementary, present but
unused), EKF (error-state/multiplicative EKF, 6- and 9-state), BIAS
(gyro-bias online estimator — implemented in SNS, not here), COV
(convergence / health monitor).

#### 4.4.1 EST-HLR

| ID           | Title                              | Statement                                                                                                                                                                                                                                                                                              | Parent           | Verification                 |
|--------------|------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|------------------------------|
| EST-MAH-001  | Attitude convergence               | The attitude estimator shall converge to within 5° of truth within 3 s of init with the vehicle stationary.                                                                                                                                                                                            | SYS-TIM-001      | Test (SITL)                  |
| EST-MAH-002  | Fault-sample rejection             | The estimator shall reject any IMU sample whose validity flag is false (per SNS-IMU-002) and shall raise an `estimator_degraded` flag after 100 ms of continuous rejection.                                                                                                                            | SYS-SAFE-003     | Test (unit, fault injection) | ✅ `estimator_mark_sample()` / `estimator_is_degraded()`; bmx160 feeds sample validity + stamps `attitude.degraded` (Phase 2b). |
| EST-MAH-003  | Filter selection                   | The implementation shall expose exactly one active fusion filter at runtime, selected by `SF_FILTER_USED` in `variables.h`. Currently set to `SF_EKF`; `SF_MAHONY` / `SF_COMPLEMENTARY` / `SF_EKF_ACCEL_BIAS` are dead code unless selected.                                                            | (process)        | Inspection                   |
| EST-COV-001  | Estimator output queues            | Attitude estimates shall be posted to two SPSC queues every step: one for telemetry, one for the rate controller. Both have `OVERWRITE` policy.                                                                                                                                                        | (process)        | Inspection                   |
| EST-EKF-001  | EKF attitude estimator             | The estimator shall offer an error-state (multiplicative) EKF attitude filter selectable via `SF_EKF`, estimating attitude and gyro bias and converging to within 5° of truth within 3 s of init with the vehicle stationary.                                                                          | EST-MAH-001      | Test (SITL)                  | ✅ `m_ekf_filter()` / `ekf_init()`; SITL `phase3_est_ekf` + on-target `-DEKF_SELFTEST` (Phase 5). |
| EST-EKF-002  | Accel-bias variant                 | A 9-state variant (`SF_EKF_ACCEL_BIAS`) shall additionally estimate accelerometer bias; the bias is observable only under motion and converges in flight (static bias and tilt are unidentifiable).                                                                                                    | EST-EKF-001      | Test (SITL)                  |
| EST-ALT-001  | Vertical state estimate            | The estimator shall fuse barometric altitude with accelerometer-derived vertical motion into an absolute altitude and climb-rate estimate, published every step to the vertical-state queue (`OVERWRITE`) for the control loop, in-air detector, and telemetry.                                          | (top level)      | Test (unit) + Test (SITL)    |
| EST-PHS-001  | Takeoff / landing detection and AGL reference | The FC shall detect liftoff and touchdown from the fused vertical estimate and commanded throttle, request the ARMED↔IN_AIR transitions, and maintain an authoritative AGL referenced to a ground altitude captured (from raw baro) while disarmed and frozen at arm.                        | (top level)      | Test (unit) + Test (SITL)    |

#### 4.4.2 EST-LLR

| ID            | Title                          | Statement                                                                                                                                                                                  | Parent           | Verification                  |
|---------------|--------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-------------------------------|
| EST-MAH-101   | Quaternion representation      | Attitude shall be represented internally as a unit quaternion `(w,x,y,z)`, normalised every iteration; Euler conversion is for output only.                                                | EST-MAH-001      | Inspection + Test (unit, fuzz) |
| EST-MAH-102   | Mahony gains                   | The filter shall use proportional gain `Kp = 3.0` and integral gain `Ki = 0.0025` from `variables.h:61-62`.                                                                                | EST-MAH-001      | Inspection                    |
| EST-MAH-103   | dt source                      | The filter shall measure dt from the DWT cycle counter (clock 84 MHz), clamped to a minimum of 1 ms to avoid divide-by-zero on hot loops.                                                  | SYS-TIM-004      | Inspection + Test (unit)      |
| EST-MAH-104   | Mag conditional update         | If `‖m‖ < 1e-6` the mag correction term shall be skipped and yaw drift is bounded by gyro integration alone for that step.                                                                 | EST-MAH-001      | Test (unit)                   |
| EST-MAH-105   | Integral feedback bound        | The Mahony integral feedback term `(integralFBx, integralFBy, integralFBz)` shall be clamped per-axis (recommended ±0.5 rad/s) and re-initialised on estimator reset.                       | EST-MAH-001      | Test (unit) + Analysis        | ✅ clamped at ±0.5 rad/s; `estimator_reset()` zeroes the term (Phase 3 CTRL). |
| EST-MAH-106   | Init quaternion                | At init the estimator shall set the quaternion to identity `(1, 0, 0, 0)` and zero the integral feedback state.                                                                            | EST-MAH-001      | Inspection                    |
| EST-COMP-101  | Complementary parameters       | The complementary filter (unused at runtime) shall use α = 0.98 for gyro weighting; documented in code only as a fallback path.                                                            | EST-MAH-003      | Inspection                    |
| EST-EKF-101   | Init / reset / state selection | `ekf_init(estimate_accel_bias)` shall build the covariance for a 6- or 9-state filter; `ekf_reset()` shall zero the bias and covariance state while preserving the configured dimension.   | EST-EKF-001      | Test (unit)                   |
| EST-EKF-102   | Accelerometer tilt update      | The accel update shall correct roll/pitch via the gravity-direction residual (`H_θ = [u_b]×`), converging tilt to truth; the mag update corrects yaw only (about the world vertical).      | EST-EKF-001      | Test (unit)                   |
| EST-EKF-103   | Gyro-bias observability        | A persistent gyro offset on a stationary vehicle shall be estimated into the gyro-bias state so the attitude stays steady (no drift).                                                      | EST-EKF-001      | Test (unit)                   |
| EST-EKF-104   | Accel-bias update (9-state)    | In the 9-state variant the specific-force model (`h = g·u_b + b_a`) shall estimate accelerometer bias under motion, with attitude tracking truth.                                          | EST-EKF-002      | Test (unit)                   |
| EST-EKF-105   | Measurement gating             | The accel update shall be skipped when `|‖a‖ − g| > EKF_ACC_GATE` or `‖a‖ ≈ 0`, and the mag yaw update skipped when `‖m‖ < 1e-6`, leaving the estimate finite and tilt accel-pinned.       | EST-EKF-001      | Test (unit)                   |
| EST-EKF-106   | Covariance / quaternion health | The quaternion shall stay finite and unit-norm and the covariance finite/symmetric under a long run, via the Joseph-form covariance update.                                               | EST-EKF-001      | Test (unit, fuzz)             |
| EST-ALT-101   | Two-state complementary fusion | The vertical filter shall be a two-state complementary/steady-state-Kalman filter: predict integrates world-up inertial acceleration (gravity removed via the attitude quaternion, semi-implicit); correct applies position gain `k_alt` and cross-term velocity gain `k_vel` against baro altitude; the first correction seeds altitude bumplessly and predict is a no-op until seeded. | EST-ALT-001      | Test (unit)                   |
| EST-PHS-101   | Detection gating and debounce  | Takeoff shall require AGL > `TAKEOFF_ALT_M`, climb > `TAKEOFF_RATE_MS`, and throttle latched above `TAKEOFF_THROTTLE` since arm, all sustained past `TAKEOFF_DEBOUNCE_S`; touchdown shall require AGL < `LAND_ALT_M`, `|climb| < LAND_RATE_MS`, and throttle < `LAND_THROTTLE` sustained past `LAND_DEBOUNCE_S`; the ground reference shall be refined only while settled. | EST-PHS-001      | Test (unit)                   |

### 4.5 CTRL — Control loops

**Scope.** Inner rate loop, outer angle loop, motor mixing, PID
implementation. Lives in `src/control/` (PID core + control buffer folded
in from `maths/` per Phase 4 R2.6). Position / waypoint control is out of scope.

**Reserved IDs.** `CTRL-*-001..099`, `CTRL-*-101..199`.

**Sub-areas.** ANGLE (outer loop), RATE (inner loop), PID
(implementation), MIX (motor mixing + saturation), ARM (arming logic +
authority ramp), NUM (numerical hygiene).

#### 4.5.1 CTRL-HLR

| ID              | Title                            | Statement                                                                                                                                                                                                                                                                                                | Parent           | Verification                          |
|-----------------|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|---------------------------------------|
| CTRL-RATE-001   | Rate-loop frequency              | The body-rate control loop shall execute at 1 kHz with jitter ≤ ±100 µs.                                                                                                                                                                                                                                  | SYS-TIM-002      | Test (target timestamp log) + Analysis |
| CTRL-ANGLE-001  | Angle hold accuracy              | In stabilised mode, commanded roll and pitch shall be tracked within ±2° in steady wind ≤ 8 m/s.                                                                                                                                                                                                          | SYS-CTRL-002     | Demonstration + Test (SITL)           |
| CTRL-MIX-001    | Motor mixing layout              | The mixer shall translate `(roll, pitch, yaw, throttle)` outputs to per-motor commands per the X-configuration in `docs/coordinate_ref.md`: `m1 = t − r + p + y` (FR), `m2 = t − r − p − y` (RR), `m3 = t + r − p + y` (RL), `m4 = t + r + p − y` (FL).                                              | (process)        | Inspection + Test (unit)              |
| CTRL-MIX-002    | Saturation-preserving scaling    | When motor commands fall outside `[0, 1]`, the mixer shall scale only the PID differential, preserving the commanded throttle: `m_i ← throttle + scale × (m_i − throttle)`. After scaling, residual values are hard-clipped to `[0, 1]`.                                                              | CTRL-MIX-001     | Test (unit)                           |
| CTRL-MIX-003    | Motor idle floor                  | While in `ARMED` state, every motor command shall be ≥ `MOTOR_IDLE_FLOOR` (currently 0.15, i.e. 15 % of throttle range — a minimum-spin floor that keeps ESCs from stalling/desyncing).                                                                                                                                                                                  | (process)        | Test (unit)                           |
| CTRL-MIX-004    | Output gating by state            | Motor commands shall be queued to the actuator FIFO **only** when `system_state == ARMED`. Other states leave the FIFO un-pushed; the motor task zeros outputs whenever it observes a non-`ARMED` state.                                                                                                  | SYS-SAFE-001     | Test (unit + SITL)                    |
| CTRL-ARM-001    | Arming preconditions              | The control subsystem shall only enter `ARMED` if SYS-SAFE-005 preconditions hold. The transition is gated by the state machine.                                                                                                                                                                          | SYS-SAFE-005     | Test (SITL)                           |
| CTRL-ARM-002    | Authority ramp                    | Below `MIN_ARMED_THROTTLE` (0.1) PID outputs are forced to zero; between `MIN_ARMED_THROTTLE` and `PID_FULL_AUTHORITY_THROTTLE` (0.30 hardware / 0.45 SITL) PID outputs are linearly scaled to full authority. Prevents twitchy ground response.                                                          | SYS-SAFE-001     | Test (unit + SITL)                    |
| CTRL-FAIL-001   | Failsafe entry on attitude       | If `|roll| > MAX_ANGLE_CUTOFF` or `|pitch| > MAX_ANGLE_CUTOFF` (70°) the control subsystem shall request transition to `FAILSAFE` within the current rate-loop iteration. Yaw is excluded.                                                                                                              | SYS-SAFE-004     | Test (SITL)                           |
| CTRL-NUM-001    | Float-only arithmetic             | Control-loop code shall use single-precision `float`; `double`, FPU exceptions, and denormal handling are prohibited.                                                                                                                                                                                     | CTRL-RATE-001    | Analysis (static check)               |
| CTRL-SID-001    | On-hardware system-ID excitation  | The firmware shall provide a GCS-commanded rate-loop excitation that injects a bounded, tapered linear chirp (amplitude/frequency/duration hard-capped, self-aborting on an angle/rate limit on the excited axis) so a host can fit the rate plant; exercisable while disarmed with motors still. | (top level)      | Test (SITL + bench)                   |

#### 4.5.2 CTRL-LLR

| ID                | Title                       | Statement                                                                                                                                                                                                                                                  | Parent                  | Verification              |
|-------------------|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------|---------------------------|
| CTRL-RATE-101     | Rate-loop trigger            | The rate loop shall be triggered by arrival of a valid IMU sample on the IMU control queue and shall not poll the clock. ✅ waits on `imu_queue_control_wait()` (binary sema given per control-queue push), with a 5 ms timeout bounding the worst-case period if the IMU stalls (Phase 3 CTRL).                              | CTRL-RATE-001, VOS-IPC-001 | Inspection + Test       |
| CTRL-RATE-102     | Rate PID gains (hardware)    | Rate-loop PID gains on hardware: roll `Kp=0.08, Ki=0.04, Kd=0.01, Kff=0.1`, `i_max=0.2`, `d_max=0.25`. Pitch identical. Yaw all zero (yaw-rate loop intentionally disabled in current firmware).                                                              | CTRL-RATE-001           | Inspection                |
| CTRL-RATE-103     | Rate PID gains (SITL)        | Rate-loop PID gains in SITL (`VAYU_SIM`): 16× smaller than hardware. Documented in `variables.h`; controlled by the SITL build flag.                                                                                                                        | CTRL-RATE-001           | Inspection                |
| CTRL-ANGLE-101    | Angle-loop trigger           | The angle loop shall run as a periodic task at ~500 Hz (`v_delay(2)`), reading the latest RC sticks, the latest attitude estimate, and writing rate setpoints to the rate loop's queue.                                                                     | CTRL-ANGLE-001          | Inspection + Test         |
| CTRL-ANGLE-102    | Angle PID gains              | Angle-loop is pure-P: roll `Kp=4.0` (hw) / `0.25` (SITL); pitch identical; yaw `Kp=0`. Output saturated to ±100 °/s.                                                                                                                                          | CTRL-ANGLE-001          | Inspection                |
| CTRL-ANGLE-103    | RC stick mapping             | RC sticks shall be normalised with a `RADIO_AVOID_BAND = 10 µs` deadband around 1500, mapped via the `PID_RC2ANGLE_RATE_MODE` curve (default `CUBIC`), then scaled: roll = `ch[0]·100°`, pitch = `−ch[1]·100°`, yaw = `ch[3]·100°`, throttle = `(ch[2]−1000)/1000`. | SYS-CTRL-001             | Test (unit)               |
| CTRL-PID-101      | Parallel-form PID            | The shared PID block (`pid.c`) shall implement parallel `P + I + D + FF` with: integrator clamped to `±i_max` during accumulation; integrator frozen if `P + I` outside the output range (anti-windup); derivative computed on measurement (`d_meas`); derivative LPF with `α = dt / (dt + d_lpf_rc)`. | CTRL-RATE-001           | Test (unit) + Analysis (CBMC) |
| CTRL-PID-102      | PID reset on ARM transition  | The shared PID block shall be reset (integrator and derivative state zeroed) on every `STANDBY → ARMED` transition.                                                                                                                                          | CTRL-ARM-001            | Test (unit)               |
| CTRL-PID-103      | PID integrator gating         | The rate-loop integrator shall remain zero while `throttle < RATE_PID_INTEGRATE_THROTTLE` (0.3), preventing windup-on-ground.                                                                                                                                 | CTRL-ARM-002            | Test (unit + SITL)        |
| CTRL-MIX-101      | NaN guard                    | After motor mixing, any motor command that fails `m == m` (NaN guard) shall be replaced by zero.                                                                                                                                                              | CTRL-MIX-001            | Test (unit, fuzz)         |
| CTRL-FAIL-101     | Failsafe path                | On `FAILSAFE` entry, the angle-rate task and motor task shall observe the state change within one task iteration; the motor task then commands zero PWM (see ACT-FAIL-001).                                                                                  | SYS-SAFE-001, CTRL-FAIL-001 | Test (SITL)             |
| CTRL-NUM-101      | No double-precision in loop  | The control loop shall not call `double`-returning libm functions. Use `sinf`, `cosf`, `atan2f`, `sqrtf` exclusively.                                                                                                                                        | CTRL-NUM-001            | Analysis (grep + clang-tidy) |
| CTRL-SID-101      | Chirp generation and self-abort | The excitation shall generate a phase-continuous linear chirp with cosine taper in/out, advancing phase per `dt`, auto-stopping at the configured duration and aborting if `|angle|` or `|rate|` on the excited axis exceeds the hard cap.                | CTRL-SID-001            | Test (unit)               |
| CTRL-SID-102      | Excitation capture and dump  | During a run the firmware shall capture the excited axis's (control-effort `u`, gyro rate) into a bounded RAM ring decimated to ~500 Hz and stream it back in chunks on request after the run.                                                                | CTRL-SID-001            | Test (SITL)               |
| CTRL-RATE-104     | Optional INDI inner loop     | The firmware shall provide a compile-time-selectable (`RATE_CTRL_ALGO_USED`) incremental-nonlinear-dynamic-inversion alternative to the PID inner loop, with synchronized gyro/derivative/command filtering and saturation-aware actuator feedback; default OFF. | CTRL-RATE-001           | Test (SITL) + Analysis    |
| CTRL-RATE-105     | Configurable gyro-measurement low-pass | The rate loop shall apply an optional per-axis first-order low-pass to the gyro (rate) measurement, with a live-settable and persisted time constant (`CMD_SET_GYRO_LPF`), default disabled (passthrough).                                          | CTRL-RATE-001           | Test (unit)               |
| CTRL-MIX-102      | Airmode sequential desaturation | When airmode is enabled, the mixer shall desaturate by first shifting collective thrust to make room, then (RP mode) sacrificing yaw, preserving roll/pitch torque rather than scaling the whole differential.                                          | CTRL-MIX-001            | Test (unit)               |

### 4.6 ACT — Actuator output

**Scope.** Motor and ESC drivers (`src/actuator/`), motor failsafe path.

**Reserved IDs.** `ACT-*-001..099`, `ACT-*-101..199`.

**Sub-areas.** MOT (motor task), ESC (PWM driver), FAIL (failsafe + disarm path).

#### 4.6.1 ACT-HLR

| ID            | Title                      | Statement                                                                                                                                                                                                                          | Parent           | Verification          |
|---------------|----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-----------------------|
| ACT-MOT-001   | Motor output range          | Motor commands shall be clipped to `[0, 1]` and mapped to ESC PWM range before being written to the hardware timer.                                                                                                                | CTRL-MIX-001     | Test (unit)           |
| ACT-MOT-002   | Motor task period           | The motor task shall consume from the motor-output FIFO at ≥ 500 Hz (`v_delay(2)`); if the FIFO is empty for one tick the task shall hold the previous command.                                                                    | CTRL-RATE-001    | Test (target)         |
| ACT-MOT-003   | Per-motor channel mapping   | Motors shall be mapped to TIM1 channels: M1=FR=CH1, M2=RR=CH2, M3=RL=CH3, M4=FL=CH4, on pins PA08..PA11.                                                                                                                            | (process)        | Inspection            |
| ACT-ESC-001   | ESC PWM frequency           | ESC PWM output shall be 400 Hz; pulse range 1.0–2.0 ms ⇒ duty 0.4–0.8.                                                                                                                                                              | (process)        | Inspection + Test (logic analyser) |
| ACT-ESC-002   | ESC arming sequence         | At boot, all four ESCs shall be armed with a 4 ms inter-ESC delay followed by a 100 ms hold at minimum throttle.                                                                                                                    | (process)        | Test (bench)          |
| ACT-FAIL-001  | Disarm output               | On any state other than `ARMED`, the motor task shall command zero PWM within one task iteration (≤ 2 ms).                                                                                                                          | SYS-SAFE-001     | Test (SITL + target)  |

### 4.7 COMM — Communications

**Scope.** RC ingest (iBUS), telemetry tx/rx, packet encoding/decoding,
heartbeat, command dispatch. `src/comm/`.

**Reserved IDs.** `COMM-*-001..099`, `COMM-*-101..199`.

**Sub-areas.** RC (iBUS ingest), CH (UART channel + ping-pong + DMA),
PKT (frame encoder/decoder), CMD (inbound command dispatch), TEL
(outbound telemetry task), HB (heartbeat).

#### 4.7.1 COMM-HLR

| ID            | Title                              | Statement                                                                                                                                                                                                                                                | Parent           | Verification                |
|---------------|------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-----------------------------|
| COMM-RC-001   | iBUS frame validity                | Incoming iBUS frames (32 B: `0x20 0x40 [14×u16 channels] csum_L csum_H`) shall be validated by 16-bit complement checksum; frames failing the checksum shall not update the latest-channel state.                                                       | SYS-SAFE-002     | Test (unit, golden vectors) |
| COMM-RC-002   | RC loss detection                  | The RC subsystem shall raise `rc_loss` if no valid frame is received for > 100 ms. ✅ `rc_loss()` trips at `RC_LOSS_DETECT_MS` = 100 ms — a fast COMM-layer detect distinct from (and an order of magnitude before) the 1.0 s `rc_has_signal()` failsafe horizon (SYS-SAFE-002). Verified two-tier in test_safety_phase2.                                                            | SYS-SAFE-002     | Test (unit + SITL)          |
| COMM-RC-003   | RC arming logic                    | While in `STANDBY`, a transition `ch[4] > 1500` triggers `ARMED` if `ch[2] < 1100`, else `FAILSAFE`. While in `ARMED` or `FAILSAFE`, `ch[4] ≤ 1500` returns to `STANDBY`.                                                                                | SYS-SAFE-005     | Test (unit + SITL)          |
| COMM-RC-004   | iBUS transport                     | The iBUS UART shall run at 115200 baud on USART2 (PA2/PA3) with circular DMA RX into a 128-byte ring (telemetry owns USART6). The task shall be woken by the UART idle-line interrupt and locate the DMA write position via `hal_uart_dma_rx_index()` rather than accessing DMA registers directly. | (process)        | Inspection + Test (bench)   |
| COMM-CH-001   | UART TX ping-pong buffer           | Each UART channel shall maintain two 512-byte ping-pong buffers, the inactive being TX'd via DMA while the active one accumulates writes.                                                                                                                 | (process)        | Inspection                  |
| COMM-CH-002   | UART backpressure                  | A write that would exceed the active buffer's capacity (2048 B ping-pong, capped at 1280 B for telemetry — the remaining 768 B is reserved for bulk-transfer) shall return `ERROR` and increment a `tx_overflow` counter. The counter shall be surfaced through telemetry. ✅ `channel_tx_overflow_count()` increments on the full-buffer drop and is emitted as a HEALTH status (SYSTEM_ORIGIN_HEALTH) at 2 Hz (Phase 3 COMM).             | (process)        | Test (unit, fault injection) |
| COMM-PKT-001  | Outbound frame format              | All outbound frames shall use the wire framing in SYS-TEL-004 (sync `0x56`, protocol nibble `0x1`, type nibble in upper 4 bits of byte 1).                                                                                                                | SYS-TEL-004      | Inspection + Test           |
| COMM-PKT-002  | Inbound frame parsing              | Incoming GCS frames shall be validated by CRC32 before dispatch; a CRC failure shall increment a `crc_failed` counter and resync; the parser shall expose the counter via telemetry.                                                                     | SYS-TEL-004      | Test (unit, golden vectors mirroring `navigator/src/protocol/PacketDecoder.cpp`) |
| COMM-PKT-003  | Inbound packet buffer              | The deserializer shall stage decoded packets in a 3-slot ring buffer between the UART RX ISR and the command-dispatch task; overflows shall increment a `pkt_dropped` counter.                                                                            | (process)        | Test (unit)                 |
| COMM-CMD-001  | Calibration commands                | The command dispatcher shall accept `CMD_CALIBRATE_IMU` (0x0001, payload: `[u16 cmd_id][u8 argc][float imu_id][float type]`) and `CMD_CANCEL_CALIBRATION` (0x0009, no args).                                                                              | SYS-CAL-001/2/3  | Test (unit)                 |
| COMM-CMD-002  | Command payload validation          | Every command handler shall validate `argc ≥ expected_argc` and `payload_length ≥ argc × 4 + 3` before reading args.                                                                                                                                       | (process)        | Test (unit, fuzz)           | ✅ `command_payload_valid()` gates the calibration + SET_PID handlers; `pkt.length` guarded before any arg read (Phase 3 COMM). |
| COMM-CMD-003  | CMD_SET_PID                         | The command dispatcher shall accept `CMD_SET_PID` (0x000A) carrying live PID gain updates. ✅ schema = [ctrl, axis, Kp, Ki, Kd, Kff] (6 float args); applied live via `*_controller_set_gains()` and persisted to SD (`0:pid.bin`), restored at boot by `pid_config_init()` (Phase 3 COMM).                                                                            | SYS-CTRL-001     | Test (unit + SITL)          |
| COMM-TEL-001  | Telemetry task cadence              | The telemetry task shall run a ~500 Hz base loop (`v_delay(TELEM_BASE_MS)`, TELEM_BASE_MS = 2), dispatching each stream on millisecond-based `TELEM_GATE` gates so per-stream effective rates (e.g. 50 Hz compressed IMU, 5 Hz baro) are independent of the base-loop frequency.                                                                                                                       | (process)        | Inspection                  |
| COMM-TEL-002  | Heartbeat cadence                   | The vehicle shall emit a heartbeat packet at ≥ 1 Hz. ✅ cadence set to 166 ticks × 6 ms = 996 ms ⇒ 1.004 Hz (closest period to 1 s that still satisfies ≥ 1 Hz); comment corrected (Phase 3 COMM).                                                                          | SYS-TEL-001      | Test (SITL)                 |
| COMM-TEL-003  | Per-packet emission rates           | Emission cadences (steady-state, ARMED): IMU full ≥ 1.6 Hz (every 100 ticks); IMU compressed ≥ 25 Hz (every 6 ticks); attitude ≥ 10 Hz; RC channels ≥ 10 Hz; system status ≥ 2 Hz; motor telemetry ≥ 18 Hz; control-loop data ≥ 18 Hz; log lines ≥ 15 Hz. | SYS-TEL-002      | Test (SITL + target log)    |
| COMM-TEL-004  | Conditional emission                | Packets sourced from queues (IMU, attitude, RC, motor, log, calibration update) shall be emitted only when the queue has a fresh sample; modulo gating shall not force an emission of stale data.                                                          | (process)        | Test (unit)                 |
| COMM-HB-001   | Heartbeat handshake                 | On receiving a heartbeat from the GCS, the firmware shall capture `device_id` and `timestamp` from the packet header for clock-drift reporting.                                                                                                            | SYS-TEL-001      | Test (SITL)                 |
| COMM-FLUSH-001 | Flush task                          | A dedicated flush task shall run every 1 ms, swapping ping-pong buffers and starting DMA on each UART channel whose `busy` flag is clear.                                                                                                                  | COMM-CH-001      | Inspection + Test (bench)   |
| COMM-XFER-001  | Bulk file transfer                  | The firmware shall support reliable bulk file transfer (download and upload) of SD-card files over NavLink using a per-session offset-based, chunked, NAK/resume protocol that survives dropped/duplicated/reordered chunks and only reports completion once bytes are durably persisted. | (top level)      | Test (SITL byte-perfect round-trip, fault injection) |
| COMM-XFER-002  | Live stream transfer                | The firmware shall support open-ended best-effort streaming of a named live data source over the transfer substrate at a requested rate, never rewinding.                                                                                                 | (top level)      | Test (SITL)                 |
| COMM-FS-001    | Filesystem navigation               | The firmware shall answer NavLink directory-list (paged) and path-stat requests by walking the VFS off the comm task, emitting one entry per item plus a terminal count.                                                                                   | (top level)      | Test (SITL)                 |
| COMM-TEL-005   | Observability / performance telemetry | The firmware shall periodically emit a scheduler/IPC/heap/ISR snapshot plus per-task and per-FIFO statistics (and estimator timing) to the GCS, degrading to a valid empty report when the perf module is disabled.                                     | (top level)      | Test (SITL) + Inspection    |
| COMM-TEL-006   | Auxiliary sensor / mode telemetry   | The firmware shall emit barometric (pressure/temp/humidity/altitude), fused vertical-state, and active flight-mode telemetry to the GCS.                                                                                                                   | (top level)      | Test (SITL)                 |
| COMM-SYNC-001  | Time-synchronisation handshake      | The firmware shall implement an NTP-style time-sync exchange (echo t1, stamp t2/t3 from the corrected clock, apply 32/64-bit GCS offsets) so command timestamps share a disciplined clock with the GCS.                                                    | SYS-TIM-006      | Test (SITL)                 |
| COMM-CMD-004   | Live tuning command set             | The command dispatcher shall accept `CMD_SET_GYRO_LPF`, `CMD_SET_D_LPF`, `CMD_SET_MOTOR_GEOMETRY` and `CMD_SET_FLIGHT_MODE`, validating each payload before applying it to the live controller.                                                            | (top level)      | Test (unit + SITL)          |
| COMM-CMD-006   | GCS software arm/disarm command     | The dispatcher shall accept `CMD_ARM`/`CMD_DISARM` by setting/clearing the software-arm latch, with the actual arm gated by the RC-task preconditions (SYS-SAFE-005) and the outcome resolved from the flight-state machine.                               | (top level)      | Test (SITL)                 |

### 4.8 LOG — On-device logging

**Scope.** `src/logger/` — SD-card binary ring-buffer logger (`logger.c`)
+ in-memory `vayu_log_queue` text feeder (`log_text.c`, moved from
`utils/` per Phase 4 R2.6).

**Reserved IDs.** `LOG-*-001..099`, `LOG-*-101..199`.

**Sub-areas.** TXT (text-line emission to telemetry), SD (binary
ring-buffer files on SD), PERSIST (cross-restart persistence).

#### 4.8.1 LOG-HLR

| ID            | Title                              | Statement                                                                                                                                                                                          | Parent           | Verification    |
|---------------|------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|-----------------|
| LOG-TXT-001   | Log line emission                  | The logger shall accept text lines from any task via `vayu_log()` and emit them as `0x7` LOG packets on the telemetry transport at ≥ 15 Hz.                                                        | SYS-TEL-004      | Test (unit)     |
| LOG-TXT-002   | Log queue drain                    | The text-log queue (`vayu_log_queue`) shall be drained by the telemetry task. ✅ `imu_telemetry_task` drains it via `mpmc_pop_bulk` into `PACKET_TYPE_LOG` at ~15 Hz (Phase 3 SLOG). | LOG-TXT-001      | Inspection      |
| LOG-RATE-001  | Bounded log rate                   | The logger shall enforce a maximum emission rate (default 50 lines/s) and drop excess lines with a single "rate-limited" summary.                                                                  | (process)        | Test (unit)     |
| LOG-SD-001    | SD-card ring-buffer logs           | Three independent binary ring-buffer files shall be preallocated at boot: `v_nav.bin` (Navlink telemetry mirror), `v_sys.bin` (system events), `v_gen.bin` (general). Each: 10 MB on hardware, 64 KB in SITL. | (process)     | Inspection + Test (bench) |
| LOG-SD-002    | Wrap-on-full                       | Each SD log shall wrap (overwrite oldest) when its write position exceeds the file size. ✅ wrap now increments a per-log counter (`logger_wrap_count()` / `_total`); aggregate surfaced in the HEALTH status. Bench round-trip on real SD deferred (Phase 3 SLOG).                  | LOG-SD-001       | Test (bench)    |
| LOG-OWN-001   | Single runtime filesystem owner    | All runtime SD/VFS access shall execute in exactly one task context; other tasks shall route reads/stat/dir/truncate to that task and block for the result, so FatFS is never driven concurrently. | (top level)      | Test (SITL + bench) |
| LOG-PERSIST-001 | Off-critical-path PID/calib persistence | PID and calibration snapshots shall be persisted to SD asynchronously via a reserved save lane that logging cannot starve, with bounded retry on transient open/write failure. | SYS-CAL-004      | Test (unit + bench) |
| LOG-XFER-001  | Bulk file upload (positioned async writes) | The FS owner shall accept positioned writes (path+offset) into a dedicated lane drained after saves but before blackbox logs, retry transient failures up to a bounded budget, and expose per-session pending/committed/failed status. | (top level)      | Test (bench round-trip, byte-perfect) |
| LOG-XFER-002  | Bulk file download (held-open positioned reads) | The FS owner shall serve positioned reads and truncate-to-empty for the xfer substrate from the owner task, holding the read fd open across a flow-controlled download and dropping the handle on a transient read error for retry. | (top level)      | Test (bench round-trip) |
| LOG-FS-001    | Filesystem navigation (stat / directory browse) | The FS owner shall expose stat, opendir, readdir and closedir funnelled through the single owner context to support GCS file/directory browsing over NavLink. | (top level)      | Test (SITL + bench) |

#### 4.8.2 LOG-LLR

| ID            | Title                       | Statement                                                                                                                                                                    | Parent      | Verification |
|---------------|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|--------------|
| LOG-SD-101    | Per-file mutex              | Each ring-buffer log file shall have its own mutex; writers shall hold the mutex for `seek + write + sync` only.                                                              | LOG-SD-001  | Inspection   |
| LOG-SD-102    | Sync on every write         | Each `logger_write_internal()` shall sync the underlying VFS file before returning, so power-loss truncation is bounded by one record.                                        | LOG-SD-001  | Inspection   |

---

## 5. Traceability

Bidirectional traceability via Doxygen-style tags. A Python script
walks the source tree, builds a trace matrix, and **fails CI** if:

1. Any tagged ID does not exist in this document.
2. Any requirement marked active (not deferred / not dropped) has zero
   `@implements` references.
3. Any requirement marked active has zero `@verifies` references.

### 5.1 Tagging convention

**In source.** On the public function (or, for module-wide concerns, on
the module's public header):

```c
/**
 * Body-rate controller step. Runs at 1 kHz, triggered by IMU sample
 * arrival on the IMU queue.
 *
 * @implements CTRL-RATE-001, CTRL-RATE-101, CTRL-PID-101
 */
status_t ctrl_rate_step(const imu_sample_t *imu,
                        const rate_setpoint_t *sp,
                        motor_cmd_t *out);
```

**In tests.** On the test function:

```c
/* @verifies CTRL-RATE-001 */
TEST(rate_ctrl, runs_at_1khz_in_sitl) { /* … */ }

/* @verifies CTRL-PID-101 */
TEST(rate_ctrl, clips_output_to_unit_range) { /* … */ }
```

Multiple IDs per tag are comma-separated. IDs are case-sensitive.

### 5.2 CI gate

A `tools/dev/trace.py` script (to be written; placeholder requirement
SYS-TEL-004 will reference the gate once it exists) shall:

1. Parse every `*.h` / `*.c` for `@implements` / `@verifies` tags.
2. Parse `docs/firmware/requirements.md` for `MOD-SUB-NNN` table rows.
3. Build `docs/firmware/trace.md` (or `.json`) mapping each ID →
   { implementer files, verifier files, status }.
4. Exit non-zero if any of the three CI-fail conditions above hold.

### 5.3 Vendored-code exemption

`HAL-*` and `VOS-*` requirements may be implemented in
`extern/vaios/**` or `extern/vaios/extern/NavHAL/**`. The trace script
treats those subtrees as in-scope for `@implements` discovery but **not**
for `@verifies` (vendor tests are run from their own repos). When a
vendor requirement has no in-repo verifier, the trace matrix marks it
`verified-upstream` rather than failing CI.

---

## 6. Audit snapshot — 2026-05-26

This doc was anchored against a full module-by-module read of `src/`.
The single biggest finding is that **the firmware is healthier than the
seed requirements assumed in many places, but the safety-net layer is
thin**. The cleanup backlog below is what the `🟡 gap` markers in
sections 3–4 collectively represent.

### Reality vs. previous spec drift (corrected this pass)

| Was claimed                                          | Reality                                                                                  |
|-------------------------------------------------------|------------------------------------------------------------------------------------------|
| BMX160 gyro range = ±2000 dps                         | ±1000 dps (`variables.h:42`)                                                             |
| BMX160 on SPI with DRDY interrupt + FIFO              | I²C 0x68 fast-mode; **no DRDY**, **no FIFO**; polled register reads with DMA-completion semaphore handshake (SNS-BMX-105). `HAL-IMU-101/102` dropped, replaced by `HAL-I2C-001` + `HAL-DMA-001`. |
| Heartbeat ≥ 1 Hz                                      | ✅ 1.004 Hz (every 996 ms = 166 ticks; comment corrected) — COMM-TEL-002                  |
| 8 priority levels in scheduler                        | 3 priorities used (0, 1, 2); scheduler supports more                                     |
| Rate loop triggered by IMU queue arrival              | ✅ sample-driven via `imu_queue_control_wait()` + 5 ms safety timeout (CTRL-RATE-101)     |
| Estimator emits a degraded flag                       | ✅ `estimator_is_degraded()` after 100 ms continuous rejection (EST-MAH-002)              |
| RC loss detected by elapsed time                      | ✅ two-tier: 100 ms `rc_loss()` (COMM-RC-002) + 1.0 s failsafe (SYS-SAFE-002)             |
| `vayu_log_queue` is consumed by telemetry             | ✅ drained by `imu_telemetry_task` into PACKET_TYPE_LOG (LOG-TXT-002)                      |
| State transitions guarded                             | ✅ `system_state_set()` validates against `k_allowed_transitions[][]` (SYS-SAFE-006 / SYS-STATE-002) |

### Cleanup backlog (every 🟡 in this doc)

These are the audit-derived gaps; each carries a requirement ID so the
work is tracked in the trace matrix when `tools/dev/trace.py` lands.

- **Safety**
  - SYS-SAFE-002 — wire an elapsed-time RC watchdog (depend on COMM-RC-002).
  - SYS-SAFE-003 — emit + consume an `estimator_degraded` flag (depend on EST-MAH-002).
  - SYS-SAFE-005 — gate `ARMED` on estimator convergence + calibration freshness, not just RC sticks.
  - SYS-SAFE-006 — static allowed-transitions table in `state.c`; reject + log out-of-order requests.
- **Estimation**
  - EST-MAH-002 — `estimator_degraded` flag after 100 ms continuous sample rejection.
  - ✅ EST-MAH-105 — clamp Mahony integral feedback term and reset it on estimator reset.
- **Control**
  - ✅ CTRL-RATE-101 — refactor rate loop from `v_delay(1)` polling to wait-on-queue.
- **Sensors**
  - ✅ SNS-BUF-002 — expose IMU buffer drop counter via telemetry.
- **Communications**
  - COMM-RC-002 — RC watchdog (already named).
  - ✅ COMM-CH-002 — surface `tx_overflow` counter.
  - ✅ COMM-CMD-002 — payload length validation in calibration command path.
  - ✅ COMM-CMD-003 — implement `CMD_SET_PID` end-to-end (currently a stub).
  - ✅ COMM-TEL-002 — fix heartbeat cadence to true 1 Hz (or update SYS-TEL-001 to 1.11 Hz officially).
- **Logging**
  - ✅ LOG-TXT-002 — wire `vayu_log_queue` to telemetry consumer.
  - ✅ LOG-SD-002 — wrap marker / counter on ring-buffer file wrap.

### What's healthy (worth recording as "verified by reading")

- Wire framing on tx matches `navigator/src/protocol/PacketDecoder.cpp` byte-for-byte (sync, version nibble, length, dev_id, ts, CRC32 polynomial).
- Motor mixing layout matches `docs/coordinate_ref.md` X-config; saturation-preserving scaling is the correct strategy (CTRL-MIX-002).
- PID anti-windup gating on `P + I` saturation is implemented correctly in `pid.c`.
- ESC arming sequence (4 ms inter-ESC + 100 ms hold) is conservative and matches typical ESC requirements.
- I2C bus recovery (9-clock SCL toggle + STOP, then HAL reinit) is the right call for stuck-SDA recovery.
- `MOTOR_IDLE_FLOOR = 0.005` while ARMED prevents PWM-zero stalls on most ESCs (verified on hardware).
- Per-logger mutex protecting `seek + write + sync` is correct and minimal.

---

## 7. Maintenance rules

- **IDs are stable.** Never re-number, never reuse after deletion.
  Dropped rows stay with status `❌ (dropped: <reason>)`.
- **One claim per statement.** If a requirement has an "and", split it.
- **Parents are required.** SYS-level uses `(top level)`; everything
  else points at at least one parent.
- **Verification is required.** No requirement ships without a method.
  Saying "Demonstration" obligates a flight-test row in the test log.
- **The table is the contract.** The narrative around a table is
  commentary; the rows are what's tested.
- **Phase plan lives elsewhere.** When work blocks of these IDs are
  scoped together for a release, put the plan in `docs/firmware/plan/`
  rather than mutating this doc.
- **Deferred ≠ dropped.** Deferred items keep their ID reserved with
  status `❌ deferred — <reason>`; they become active by changing status
  + assigning a verifier.
