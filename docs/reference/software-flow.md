# Firmware software flow

A complete, current map of how the Vayu flight-controller firmware runs: the boot
sequence, every RTOS task, the IPC that connects them, and the four hot paths
(sensing → estimation → control → actuation, plus comms). Everything here is
drawn from the source (`src/`, `include/`); see the per-section file pointers.

> Supersedes the legacy hand-drawn `journal/legacy-soft-flow.drawio` (pre-NavLink-v2,
> pre-modular-refactor, Mahony-era). For prose detail see
> [firmware-control.md](firmware-control.md) and [pipeline-overview.md](pipeline-overview.md);
> for the wire format see [navlink-v2-spec.md](../../navlink/docs/reference/navlink-v2-spec.md).

**Platform:** STM32F401RE @ 84 MHz, **vaios** in-house RTOS (`task_create_named`,
`v_semaphore_*`, `task_delay_until`). Higher numeric priority = more urgent.

## How to read these diagrams

- **Rounded boxes** = RTOS tasks (with priority + rate). **Cylinders** = hardware/peripherals.
- **Double-bordered nodes** = lock-free SPSC ring buffers / channels (the IPC).
- Edge labels name the **queue / semaphore / signal** carrying the data and its rate.
- `prio N` is the vaios task priority; `@F` is the loop frequency.

---

## 1. System overview

Every task and the IPC between them. Single producer per queue; queues are
overwrite-policy rings (newest wins) so a slow consumer never blocks a fast producer.

```mermaid
flowchart TD
    classDef task fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef ipc  fill:#f3e8fd,stroke:#7b2cbf,color:#111;
    classDef hw   fill:#e6f4ea,stroke:#137333,color:#111;

    HWIMU[("BMX160 IMU<br/>(I2C1, DMA)")]:::hw
    HWRC[("FlySky RX<br/>(iBus, USART2)")]:::hw
    ESC[("4× ESC<br/>(TIM1 PWM)")]:::hw
    GCS[("GCS / Navigator<br/>(USART6, 230400)")]:::hw

    subgraph SENSE ["Acquisition (hot path)"]
        TICK["2 kHz tick ISR<br/>bmx160_fast_tick_isr"]:::task
        IMUREAD["imu_read · prio 2<br/>event-driven"]:::task
    end
    subgraph EST ["Estimation"]
        ATT["attitude · prio 1<br/>EKF @250 Hz"]:::task
    end
    subgraph CTRL ["Control cascade"]
        ANGLE["angle_ctl · prio 1<br/>angle PID @250 Hz"]:::task
        RATE["rate_ctl · prio 1<br/>rate PID + mixer @1 kHz"]:::task
        MOTOR["motor · prio 1<br/>@~500 Hz"]:::task
    end
    subgraph RCIN ["RC input"]
        RCT["rc_ibus · prio 0<br/>event-driven"]:::task
    end
    subgraph COMMS ["Comms"]
        COMMP["comm_processor · prio 0<br/>NavLink v2 RX @250 Hz"]:::task
        TELE["imu_telemetry · prio 0<br/>TX @166 Hz"]:::task
        FLUSH["flush · prio 0<br/>DMA flush @1 kHz"]:::task
        PERF["perf_telemetry · prio 0<br/>@1 Hz"]:::task
    end
    subgraph IND ["Indicators / boot"]
        HB["heartbeat · prio 0<br/>LED + buzzer"]:::task
        BOOT["boot · once<br/>checks → STANDBY/FAILSAFE"]:::task
    end

    CH[["g_telemetry_channel<br/>ping-pong 512 B"]]:::ipc

    HWIMU -->|DMA done → ready_sema| IMUREAD
    TICK  -->|fast_tick_sema 2 kHz| IMUREAD
    IMUREAD -->|imu_attitude_queue| ATT
    IMUREAD -->|imu_control_queue| RATE
    IMUREAD -->|imu_telemetry_queue| TELE
    ATT  -->|attitude_control_queue| ANGLE
    ATT  -->|attitude_telemetry_queue| TELE
    HWRC -->|idle ISR → ibus_frame_sema| RCT
    RCT  -->|rc_control_queue| ANGLE
    RCT  -->|rc_telemetry_queue| TELE
    ANGLE -->|angle_controller_fifo| RATE
    RATE  -->|motor_queue| MOTOR
    RATE  -->|control_telemetry_queue| TELE
    MOTOR -->|TIM1 PWM| ESC
    MOTOR -->|motor_telemetry_queue| TELE

    TELE --> CH
    PERF --> CH
    CH -->|hal_uart_write_dma| FLUSH
    FLUSH --> GCS
    GCS  -->|RX ISR → rx_raw_buf| COMMP
    COMMP -.->|setpoints / mode / PID / arm| CTRL
    COMMP -.->|spawns| CALIB["calibration_task<br/>on CMD_CALIBRATE_IMU"]:::task
```

State and arm latches (`_system_current_status`, `g_sw_arm_request`) are volatile
globals read across tasks (vaios R8.6 lock-free scalars), shown as dashed influence
rather than queues.

*Source: `src/main.c:84-123` (task creation), `src/sensor/imu_buffer.c` (queues),
`src/comm/channel.c` (`g_telemetry_channel`).*

---

## 2. Boot & init (`main()` → scheduler)

```mermaid
flowchart TD
    classDef step fill:#fff,stroke:#444,color:#111;
    A["reset → main()"]:::step
    A --> B["assert state == UNINITIALIZED (canary)"]:::step
    B --> C["clock_setup() → 84 MHz (HSE+PLL)"]:::step
    C --> D["hal_cycle_counter_init() (DWT timestamps)"]:::step
    D --> E["v_system_init() — vaios kernel + SD"]:::step
    E --> F["init_i2c_manager() — single-owner I2C1"]:::step
    F --> G["logger_init()"]:::step
    G --> H["pid_config_init() — restore tune from SD"]:::step
    H --> I["system_state_init() → SYSTEM_STATE_INIT"]:::step
    I --> J["init_sensors():<br/>imu_buffer_init · control_telemetry_buffer_init<br/>bmx160_init · rc_buffer_init<br/>open g_telemetry_channel (USART6 @230400)"]:::step
    J --> K["system_init_tasks() → heartbeat, boot"]:::step
    K --> L["init_tasks() → 10 runtime tasks"]:::step
    L --> M["init_timer_callbacks():<br/>10 kHz timer; bmx160_fast_tick_isr @500 µs (2 kHz)"]:::step
    M --> N["scheduler_start(); while(1)"]:::step
    N --> O["boot task: clock + SD checks<br/>pass → STANDBY · fail → FAILSAFE"]:::step
```

*Source: `src/main.c:42-160`.*

---

## 3. System state machine

One-hot bitmask states. Transitions are validated against a static allow-table;
**FAILSAFE is reachable from any active state** (safety overrides protocol).

```mermaid
stateDiagram-v2
    [*] --> UNINITIALIZED
    UNINITIALIZED --> INIT : system_state_init()
    INIT --> STANDBY : boot checks pass
    INIT --> FAILSAFE : boot checks fail
    STANDBY --> ARMED : arm engaged + preconditions
    STANDBY --> PREARM
    STANDBY --> CALIBRATING : CMD_CALIBRATE_IMU
    ARMED --> IN_AIR
    ARMED --> STANDBY : disarm
    IN_AIR --> ARMED
    IN_AIR --> STANDBY
    PREARM --> ARMED
    PREARM --> STANDBY
    CALIBRATING --> STANDBY : calibration done
    FAILSAFE --> STANDBY : disarm switch
    FAILSAFE --> CALIBRATING
    ARMED --> FAILSAFE
    IN_AIR --> FAILSAFE
    STANDBY --> FAILSAFE

    note right of FAILSAFE
      Entered from any active state by:
      RC staleness >1 s · throttle-jump
      bank angle >70° (angle mode)
      estimator degraded >100 ms
      arm-precondition fail · boot fail
    end note
    note left of PREARM
      PREARM / IN_AIR / TERMINATED are
      defined but not yet driven by code.
    end note
```

*Source: `include/sys/state.h:6-16`, `src/sys/state.c:30-73`, `src/comm/rc_safety.c`,
`src/sys/boot.c`.*

---

## 4. IMU acquisition & fan-out (the 2 kHz hot path)

A 10 kHz timer paces a 2 kHz tick. The `imu_read` task owns a 3-phase split-rate
read over the single-owner I2C DMA bus, then fans the processed sample out to four
consumers.

```mermaid
flowchart TD
    classDef task fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef ipc  fill:#f3e8fd,stroke:#7b2cbf,color:#111;
    classDef hw   fill:#e6f4ea,stroke:#137333,color:#111;

    T["TIM 10 kHz → fast_tick_isr<br/>every 500 µs"]:::hw
    T -->|fast_tick_sema| RL["imu_read · prio 2<br/>bmx160_initiate_read"]:::task
    RL -->|"i2c_manager_read_async<br/>(DMA1 S0)"| BUS["I2C1 single-owner bus<br/>_i2c_sema + _bus_busy"]:::hw
    BUS -->|DMA complete ISR| CB["bmx160_dma_callback_*<br/>→ ready_sema"]:::task
    CB --> RL

    RL --> SEQ{"op phase"}:::task
    SEQ -->|"FAST: 12 B @0x0C (gyro+accel)"| PROC
    SEQ -->|"every 13th: MAG 8 B @0x04"| PROC
    SEQ -->|"then TEMP 2 B @0x20"| PROC
    PROC["bmx160_process_data<br/>units · axis remap · LPF<br/>online gyro-bias · mag/temp when fresh"]:::task

    PROC -->|imu_telemetry_queue| Q1[["→ imu_telemetry"]]:::ipc
    PROC -->|imu_control_queue| Q2[["→ rate_ctl (gyro)"]]:::ipc
    PROC -->|"if CALIBRATING:<br/>imu_calibration_queue"| Q3[["→ calibration_task"]]:::ipc
    PROC -->|"else: estimator_mark_sample()<br/>imu_attitude_queue"| Q4[["→ attitude"]]:::ipc
```

*Source: `src/sensor/bmx160.c:852-1275`, `src/sensor/i2c_manager.c`,
`src/sensor/imu_buffer.c`. Sample rate `IMU_SAMPLE_FREQ_HZ` (`variables.h:218`).*

---

## 5. Estimation & control cascade

Attitude estimate (EKF) → outer **angle** loop → inner **rate** loop + mixer →
motors. The outer loop produces rate setpoints; the inner loop closes on the gyro.
Motors are driven **only while ARMED**.

```mermaid
flowchart TD
    classDef task fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef ipc  fill:#f3e8fd,stroke:#7b2cbf,color:#111;
    classDef hw   fill:#e6f4ea,stroke:#137333,color:#111;

    IMUA[["imu_attitude_queue"]]:::ipc --> ATT
    ATT["attitude · prio 1<br/>SF_EKF, decimated 2 kHz→250 Hz (÷8)<br/>predict(Σdt) + correct"]:::task
    ATT -->|"degraded >100 ms → FAILSAFE"| SAFE["estimator_safety_step()"]:::task
    ATT -->|attitude_control_queue<br/>q + euler| ANGLE

    RCQ[["rc_control_queue"]]:::ipc --> ANGLE
    ANGLE["angle_ctl · prio 1 @250 Hz<br/>normalize RC (deadband/expo)<br/>ANGLE: roll/pitch angle PID → rate sp<br/>ACRO: sticks → rate sp direct<br/>yaw always rate · bank >70° → FAILSAFE"]:::task
    ANGLE -->|angle_controller_fifo<br/>rate setpoints + throttle| RATE

    IMUC[["imu_control_queue (gyro)"]]:::ipc --> RATE
    RATE["rate_ctl · prio 1 @1 kHz<br/>gyro deadband + LPF · rate PID<br/>authority ramp (0.1→0.45 thr)<br/>mixer (geometry) + anti-saturation"]:::task
    RATE -->|"ARMED only:<br/>motor_queue"| MOTOR
    RATE -->|control_telemetry_queue| TELE([imu_telemetry]):::task

    MOTOR["motor · prio 1 @~500 Hz<br/>zero outputs if !ARMED"]:::task
    MOTOR -->|esc_set_throttle ×4| ESC[("4× ESC · TIM1 PWM")]:::hw
    MOTOR -->|motor_telemetry_queue| TELE
```

Flight modes: **ANGLE** (self-levelling) / **ACRO** (rate) toggled by RC ch6 or a
GCS override, arbitrated by `flight_mode_resolve_acro`. Arm transition hard-resets
all rate PIDs and LPF state.

*Source: `src/est/attitude_task.c`, `src/control/angle_controller.c`,
`src/control/angle_rate_controller.c`, `src/actuator/motor.c`. Rates:
`OUTER_LOOP_FREQ_HZ=250`, `INNER_LOOP_FREQ_HZ=1000` (`variables.h:207-230`).*

---

## 6. Command RX & dispatch (GCS → FC)

Pure NavLink v2 uplink. A command gate **rejects every command until the clock is
time-synced**; `COMMAND_ACK` is emitted automatically for ack-requiring commands.

```mermaid
flowchart TD
    classDef task fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef ipc  fill:#f3e8fd,stroke:#7b2cbf,color:#111;

    RX["USART6 RX ISR<br/>uart2_packet_recv_callback"] -->|byte| RING[["rx_raw_buf<br/>512 B SPSC"]]:::ipc
    RING --> POLL["comm_processor · prio 0 @250 Hz<br/>navlink_router_poll → parser"]:::task
    POLL --> GATE{"time-synced?<br/>(command gate)"}:::task
    GATE -->|no| REJ["COMMAND_ACK = TEMPORARILY_REJECTED"]:::task
    GATE -->|yes| DISP{"msgid"}:::task

    DISP -->|CMD_SET_PID 8195| H1["pid_config_apply_command → SD"]:::task
    DISP -->|CMD_ARM 8192| H2["g_sw_arm_request=1<br/>deferred ack (ARMED→ACCEPTED / 800 ms→TEMP_REJ)"]:::task
    DISP -->|CMD_DISARM 8193| H3["clear arm latch"]:::task
    DISP -->|CMD_CALIBRATE_IMU 8194| H4["spawn calibration_task<br/>(STANDBY/FAILSAFE only)"]:::task
    DISP -->|CMD_SET_GYRO_LPF 8196| H5["set gyro LPF RC"]:::task
    DISP -->|CMD_SET_MOTOR_GEOMETRY 8197| H6["mixer signs"]:::task
    DISP -->|CMD_SET_FLIGHT_MODE 8198| H7["ANGLE / ACRO / release-to-RC"]:::task
    DISP -->|TIME_SYNC 10| H8["apply offset, stamp t2/t3, reply"]:::task
    DISP -->|HEARTBEAT 0| H9["set_device_id only (liveness)"]:::task

    H1 --> ACK["router_send → COMMAND_ACK 5"]:::task
    H5 --> ACK
    H6 --> ACK
    H7 --> ACK
```

*Source: `src/comm/serializer.c`, `src/comm/navlink_router.c:120-308`,
`src/comm/comm_processor.c:42-142`. Heartbeat RX is liveness/device-id only — no
clock jam (that is the dedicated TIME_SYNC path).*

---

## 7. Telemetry TX (FC → GCS)

`imu_telemetry_task` runs a 166 Hz base loop and gates each message by
`packet_counter % N`. Output is buffered in `g_telemetry_channel` (ping-pong) and
DMA-flushed by `flush_task`. `perf_telemetry_task` emits kernel/IPC stats at 1 Hz.

```mermaid
flowchart LR
    classDef task fill:#e8f0fe,stroke:#3367d6,color:#111;
    classDef ipc  fill:#f3e8fd,stroke:#7b2cbf,color:#111;

    TELE["imu_telemetry · @166 Hz<br/>packet_counter % N gates"]:::task
    TELE -->|"%6"| M2["IMU_COMPRESSED 1025 (~25 Hz)"]
    TELE -->|"%100"| M1["IMU_RAW 1024 (1 Hz)"]
    TELE -->|"%15"| M3["ATTITUDE_EULER 1026 (~10 Hz)"]
    TELE -->|"%15"| M4["RC_CHANNELS 1028 (~10 Hz)"]
    TELE -->|"%8"| M5["MOTOR_TELEMETRY 1029 (~18 Hz)"]
    TELE -->|"%8"| M6["CONTROL_TRACE 1030 (~18 Hz)"]
    TELE -->|"%50"| M7["HEARTBEAT 0 · SYSTEM_HEALTH 2 · FLIGHT_MODE 3 (2 Hz)"]
    TELE -->|"%10"| M8["STATUSTEXT 4 (log drain)"]
    TELE -->|event| M9["CALIBRATION_STATUS 12320 · EST_PERF 1033"]

    PERF["perf_telemetry · @1 Hz"]:::task --> M10["PERF_GLOBAL 1034 · PERF_TASK 1035 · PERF_FIFO 1036"]

    M1 & M2 & M3 & M4 & M5 & M6 & M7 & M8 & M9 & M10 --> CH[["g_telemetry_channel<br/>ping-pong 512 B"]]:::ipc
    CH -->|"flush · @1 kHz · DMA"| OUT(["USART6 → GCS"])
```

`imu_telemetry_task` also runs `time_sync_discipline_tick()` every loop to slew the
disciplined clock between sync handshakes.

*Source: `src/comm/telemetry_task.c:26-135`, `src/comm/navlink_tx.c`,
`src/comm/perf_telemetry.c`, `src/comm/channel.c`. Msgids per
[`navlink/dialect.json`](../../navlink/dialect.json).*

---

## Reference tables

### Tasks

| Task | Entry | Stack | Prio | Rate / trigger |
|------|-------|------:|:----:|----------------|
| `comm_processor` | `comm_processor_task` | 2048 | 0 | poll @~250 Hz |
| `imu_read` | `bmx160_initiate_read` | 1536 | 2 | event (2 kHz tick + DMA sema) |
| `attitude` | `attitude_task` | 2048 | 1 | event (imu_attitude_queue) |
| `rc_ibus` | `rc_ibus_task` | 1024 | 0 | event (USART2 idle ISR) |
| `angle_ctl` | `angle_controller_task` | 2048 | 1 | periodic 250 Hz |
| `rate_ctl` | `angle_rate_controller_task` | 2048 | 1 | periodic 1000 Hz |
| `motor` | `motor_task` | 1024 | 1 | poll @~500 Hz |
| `imu_telemetry` | `imu_telemetry_task` | 2048 | 0 | poll @~166 Hz |
| `flush` | `flush_task` | 1024 | 0 | poll @~1 kHz |
| `perf_telemetry` | `perf_telemetry_task` | 2048 | 0 | 1 Hz |
| `heartbeat` | `heartbeat_task` | 1024 | 0 | ≥250 ms |
| `boot` | `boot_task` | 1024 | 0 | once → exit |
| `calibration_task` | (on demand) | 8192 | 0 | spawned by CMD_CALIBRATE_IMU |

### Key IPC (all SPSC overwrite rings unless noted)

| Queue | Producer → Consumer |
|-------|---------------------|
| `imu_telemetry_queue` | imu_read → imu_telemetry |
| `imu_control_queue` (+sema) | imu_read → rate_ctl |
| `imu_attitude_queue` (+sema) | imu_read → attitude |
| `imu_calibration_queue` | imu_read → calibration_task |
| `attitude_control_queue` (+sema) | attitude → angle_ctl |
| `attitude_telemetry_queue` | attitude → imu_telemetry |
| `angle_controller_fifo` | angle_ctl → rate_ctl |
| `motor_queue` | rate_ctl → motor |
| `motor_telemetry_queue` | motor → imu_telemetry |
| `control_telemetry_queue` | rate_ctl → imu_telemetry |
| `rc_control_queue` / `rc_telemetry_queue` | rc_ibus → angle_ctl / imu_telemetry |
| `rx_raw_buf` | USART6 RX ISR → comm_processor |
| `g_telemetry_channel` (ping-pong) | all TX tasks → flush → USART6 |

---

## Notes & caveats (verified honesty)

- **ESC PWM rate:** `motor_task`'s software cadence is ~500 Hz; the actual ESC PWM
  rate is set by the TIM1 config in `esc_init` and is **not asserted here** (not read).
- **EKF internals** (`src/est/ekf.c`) are summarised at the dispatch level only.
- **Telemetry UART:** the channel opens on **USART6**, but `flush_channel` special-cases
  USART2/DMA1-Stream6; on USART6 it currently falls back to a blocking byte write — a
  latent inconsistency to confirm, not architecture.
- **Stale in-source comments** in `attitude_task.c` (decimation target) and both
  controllers (`imu_queue_control_wait` vs the actual `task_delay_until`) describe older
  behavior; the diagrams above follow the **code**, not those comments.
- `PREARM` / `IN_AIR` / `TERMINATED` states exist in the enum but have no live driving
  transition yet.
