# Vayu Sensor Data Flow Map

This document maps the path of sensor data from the BMX160 hardware to the processing tasks (Control and Telemetry), highlighting shared buffers and synchronization primitives.

## Data Path Overview

```mermaid
graph TD
    subgraph "Interrupt Context"
        TimerISR[Timer ISR 1kHz] -->|v_semaphore_give| TimerSema((bmx160_timer_sema))
        DMACallback[I2C DMA Callback] -->|v_semaphore_give| DMASema((bmx160_dma_sema))
        DMACallback -->|v_semaphore_give| ManagerDMASema((_dma_done_sema))
    end

    subgraph "Sensor Task (bmx160_initiate_read)"
        TimerSema --> TaskWait[Wait for Timer]
        TaskWait --> I2CRequest[i2c_manager_read_async]
        I2CRequest -->|v_semaphore_take| DMASema
        DMASema --> ProcessData[bmx160_process_data]
    end

    subgraph "I2C Manager Task"
        I2CRequest -->|Push| I2CQueue[(I2C Request Queue)]
        I2CRequest -->|v_semaphore_give| DataReady((_data_ready_sema))
        DataReady --> ManagerWait[Wait for Request]
        ManagerWait --> BusLock[Take _i2c_sema]
        BusLock --> DMARead[hal_i2c_read_regs_dma]
        DMARead -.-> DMACallback
        ManagerDMASema --> ManagerDone[Wait for DMA Done]
        ManagerDone --> BusRelease[Give _i2c_sema]
    end

    subgraph "Shared Resources (SPSC rings, OVERWRITE)"
        ProcessData -->|imu_queue_control_push| IMUCtrl[(imu_queue_control)]
        ProcessData -->|imu_queue_telemetry_push| IMUTelem[(imu_queue_telemetry)]
        ProcessData -->|imu_queue_attitude_push| IMUAtt[(imu_queue_attitude)]
        ProcessData -->|imu_queue_calibration_push<br/>when CALIBRATING| IMUCal[(imu_queue_calibration)]
    end

    subgraph "Estimator (attitude_task)"
        IMUAtt -->|imu_queue_attitude_pop| EKF[attitude_task: EKF fusion]
        EKF -->|attitude_queue_control_push| AttCtrl[(attitude_queue_control)]
        EKF -->|attitude_queue_telemetry_push| AttTelem[(attitude_queue_telemetry)]
    end

    subgraph "Processing Tasks"
        IMUCtrl -->|imu_queue_control_pop| ControlTask[Control Task]
        AttCtrl -->|attitude_queue_control_pop| ControlTask
        IMUTelem -->|imu_queue_telemetry_pop| TelemetryTask[Telemetry Task]
        AttTelem -->|attitude_queue_telemetry_pop| TelemetryTask
    end

    style TimerSema fill:#f9f,stroke:#333
    style DMASema fill:#f9f,stroke:#333
    style ManagerDMASema fill:#f9f,stroke:#333
    style DataReady fill:#f9f,stroke:#333
    style AttitudeMutex fill:#ff9,stroke:#333
    style IMUCtrl fill:#bbf,stroke:#333
    style IMUTelem fill:#bbf,stroke:#333
```
```mermaid
sequenceDiagram
    participant T as Timer ISR (1 kHz)
    participant IMU as BMX160 Task
    participant Q as I2C Queue
    participant I2C as I2C Manager Task
    participant DMA as DMA Engine
    participant ISR as DMA Callback
    participant CTRL as Control Task

    Note over T: Period = 1 ms trigger

    T->>IMU: give(bmx160_timer_sema)
    Note right of IMU: Wake latency ~100–300 µs

    IMU->>Q: i2c_manager_read_async()
    Q->>I2C: give(_data_ready_sema)

    Note right of I2C: Scheduling delay ~100–300 µs

    I2C->>I2C: take(_i2c_sema)
    I2C->>DMA: start I2C + DMA

    Note over DMA: I2C transfer ~0.75 ms  (measured ~64k cycles)

    DMA-->>ISR: DMA complete IRQ

    Note right of ISR: ISR latency ~10–50 µs

    ISR->>I2C: give(_dma_done_sema)
    ISR->>IMU: give(bmx160_dma_sema)

    Note right of I2C: Wake latency ~100–300 µs
    I2C->>I2C: release(_i2c_sema)

    Note right of IMU: Wake latency ~100–300 µs

    IMU->>IMU: process_data()
    Note right of IMU: ~200–300 µs (convert + LPF + calibration)

    IMU->>CTRL: push IMU data
```
## Shared Resources & Synchronization

| Resource | Implementation | Sync Primitive | Producer | Consumer(s) | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **I2C Bus** | Physical Bus | `_i2c_sema` (Binary Semaphore) | I2C Manager Task | Any Task calling I2C | Ensures exclusive access during transactions. |
| **I2C Request Queue**| `i2c_queue_t` | `_queue_sema` + `_data_ready_sema` | BMX160 Task | I2C Manager Task | Queue for async I2C operations. |
| **I2C DMA Buffer** | `_rx_data` in [i2c_manager.c](../../../src/sensor/i2c_manager.c) | `_dma_done_sema` (Binary Semaphore) | I2C DMA ISR | I2C Manager Task | Hardware buffer protected by task blocking. |
| **Sensor DMA Done** | `bmx160_dma_sema` | Binary Semaphore | I2C Manager Callback | BMX160 Task | Signals sensor task to start processing. |
| **IMU control ring** | `imu_queue_control` (SPSC) | Lock-free + wake semaphore | BMX160 Task | Control Task | OVERWRITE policy; rate-loop PID feedback. |
| **IMU telemetry ring** | `imu_queue_telemetry` (SPSC) | Lock-free + wake semaphore | BMX160 Task | Telemetry Task | OVERWRITE; logging / remote monitoring. |
| **IMU attitude ring** | `imu_queue_attitude` (SPSC) | Lock-free + wake semaphore | BMX160 Task | `attitude_task` | OVERWRITE; feeds the estimator. |
| **Attitude estimate** | `attitude_queue_{control,telemetry}` (SPSC) | Lock-free + wake semaphore | `attitude_task` (EKF) | Control, Telemetry | Euler/quaternion from the EKF fusion (not the driver). |

## Detailed Sequence

1.  **Trigger**: Every 1ms, a high-frequency timer ISR signals `bmx160_timer_sema`.
2.  **Request**: The **BMX160 Task** wakes up and calls [i2c_manager_read_async()](../../../src/sensor/i2c_manager.c), which pushes the request into `I2CQueue` and signals `_data_ready_sema`.
3.  **Acquisition**: The **I2C Manager Task** wakes up, takes `_i2c_sema` (bus mutex), initiates an asynchronous DMA read, and waits on `_dma_done_sema`.
4.  **Completion**: When the DMA transfer finishes, the **I2C DMA Callback** (ISR context):
    -   Copies data to the local sensor buffer.
    -   Signals `bmx160_dma_sema` to notify the sensor task.
    -   Signals `_dma_done_sema` to notify the manager task.
5.  **Task Resumption**:
    -   The **I2C Manager Task** unblocks and releases `_i2c_sema`.
    -   The **BMX160 Task** unblocks and calls [bmx160_process_data()](../../../src/sensor/bmx160.c).
6.  **Processing**: The **BMX160 Task** converts raw values and applies Low-Pass Filters (LPF) + calibration. Attitude fusion is **not** done here — it runs in a separate `attitude_task` (see below).
7.  **Distribution**: the processed sample is pushed to the per-consumer SPSC rings: `imu_queue_control`, `imu_queue_telemetry`, `imu_queue_attitude`, and (while CALIBRATING) `imu_queue_calibration`.
8.  **Fusion**: `attitude_task` pops `imu_queue_attitude`, runs the **EKF** (`src/est/`), and publishes the estimate to `attitude_queue_control` and `attitude_queue_telemetry`.
9.  **Consumption**:
    -   **Control Task** pops `imu_queue_control` and `attitude_queue_control`.
    -   **Telemetry Task** pops `imu_queue_telemetry` and `attitude_queue_telemetry`.
