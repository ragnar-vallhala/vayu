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

    subgraph "Shared Resources"
        ProcessData -->|imu_buffer_push| IMUFifo[(SPSC FIFO: _imu_fifo)]
        ProcessData -->|imu_distribution_queue_push| IMUQueue[(MPMC Queue: _imu_distribution_queue)]
        ProcessData -->|m_mahony_filter| AttitudeData{{Attitude: _bmx_orientation}}
        AttitudeMutex[[bmx160_attitude_mutex]] -.-> AttitudeData
    end

    subgraph "Processing Tasks"
        IMUFifo -->|imu_buffer_peek| ControlTask[Control Task]
        IMUQueue -->|imu_distribution_queue_peek| TelemetryTask[Telemetry Task]
        AttitudeData -->|bmx160_get_attitude| ControlTask
        AttitudeData -->|bmx160_get_attitude| TelemetryTask
    end

    style TimerSema fill:#f9f,stroke:#333
    style DMASema fill:#f9f,stroke:#333
    style ManagerDMASema fill:#f9f,stroke:#333
    style DataReady fill:#f9f,stroke:#333
    style AttitudeMutex fill:#ff9,stroke:#333
    style IMUFifo fill:#bbf,stroke:#333
    style IMUQueue fill:#bbf,stroke:#333
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

    Note over T: Period = 1 ms trigger (but system not keeping up)

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
    Note right of IMU: ~200–300 µs (Mahony + LPF)

    IMU->>CTRL: push IMU data

    Note over IMU: MISSED next timer tick(s)
    Note over T,IMU: Effective loop ≈ 6 ms → 166 Hz
```
## Shared Resources & Synchronization

| Resource | Implementation | Sync Primitive | Producer | Consumer(s) | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **I2C Bus** | Physical Bus | `_i2c_sema` (Binary Semaphore) | I2C Manager Task | Any Task calling I2C | Ensures exclusive access during transactions. |
| **I2C Request Queue**| `i2c_queue_t` | `_queue_sema` + `_data_ready_sema` | BMX160 Task | I2C Manager Task | Queue for async I2C operations. |
| **I2C DMA Buffer** | `_rx_data` in [i2c_manager.c](file:///home/ragnar/Documents/Drone/vayu/src/drivers/i2c_manager.c) | `_dma_done_sema` (Binary Semaphore) | I2C DMA ISR | I2C Manager Task | Hardware buffer protected by task blocking. |
| **Sensor DMA Done** | `bmx160_dma_sema` | Binary Semaphore | I2C Manager Callback | BMX160 Task | Signals sensor task to start processing. |
| **IMU Buffer** | `_imu_fifo` (`spsc_fifo_t`) | Lock-free (Volatile head/tail) | BMX160 Task | Control Task | Overwrite policy; used for rate-loop PID feedback. |
| **IMU Dist. Queue** | `_imu_distribution_queue` (`mpmc_queue_t`) | Mutex + 2 Semaphores | BMX160 Task | Telemetry Task | Used for logging and remote monitoring. |
| **Attitude State** | `_bmx_orientation` (`attitude_t`) | `bmx160_attitude_mutex` (Mutex) | BMX160 Task | Control, Telemetry | Stores Euler angles/Quaternions from sensor fusion. |

## Detailed Sequence

1.  **Trigger**: Every 1ms, a high-frequency timer ISR signals `bmx160_timer_sema`.
2.  **Request**: The **BMX160 Task** wakes up and calls [i2c_manager_read_async()](file:///home/ragnar/Documents/Drone/vayu/src/drivers/i2c_manager.c#124-143), which pushes the request into `I2CQueue` and signals `_data_ready_sema`.
3.  **Acquisition**: The **I2C Manager Task** wakes up, takes `_i2c_sema` (bus mutex), initiates an asynchronous DMA read, and waits on `_dma_done_sema`.
4.  **Completion**: When the DMA transfer finishes, the **I2C DMA Callback** (ISR context):
    -   Copies data to the local sensor buffer.
    -   Signals `bmx160_dma_sema` to notify the sensor task.
    -   Signals `_dma_done_sema` to notify the manager task.
5.  **Task Resumption**:
    -   The **I2C Manager Task** unblocks and releases `_i2c_sema`.
    -   The **BMX160 Task** unblocks and calls [bmx160_process_data()](file:///home/ragnar/Documents/Drone/vayu/src/sensor/bmx160.c#860-1113).
6.  **Processing**: The **BMX160 Task** converts raw values, applies Low-Pass Filters (LPF) and calibration, and runs the **Mahony Filter** for attitude estimation.
7.  **Distribution**:
    -   Filtered IMU data is pushed to `_imu_fifo`.
    -   IMU data is pushed to `_imu_distribution_queue`.
    -   Fused attitude is updated in `_bmx_orientation` under `bmx160_attitude_mutex`.
8.  **Consumption**:
    -   **Control Task** peeks at `_imu_fifo` and `_bmx_orientation`.
    -   **Telemetry Task** pops from `_imu_distribution_queue` and peeks at `_bmx_orientation`.
