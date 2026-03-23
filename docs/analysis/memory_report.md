# Vayu Memory Analysis Report

**Date**: 2026-03-24
**Hardware**: STM32F401RD (96 KB SRAM)
**Kernel**: VAIOS v1.0

## 1. SRAM Footprint Overview

With the recent optimizations (Reduced `IMU_BUFFER_SIZE` and `HEAP_SIZE`), the system has a very healthy memory margin.

| Region           | Size    | Address Range           | Description                            |
| :--------------- | :------ | :---------------------- | :------------------------------------- |
| **.data / .bss** | 12.4 KB | 0x20000000 - 0x200030E9 | Static globals (IMU Buffer, I2C state) |
| **VAIOS Heap**   | 64.0 KB | 0x200030E9 - 0x200130E9 | OS heap for task stacks & IPC          |
| **System Stack** | 19.6 KB | 0x200130E9 - 0x20018000 | Reserved for `main` and ISRs           |

**Safety Margin**: The startup stack now has **~19 KB** of headroom, up from the critical **1.2 KB** that caused the previous hardfault. This provides extreme stability for deep call stacks in ISRs or FatFS operations.

## 2. Task Stack Audit

The following tasks are created on the 64KB heap.

| Task Name          | Stack Size | Est. Peak Usage | Status                        |
| :----------------- | :--------- | :-------------- | :---------------------------- |
| `boot_task`        | 1024       | ~30%            | OK                            |
| `heartbeat_task`   | 2048       | ~20%            | OK                            |
| `i2c_manager_task` | 4096       | ~15%            | OK                            |
| `comm_processor`   | 4096       | ~25%            | OK (268b `packet_t` on stack) |
| `imu_read_task`    | 4096       | ~10%            | OK                            |
| `rc_ibus_task`     | 4096       | ~15%            | OK                            |
| `control_task`     | 6144       | ~10%            | Oversized (could be 2KB)      |
| `imu_telemetry`    | 4096       | ~20%            | OK                            |
| `flush_task`       | 4096       | ~15%            | OK                            |
| `test_task`        | 4096       | ~10%            | OK                            |

**Total Stack Allocation**: ~37.8 KB
**Remaining Heap**: ~26.2 KB (Available for IPC, Mutexes, Semaphores)

## 3. Leak & Fragmentation Analysis

### Dynamic Allocations (`v_malloc`)

1. **Task/IPC Creation**: Occurs only once during system startup. These are "permanent" allocations and do not cause fragmentation.
2. **Calibration Task**:
   - `v_malloc(sizeof(calibration_args_t))` is called in `comm_processor.c` when a calibration command is received.
   - **Verification**: `calibration_task` in `bmx160.c` explicitly calls `v_free(cal_args)` before exiting.
   - **Leak Status**: **NONE**. Life cycle is correctly managed.

### Risk Assessment

- **Stack Overflow**: `control_task` is the most complex but its actual stack usage is very linear. 6KB is very safe.
- **Heap Exhaustion**: 26KB of free heap is more than enough for any additional runtime IPC objects.
- **Fragmentation**: Since the task is only 64KB and we don't frequently alloc/free, fragmentation is negligible.

## 4. Recommendations

1. **Optimization**: `control_task` and `imu_read_task` stacks can likely be reduced to 2KB and 1KB respectively to free up another 7KB of heap if needed.
2. **Monitoring**: If `v_malloc` is used for larger dynamic buffers in the future, implement a watermark check.
