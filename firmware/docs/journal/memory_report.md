# Vayu Memory Analysis Report

**Date**: 2026-03-24 (figures regenerated against the post-refactor tree)
**Hardware**: STM32F401RD (96 KB SRAM)
**Kernel**: VAIOS v1.0

> **Figures regenerated.** Heap and task-stack numbers below were updated to
> match the live tree: `HEAP_SIZE = 0xE000` (**56 KB**, from
> `include/vaios_app_config.h`) and the actual `task_create_named` calls in
> `src/main.c` (the modular-refactor split replaced `control_task` /
> `i2c_manager_task` / `imu_read_task` with `angle_controller_task`,
> `angle_rate_controller_task`, `attitude_task`, etc.). If you re-tune stacks,
> regenerate from those two sources.

## 1. SRAM Footprint Overview

The VAIOS heap is sized at `HEAP_SIZE = 0xE000` = **56 KB** (`include/vaios_app_config.h`),
deliberately reduced from the old `0x16000` kernel default which overran the 96 KB SRAM
once `.bss` grew (boot HardFault). `MAIN_STACK_SIZE` is 10 KB.

| Region           | Size    | Description                            |
| :--------------- | :------ | :------------------------------------- |
| **VAIOS Heap**   | 56.0 KB | OS heap for task stacks & IPC (`HEAP_SIZE = 0xE000`) |
| **.data / .bss + system stack** | remainder | Static globals, `main` stack (10 KB), ISRs |

## 2. Task Stack Audit

The following tasks are created on the 56 KB heap (from `src/main.c` `task_create_named`
calls; peaks are the in-code high-water comments where present).

| Task Name                     | Stack Size | Note / measured peak (bytes) |
| :---------------------------- | :--------- | :--------------------------- |
| `comm_processor_task`         | 2048       | peak ~748                    |
| `bmx160_initiate_read` (imu_read) | 1536   | peak ~404                    |
| `attitude_task`               | 2048       | fusion split out of IMU driver |
| `rc_ibus_task`                | 1024       | peak ~120                    |
| `angle_controller_task`       | 2048       | peak ~396 (control)          |
| `angle_rate_controller_task`  | 2048       | peak ~484 (control)          |
| `motor_task`                  | 1024       | peak ~252 (actuator)         |
| `imu_telemetry_task`          | 2048       | peak ~748                    |
| `flush_task`                  | 1024       | peak ~124                    |
| `perf_telemetry_task`         | 2048       | peak ~796                    |
| `heartbeat_task`              | 1024       | peak ~124                    |
| `boot_task`                   | 1024       |                              |

**Total Stack Allocation**: ~19.5 KB (19944 bytes)
**Remaining Heap**: ~36.5 KB (Available for IPC, Mutexes, Semaphores)

## 3. Leak & Fragmentation Analysis

### Dynamic Allocations (`v_malloc`)

1. **Task/IPC Creation**: Occurs only once during system startup. These are "permanent" allocations and do not cause fragmentation.
2. **Calibration Task**:
   - `v_malloc(sizeof(calibration_args_t))` is called in `comm_processor.c` when a calibration command is received.
   - **Verification**: `calibration_task` in `bmx160.c` explicitly calls `v_free(cal_args)` before exiting.
   - **Leak Status**: **NONE**. Life cycle is correctly managed.

### Risk Assessment

- **Stack Overflow**: the control path (`angle_controller_task` / `angle_rate_controller_task`) is the most complex but measured peaks are well under their 2 KB stacks (~396 / ~484 bytes).
- **Heap Exhaustion**: ~36 KB of free heap is more than enough for any additional runtime IPC objects.
- **Fragmentation**: With a 56 KB heap and no frequent alloc/free, fragmentation is negligible.

## 4. Recommendations

1. **Optimization**: several 2 KB stacks (telemetry, attitude) carry large margins over their measured peaks and could be trimmed if heap is needed.
2. **Monitoring**: If `v_malloc` is used for larger dynamic buffers in the future, implement a watermark check.
