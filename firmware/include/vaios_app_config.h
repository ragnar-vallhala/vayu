#ifndef VAIOS_APP_CONFIG_H
#define VAIOS_APP_CONFIG_H

#define SYSTICK_PERIOD 1000 // in microseconds

#define TICKS_TO_MS(x) ((x * SYSTICK_PERIOD) / 1000)

/* vaios task.h provides a fallback MS_TO_TICKS under its own #ifndef; undo
 * it first so this app-config definition wins consistently regardless of
 * include order (silences -Wmacro-redefined). */
#ifdef MS_TO_TICKS
#undef MS_TO_TICKS
#endif
#define MS_TO_TICKS(x) ((x * 1000) / SYSTICK_PERIOD)

#define US_TO_TICKS(x) ((x * 1) / SYSTICK_PERIOD)

#define UART_LOGGING_ENABLE 1

#define UART_BAUDRATE                                                          \
  460800 // ~45 KiB/s. Raised from 230400 (the old ESP8266 cap);
// the bridge now has a 4 KiB RX ring to absorb WiFi-TX stalls at this rate. The ESP's
// FC_BAUD must match. See firmware/docs/plans/link-bandwidth-boost.md.

/* Telemetry base tick (ms) for imu_telemetry_task's main loop. The per-stream gates are
 * expressed in MILLISECONDS (TELEM_GATE in telemetry_task.c), so this sets the loop/flush
 * granularity WITHOUT changing any stream's effective rate: a smaller value runs the loop
 * faster and spreads emissions over more, smaller iterations -> smaller channel-buffer
 * bursts -> smoother/faster flushing. 2 ms (~500 Hz) default; 6 ms reproduces the original
 * tick periods exactly. (Gate floor is 1 tick, so at very small values low-period streams
 * saturate — useful as a coarse load knob.) */
#ifndef TELEM_BASE_MS
#define TELEM_BASE_MS 2
#endif

#define LOGGING_ENABLED 0

// Interrupts
#define __NVIC_PRIO_BITS 4

#define MAX_SYSCALL_INTERRUPT_PRIORITY (7 << (8 - __NVIC_PRIO_BITS))

#define DMA_MIN_THRESHOLD 16

// Scheduling
#define TIME_SLICE 2

// Memory
#define MAIN_STACK_SIZE 10240 // 10KB

// 40KB. vaios kernel/memory.c includes the vaios_config.h aggregator (which
// pulls in this app config ahead of the #ifndef-guarded kernel default), so this
// value reaches the heap sizing directly — no -DHEAP_SIZE override needed.
//
// Holds the boot task stacks + IPC (~18.6KB, right-sized to per-task stack
// high-water in init_tasks) plus the heaviest post-boot allocation, the
// calibration task's 8KB stack v_malloc'd on CMD_CALIBRATE_IMU — leaving ~21KB
// free post-boot. The kernel reserves HEAP_WATERMARK_THRESHOLD (1KB) and memsets
// HEAP_SIZE bytes from heap_start at boot, under the hard constraint
//     heap_start + HEAP_SIZE <= 0x20018000   (top of RAM)
// HEAP_WATERMARK_ENABLE (below) reports the free-heap high-water at runtime.
//
// #ifndef-guarded so the host SITL (no 96KB SRAM limit, and needs room for many
// 8KB task stacks under the real scheduler) can pass -DHEAP_SIZE; the target
// build sets no override and keeps 0xA000.
#ifndef HEAP_SIZE
#define HEAP_SIZE 0xA000
#endif

#define STACK_ALIGN_SIZE 8

#define HEAP_WATERMARK_ENABLE 1

#define HEAP_WATERMARK_THRESHOLD 1024

// Tasks
#define MAX_TASK_PRIORITY 7

#define IDLE_TASK_PRIORITY 0

/* Idle does only log-flush + dead-task GC + WFI; measured high-water ~100 B.
 * 512 keeps ~5x margin and frees ~1.5 KB SRAM vs the old 2048. (This app
 * override is what actually takes effect; the vaios default is also 512.) */
#define IDLE_TASK_STACK_SIZE 512

#define TASK_STACK_WATERMARK_ENABLE 1

#define TASK_STACK_OVERFLOW_THRESHOLD 256

// IPC
#define MAX_SEMAPHORE_COUNT 0xFFFFFFFF

#define STATIC_SEMAPHORE_SIZE 32

// Logging
#define COLOR_LOGGING 0

#define LOG_BUFFER_SIZE 32

#define LOG_MSG_MAX_LEN 64

#define LOG_BUFFER_STORAGE_SIZE 128

#define BUFFERED_LOGGING 1

#define MIN_LOG_LEVEL LOG_TRACE

#define ALLOWED_MODULES "ALL"

// Terminal
#define ENABLE_TERMINAL 1

#define TERMINAL_LOG_LEVEL MIN_LOG_LEVEL

#define CMD_BUFFER_SIZE 10

#define CMD_MAX_LEN 32

#define MAX_CMD_NUMBER 6

#define ESCAPE_SEQ_LEN 2

#define TERMINAL_TASK_STACK_SIZE 1024

// General macros
#define PANIC(msg) v_panic(__FILE__, __LINE__, msg)
#if defined(PANIC) && defined(NAVHAL)
#include "navhal.h"
#endif
#endif // !VAIOS_APP_CONFIG_H
