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

#define UART_BAUDRATE 230400 // ~22.5 KiB/s — sized for the ESP8266 WiFi relay

#define LOGGING_ENABLED 0

// Interrupts
#define __NVIC_PRIO_BITS 4

#define MAX_SYSCALL_INTERRUPT_PRIORITY (7 << (8 - __NVIC_PRIO_BITS))

#define DMA_MIN_THRESHOLD 16

// Scheduling
#define TIME_SLICE 2

// Memory
#define MAIN_STACK_SIZE 10240 // 10KB

// 56KB. vaios kernel/memory.c now includes the vaios_config.h aggregator
// (which pulls in this app config ahead of the #ifndef-guarded kernel default),
// so this value reaches the heap sizing directly — no -DHEAP_SIZE override
// needed. Sized to fit the 96KB SRAM; the old 0x16000 kernel default overran it
// once .bss grew past ~8KB (heap_start + 0x16000 > top of RAM) -> boot HardFault.
#define HEAP_SIZE 0xE000

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
