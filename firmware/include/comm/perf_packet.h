/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef VAYU_COMM_PERF_PACKET_H
#define VAYU_COMM_PERF_PACKET_H

/**
 * @file perf_packet.h
 * @brief Wire format for the kernel/observability telemetry packet
 *        (PACKET_TYPE_PERF_STATS, FC -> GCS).
 *
 * The vaios perf module (VAIOS_MODULE_PERF) exposes far more state than fits in
 * a single 255-byte telemetry payload once vayu runs ~13 tasks and ~10 SPSC
 * FIFOs. So a perf "report" is split into self-describing fragments that share
 * a sequence number:
 *
 *   [GLOBAL]            one packet: scheduler / ISR / IPC / heap snapshot
 *   [TASKS]  index 0..N chunks of per-task rows (cycles, switches, stack HWM)
 *   [FIFOS]  index 0..N chunks of per-FIFO rows (peak fill, capacity, drops)
 *
 * Every packet begins with perf_frag_hdr_t. The GCS starts a new report when it
 * sees a GLOBAL fragment, then accumulates TASKS/FIFOS rows until the next
 * GLOBAL (or `seq` change). All multi-byte fields are little-endian (Cortex-M4).
 */

#include <stdint.h>

#define PERF_SCHEMA_VERSION 1u

/* Section ids carried in perf_frag_hdr_t.section. */
#define PERF_SECTION_GLOBAL 0u
#define PERF_SECTION_TASKS 1u
#define PERF_SECTION_FIFOS 2u

/* GLOBAL flags. */
#define PERF_FLAG_ENABLED 0x01u /* VAIOS_MODULE_PERF compiled in & live */

/* Task names are NOT streamed in the perf report (they're static; sending them
 * every second wastes bandwidth). The GCS resolves id -> name on demand via the
 * PACKET_TYPE_PERF_TASKNAME request/response pair and caches the result.
 * PERF_TASKNAME_MAX bounds the name length carried in a response payload. */
#define PERF_TASKNAME_MAX 32

/* Per-report row caps, chosen so each packet stays well under 255 bytes:
 *   TASKS: 8 + 9*20 = 188   FIFOS: 8 + 16*8 = 136 */
#define PERF_TASKS_PER_PKT 9
#define PERF_FIFOS_PER_PKT 16

/* Stable FIFO ids so the GCS can label rows regardless of send order. */
typedef enum {
  PERF_FIFO_IMU_RAW = 0, /* reserved + not emitted; id kept stable for wire
                            compatibility with older GCS builds */
  PERF_FIFO_IMU_TELEMETRY = 1,
  PERF_FIFO_IMU_CONTROL = 2,
  PERF_FIFO_IMU_CALIB = 3,
  PERF_FIFO_IMU_CALIB_TELEM = 4,
  PERF_FIFO_ATTITUDE_TELEMETRY = 5,
  PERF_FIFO_ATTITUDE_CONTROL = 6,
  PERF_FIFO_RC_TELEMETRY = 7,
  PERF_FIFO_RC_CONTROL = 8,
  PERF_FIFO_TELEMETRY = 9,
} perf_fifo_id_t;

/* 8-byte fragment header — prefixes every PERF_STATS payload. */
typedef struct __attribute__((packed)) {
  uint8_t schema;  /* PERF_SCHEMA_VERSION                                  */
  uint8_t section; /* PERF_SECTION_*                                       */
  uint8_t index;   /* chunk index within the section (0-based)            */
  uint8_t count;   /* rows in this packet (TASKS/FIFOS); 0 for GLOBAL     */
  uint32_t seq;    /* report id — every fragment of one report shares it  */
} perf_frag_hdr_t;

/* GLOBAL body — follows the fragment header in a GLOBAL packet. */
typedef struct __attribute__((packed)) {
  uint8_t flags;       /* PERF_FLAG_*                                       */
  uint8_t total_tasks; /* task count in this report (across all chunks)    */
  uint8_t total_fifos; /* fifo count in this report                        */
  uint8_t _pad;
  uint32_t uptime_ticks;
  uint32_t sched_switches;
  uint32_t cpu_cycles_lo;  /* low 32 bits of the 64-bit cycle counter      */
  uint32_t idle_cycles_lo; /* low 32 bits of idle cycles                   */
  uint32_t systick_count;
  uint32_t systick_last_cyc;
  uint32_t systick_max_cyc;
  uint32_t systick_preemptions;
  uint32_t ipc_takes;
  uint32_t ipc_takes_blocked;
  uint32_t ipc_gives;
  uint32_t ipc_timeouts;
  uint32_t heap_allocs;
  uint32_t heap_frees;
  uint32_t heap_oom;
  uint32_t heap_peak_bytes;
  uint32_t
      heap_total_bytes; /* total heap pool size (for peak % usage)        */
} perf_global_body_t;

/* TASKS row — `count` of these follow the header in a TASKS packet. */
typedef struct __attribute__((packed)) {
  uint8_t task_id;
  uint8_t priority;
  uint8_t state; /* Task_Status                                            */
  uint8_t _pad;
  uint16_t stack_peak; /* high-water bytes used                            */
  uint16_t stack_size; /* total stack bytes                                */
  uint32_t cycles_lo;  /* low 32 bits of total cycles run                  */
  uint32_t switches_in;
  uint32_t max_burst_cyc;
} perf_task_row_t;

/* PACKET_TYPE_PERF_TASKNAME payloads (id <-> name resolution):
 *   GCS -> FC request : [uint8 task_id]                         (length 1)
 *   FC  -> GCS reply  : [uint8 task_id][char name[] NUL-term]   (length 1+n)
 * The direction disambiguates which is which. */
typedef struct __attribute__((packed)) {
  uint8_t task_id;
  /* reply only: NUL-terminated name follows, up to PERF_TASKNAME_MAX bytes */
  char name[PERF_TASKNAME_MAX];
} perf_taskname_t;

/* FIFOS row — `count` of these follow the header in a FIFOS packet. */
typedef struct __attribute__((packed)) {
  uint8_t fifo_id; /* perf_fifo_id_t                                       */
  uint8_t _pad;
  uint16_t peak;     /* high-water fill (elements)                         */
  uint16_t capacity; /* usable capacity (elements)                         */
  uint16_t drops;    /* items lost/overwritten (saturating at 0xFFFF)      */
} perf_fifo_row_t;

#endif /* VAYU_COMM_PERF_PACKET_H */
