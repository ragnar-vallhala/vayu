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
/**
 * @file perf_telemetry.c
 * @brief Streams the vaios kernel/observability snapshot to the GCS.
 *
 * Once per period the task captures a coherent perf snapshot (scheduler / ISR /
 * IPC / heap), the live task list (cycles, switch-ins, stack high-water) and
 * the SPSC FIFO stats (peak fill, capacity, drops) of every app buffer, then
 * emits them as NavLink v2 PERF_GLOBAL + one PERF_TASK / PERF_FIFO message per
 * row, all tied by a per-report sequence number.
 *
 * Everything here degrades to zeros when VAIOS_MODULE_PERF is off (the vaios
 * perf API ships no-op inlines), so the GCS still gets a valid, if empty,
 * report and the task costs only its periodic wakeup.
 *
 * The wire encoding lives in the navlink_tx.c seam; this file only gathers the
 * firmware perf structs and hands them to navlink_tx_perf_*().
 */

#include "comm/perf_telemetry.h"
#include "comm/comm_types.h"
#include "comm/navlink_tx.h"
#include "comm/rc_buffer.h"
#include "control/control_buffer.h"
#include "memory.h"
#include "perf.h"
#include "sensor/imu_buffer.h"
#include "structure.h"
#include "task.h"
#include "vaios.h"
#include "variables.h"
#include <string.h>

/* Report cadence (ms). 1 Hz is plenty for stack/heap/fifo trend watching and
 * keeps the telemetry link uncongested next to the IMU/attitude streams. */
/* One report emits a frame PER TASK plus one per FIFO — ~27 frames in a single
 * burst, which is both ~13% of the link's frame budget and exactly the shape
 * the ESP bridge's RX ring handles worst. Stretched from 1 s to 5 s: stack
 * watermarks and CPU shares are slow-moving, and this is the readout that
 * catches a task creeping toward the overflow that panics the kernel at boot,
 * so it is slowed rather than switched off. */
#define PERF_REPORT_PERIOD_MS 5000u

/* Local gather bounds — sized above the current task/fifo counts with margin. */
#define PERF_TASK_CAP 24
#define PERF_FIFO_CAP 16

/* perf_fifo_fill_row lives in perf_packet.c so the buffer modules can link it
 * without pulling in this task's kernel/serializer deps (e.g. SITL). */

static uint32_t _seq;

/** @noreq perf snapshot gather glue */
static void send_global(uint8_t total_tasks, uint8_t total_fifos) {
  perf_global_body_t g;
  v_perf_snapshot_t s;

  v_perf_snapshot(&s);

  memset(&g, 0, sizeof(g));
#if VAIOS_MODULE_PERF
  g.flags = PERF_FLAG_ENABLED;
#endif
  g.total_tasks = total_tasks;
  g.total_fifos = total_fifos;
  g.uptime_ticks = s.uptime_ticks;
  g.sched_switches = s.sched_switches;
  g.cpu_cycles_lo = (uint32_t)s.cycles;
  g.idle_cycles_lo = (uint32_t)s.idle_cycles;
  g.systick_count = s.isr.systick_count;
  g.systick_last_cyc = s.isr.systick_last_cyc;
  g.systick_max_cyc = s.isr.systick_max_cyc;
  g.systick_preemptions = s.isr.systick_preemptions;
  g.ipc_takes = s.ipc.takes;
  g.ipc_takes_blocked = s.ipc.takes_blocked;
  g.ipc_gives = s.ipc.gives;
  g.ipc_timeouts = s.ipc.timeouts;
  g.heap_allocs = s.heap.allocs;
  g.heap_frees = s.heap.frees;
  g.heap_oom = s.heap.oom;
  g.heap_peak_bytes = s.heap.peak_bytes_in_use;
  g.heap_total_bytes = v_get_heap_size();

  navlink_tx_perf_global(&g, _seq);
}

/** @noreq per-task perf gather glue */
static void send_tasks(TCB **list, int n) {
  for (int i = 0; i < n; i++) {
    TCB *t = list[i];
    v_perf_task_t p;
    v_perf_task_stats(t, &p);
    perf_task_row_t row;
    row.task_id = (uint8_t)t->task_id;
    row.priority = (uint8_t)t->priority;
    row.state = (uint8_t)t->status;
    row._pad = 0;
    row.stack_peak = (uint16_t)p.stack_peak;
    row.stack_size = (uint16_t)p.stack_size;
    row.cycles_lo = (uint32_t)p.cycles_run;
    row.switches_in = p.switches_in;
    row.max_burst_cyc = p.max_burst_cyc;
    /* Name is resolved on demand via PERF_TASKNAME, not streamed. */
    navlink_tx_perf_task(&row, _seq);
  }
}

/** @noreq per-fifo perf gather glue */
static void send_fifos(const perf_fifo_row_t *rows, int n) {
  for (int i = 0; i < n; i++) {
    navlink_tx_perf_fifo(&rows[i], _seq);
  }
}

/** @implements COMM-TEL-005 */
void perf_telemetry_task(void *args) {
  (void)args;
  static perf_fifo_row_t fifos[PERF_FIFO_CAP];
  static TCB *tasks[PERF_TASK_CAP];

  while (1) {
    _seq++;

    int nf = 0;
    nf += rc_buffer_perf_fifos(&fifos[nf], PERF_FIFO_CAP - nf);
    nf += imu_buffer_perf_fifos(&fifos[nf], PERF_FIFO_CAP - nf);
    nf += control_buffer_perf_fifos(&fifos[nf], PERF_FIFO_CAP - nf);

    int nt = task_snapshot_list(tasks, PERF_TASK_CAP);

    send_global((uint8_t)nt, (uint8_t)nf);
    send_tasks(tasks, nt);
    send_fifos(fifos, nf);

    v_delay(PERF_REPORT_PERIOD_MS);
  }
}
