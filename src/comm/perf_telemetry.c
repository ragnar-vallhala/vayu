/**
 * @file perf_telemetry.c
 * @brief Streams the vaios kernel/observability snapshot to the GCS.
 *
 * Once per period the task captures a coherent perf snapshot (scheduler / ISR /
 * IPC / heap), the live task list (cycles, switch-ins, stack high-water) and
 * the SPSC FIFO stats (peak fill, capacity, drops) of every app buffer, then
 * emits them as fragmented PACKET_TYPE_PERF_STATS packets — GLOBAL first, then
 * TASKS chunks, then FIFOS chunks, all tied by a per-report sequence number.
 *
 * Everything here degrades to zeros when VAIOS_MODULE_PERF is off (the vaios
 * perf API ships no-op inlines), so the GCS still gets a valid, if empty,
 * report and the task costs only its periodic wakeup.
 *
 * NOTE: this is the one telemetry producer that still calls send_packet()
 * directly rather than going through the navlink_tx.c TX seam — PERF is
 * inherently multi-frame (fragmented by section/seq) with no codec dependency,
 * so it's the documented exception. It joins the seam when PERF itself migrates
 * to NavLink v2. See include/comm/navlink_tx.h.
 */

#include "comm/perf_telemetry.h"
#include "comm/comm_types.h"
#include "comm/rc_buffer.h"
#include "comm/serializer.h"
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
#define PERF_REPORT_PERIOD_MS 1000u

/* Local gather bounds — sized above the current task/fifo counts with margin. */
#define PERF_TASK_CAP 24
#define PERF_FIFO_CAP 16

/* perf_fifo_fill_row lives in perf_packet.c so the buffer modules can link it
 * without pulling in this task's kernel/serializer deps (e.g. SITL). */

static uint32_t _seq;

static void send_global(uint8_t total_tasks, uint8_t total_fifos) {
  uint8_t buf[sizeof(perf_frag_hdr_t) + sizeof(perf_global_body_t)];
  perf_frag_hdr_t *h = (perf_frag_hdr_t *)buf;
  perf_global_body_t *g = (perf_global_body_t *)(buf + sizeof(*h));
  v_perf_snapshot_t s;

  v_perf_snapshot(&s);

  h->schema = PERF_SCHEMA_VERSION;
  h->section = PERF_SECTION_GLOBAL;
  h->index = 0;
  h->count = 0;
  h->seq = _seq;

  memset(g, 0, sizeof(*g));
#if VAIOS_MODULE_PERF
  g->flags = PERF_FLAG_ENABLED;
#endif
  g->total_tasks = total_tasks;
  g->total_fifos = total_fifos;
  g->uptime_ticks = s.uptime_ticks;
  g->sched_switches = s.sched_switches;
  g->cpu_cycles_lo = (uint32_t)s.cycles;
  g->idle_cycles_lo = (uint32_t)s.idle_cycles;
  g->systick_count = s.isr.systick_count;
  g->systick_last_cyc = s.isr.systick_last_cyc;
  g->systick_max_cyc = s.isr.systick_max_cyc;
  g->systick_preemptions = s.isr.systick_preemptions;
  g->ipc_takes = s.ipc.takes;
  g->ipc_takes_blocked = s.ipc.takes_blocked;
  g->ipc_gives = s.ipc.gives;
  g->ipc_timeouts = s.ipc.timeouts;
  g->heap_allocs = s.heap.allocs;
  g->heap_frees = s.heap.frees;
  g->heap_oom = s.heap.oom;
  g->heap_peak_bytes = s.heap.peak_bytes_in_use;
  g->heap_total_bytes = v_get_heap_size();

  send_packet(&g_telemetry_channel, PACKET_TYPE_PERF_STATS, buf, sizeof(buf));
}

static void send_tasks(TCB **list, int n) {
  uint8_t chunk_idx = 0;
  for (int sent = 0; sent < n;) {
    int chunk = n - sent;
    if (chunk > PERF_TASKS_PER_PKT)
      chunk = PERF_TASKS_PER_PKT;

    uint8_t buf[sizeof(perf_frag_hdr_t) +
                PERF_TASKS_PER_PKT * sizeof(perf_task_row_t)];
    perf_frag_hdr_t *h = (perf_frag_hdr_t *)buf;
    perf_task_row_t *rows = (perf_task_row_t *)(buf + sizeof(*h));

    h->schema = PERF_SCHEMA_VERSION;
    h->section = PERF_SECTION_TASKS;
    h->index = chunk_idx++;
    h->count = (uint8_t)chunk;
    h->seq = _seq;

    for (int i = 0; i < chunk; i++) {
      TCB *t = list[sent + i];
      v_perf_task_t p;
      v_perf_task_stats(t, &p);
      rows[i].task_id = (uint8_t)t->task_id;
      rows[i].priority = (uint8_t)t->priority;
      rows[i].state = (uint8_t)t->status;
      rows[i]._pad = 0;
      rows[i].stack_peak = (uint16_t)p.stack_peak;
      rows[i].stack_size = (uint16_t)p.stack_size;
      rows[i].cycles_lo = (uint32_t)p.cycles_run;
      rows[i].switches_in = p.switches_in;
      rows[i].max_burst_cyc = p.max_burst_cyc;
      /* Name is resolved on demand via PACKET_TYPE_PERF_TASKNAME, not streamed. */
    }

    uint8_t len = (uint8_t)(sizeof(perf_frag_hdr_t) +
                            (unsigned)chunk * sizeof(perf_task_row_t));
    send_packet(&g_telemetry_channel, PACKET_TYPE_PERF_STATS, buf, len);
    sent += chunk;
  }
}

static void send_fifos(const perf_fifo_row_t *rows, int n) {
  uint8_t chunk_idx = 0;
  for (int sent = 0; sent < n;) {
    int chunk = n - sent;
    if (chunk > PERF_FIFOS_PER_PKT)
      chunk = PERF_FIFOS_PER_PKT;

    uint8_t buf[sizeof(perf_frag_hdr_t) +
                PERF_FIFOS_PER_PKT * sizeof(perf_fifo_row_t)];
    perf_frag_hdr_t *h = (perf_frag_hdr_t *)buf;

    h->schema = PERF_SCHEMA_VERSION;
    h->section = PERF_SECTION_FIFOS;
    h->index = chunk_idx++;
    h->count = (uint8_t)chunk;
    h->seq = _seq;

    memcpy(buf + sizeof(*h), &rows[sent],
           (unsigned)chunk * sizeof(perf_fifo_row_t));

    uint8_t len = (uint8_t)(sizeof(perf_frag_hdr_t) +
                            (unsigned)chunk * sizeof(perf_fifo_row_t));
    send_packet(&g_telemetry_channel, PACKET_TYPE_PERF_STATS, buf, len);
    sent += chunk;
  }
}

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
