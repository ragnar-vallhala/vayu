# Centralised Filesystem Owner (storage task)

## Context

Today every runtime SD/VFS **write** runs synchronously **in the caller's task
context** under the global `vfs_mutex`:

- `pid_config_save()` (`src/control/pid_config.c:94-105`) runs inside
  `comm_processor_task` when handling `CMD_SET_PID` / `CMD_SET_GYRO_LPF`. While it
  blocks on SD-block latency (~tens of ms), `comm_processor_task` is **not draining
  the 512 B UART6 RX ring** (~22 ms of headroom at 230400 baud), so inbound GCS
  commands silently overflow and are lost. This is the **C1→C3 chain** documented in
  `docs/scratch/resource-ownership-map.md` §8 — the one genuine unmitigated risk.
- Calib save (`src/sensor/bmx160.c:1918-1934`) runs in `calibration_task`.
- `logger_write()` (`src/logger/logger.c:142-166`) runs inline in any caller's
  context (currently only `src/utils/test_file.c:11`, but it is the planned
  binary-blackbox hot path).

**Goal:** introduce a single low-priority **FS owner task** that is the sole runtime
owner of all SD/VFS writes. Producers snapshot their data into a queue and return
immediately; the FS task performs `vfs_open/write/sync/close` serially off the
critical path. Boot-time reads stay synchronous (scheduler off, single-threaded).

**Decisions locked with the user:**
- **Scope = all runtime writes** (logger + PID save + calib save).
- **Full-queue policy = reserved save slots** — saves get a dedicated lane that
  logging can never starve. Implemented as a **two-lane** design (separate log
  queue + save queue, saves drained first).
- **The binary blackbox logger is fully absorbed** — there is no separate "logger"
  module layered over the FS owner. `src/logger/logger.c` is **deleted**; all of its
  responsibilities (boot prealloc/open of the 3 circular files, the fds, `write_pos`,
  `wrap_count`, the circular wrap/seek/write/sync, and the wrap-count accounting) move
  into `fs_owner`. One owner does it all — no two-level indirection.
- **`include/logger/logger.h` is deleted too** — there is one header. Its remaining
  text-log declarations (`vayu_log`, `vayu_log_queue`, `VAYU_LOG_QUEUE_SIZE`) move into
  `fs_owner.h`, which becomes the single storage+logging facade. Every `#include
  "logger/logger.h"` becomes `#include "storage/fs_owner.h"`.
  The text-log *implementation* `src/logger/log_text.c` stays as-is (it streams formatted
  text out over the telemetry LOG channel — it never touches SD); only its header home
  changes.

`extern/vaios/**` is kernel code and **off-limits** — no changes to `vfs.c` /
`structure.c`. All work stays in `src/`, `include/`, and CMake.

## New module

- `include/storage/fs_owner.h`
- `src/storage/fs_owner.c`

`fs_owner.h` is the single storage+logging header — it absorbs everything that was in
`logger.h` (both the blackbox type and the text-log API):

```c
#include "structure.h"   /* mpmc_queue_t (for vayu_log_queue) */
#include <stdint.h>
#include <stdbool.h>

/* ---- Text log (impl in src/logger/log_text.c) — moved verbatim from logger.h ---- */
extern mpmc_queue_t vayu_log_queue;
#define VAYU_LOG_QUEUE_SIZE 128
void vayu_log(const char *fmt, ...);

/* ---- Binary blackbox: the 3 circular SD files ---- */
typedef enum { NAVLINK_LOGGER, SYSTEM_LOGGER, GENERAL_LOGGER } logger_type_t;

/* Boot: prealloc + open the 3 circular files (direct vfs_*, scheduler off).
   Replaces logger_init(); called from main() before scheduler_start(). */
void fs_owner_boot_init(void);

void fs_owner_init(void);            /* create both queues; lazy, scheduler-live */
void fs_owner_task(void *args);      /* storage task entry (also declared in vayu_tasks.h) */

/* Producers — all snapshot (memcpy) their payload in and return immediately. */
bool fs_owner_enqueue_log(logger_type_t type, const void *data, uint32_t len);  /* was logger_write */
bool fs_owner_enqueue_pid_save(const void *store, uint32_t len);
bool fs_owner_enqueue_calib_save(const void *header, uint32_t hlen,
                                 const void *payload, uint32_t plen);

/* Wrap accounting (LOG-SD-002), moved here from logger.c. */
uint32_t fs_owner_log_wrap_count(logger_type_t type);
uint32_t fs_owner_log_wrap_count_total(void);   /* called by telemetry_task.c */

/* Drop accounting (mirrors COMM-CH-002). */
uint32_t fs_owner_dropped_logs(void);
uint32_t fs_owner_dropped_saves(void);
```

Two init phases (no per-logger mutexes — single consumer):
- **`fs_owner_boot_init()`** runs at boot, replacing `logger_init()`: prealloc + open
  `0:v_nav.bin` / `0:v_sys.bin` / `0:v_gen.bin` (the current `logger_init()` body,
  `logger.c:95-137`, minus the `v_mutex_create()` calls), set the 3 fds, zero
  `write_pos`/`wrap_count`. Direct `vfs_*`, single-threaded, scheduler off.
- **`fs_owner_init()`** runs as the **first line of `fs_owner_task()`** (lazy, like
  `vayu_log`), because `mpmc_init` creates a mutex + semaphores and needs the scheduler
  running. A `static volatile bool s_ready` is set at the end; every `fs_owner_enqueue_*`
  returns `false` (and counts a drop) until `s_ready` is true, so an early producer can
  never touch an uninitialised queue.

## Two-lane queue design (reserved save slots)

Two separate MPMC queues so a flood of logs can never displace a save:

```c
typedef enum { FS_REQ_PID_SAVE = 0, FS_REQ_CALIB_SAVE } fs_save_type_t;

#define FS_LOG_PAYLOAD_MAX  256   /* >= max navlink log line */
#define FS_SAVE_PAYLOAD_MAX 160   /* pid_store_t ~140B; calib hdr(8)+payload(84)=92B */

typedef struct {                  /* log lane element */
  uint8_t  logger_type;           /* logger_type_t */
  uint16_t len;
  uint8_t  payload[FS_LOG_PAYLOAD_MAX];
} fs_log_req_t;                   /* ~260 B */

typedef struct {                  /* save lane element */
  uint8_t  type;                  /* fs_save_type_t */
  uint16_t len;                   /* total bytes in payload (calib = hdr+payload concatenated) */
  uint8_t  payload[FS_SAVE_PAYLOAD_MAX];
} fs_save_req_t;                  /* ~164 B */
```

Static backing buffers in `fs_owner.c`:
- **Log lane:** `s_log_queue`, capacity **32** → `32 × 260 ≈ 8.3 KB`. `MPMC_POLICY_DROP`.
- **Save lane:** `s_save_queue`, capacity **4** → `4 × 164 ≈ 0.6 KB`. `MPMC_POLICY_DROP`.

Total ≈ **9 KB** of 96 KB SRAM (plus two small mpmc control structs). If the binary-log
hot path is later connected and 32 proves shallow, bump the log lane to 48–64.

**Copy-into-queue, not pointer-passing** — mandatory for lifetime/torn-read safety:
each `fs_owner_enqueue_*` `memcpy`s the payload into the element it pushes, in the
producer's context, before returning. The consumer only ever sees a fully-copied,
self-consistent snapshot; no `s_store` / `bmx160_calib` pointer crosses the task
boundary. (All `CMD_SET_PID` handlers are serialized on the single comm task, so
`s_store` is consistent at snapshot time.)

**Push policy per lane:**
- **Logs** → `mpmc_try_push` (non-blocking); on full, `s_dropped_logs++` and return
  false. Lossy logging is acceptable (consistent with COMM-CH-002 drop-and-count).
- **Saves** → `mpmc_try_push` into the dedicated 4-slot lane. Because logs can never
  occupy this lane and saves are rare (2 types, command-triggered), a slot is
  essentially always free; on the pathological full case, `s_dropped_saves++` and
  return false. **No blocking in the comm task** — this is the whole point. Persistence
  is best-effort; the live-controller apply already succeeded and is authoritative.

## FS task loop (saves take precedence)

```c
void fs_owner_task(void *args) {
  (void)args;
  fs_owner_init();
  fs_save_req_t sreq; fs_log_req_t lreq;
  for (;;) {
    /* Drain the reserved save lane fully first. */
    while (mpmc_try_pop(&s_save_queue, &sreq)) fs_do_save(&sreq);
    /* Then block briefly for a log; the timeout bounds save latency. */
    if (mpmc_pop_timeout(&s_log_queue, &lreq, FS_POLL_TICKS /* ~5-10 ms */))
      fs_do_log(&lreq);
  }
}
```

- `fs_do_save()` switches on `type`:
  - **PID:** open `0:pid.bin` `O_WRONLY|O_CREAT|O_TRUNC` → `vfs_write(fd, payload, len)`
    → `vfs_sync` → `vfs_close` (current `pid_config.c:96-104` body on the snapshot).
  - **CALIB:** open `0:cal.bin` `O_WRONLY|O_CREAT|O_TRUNC` → single `vfs_write` of the
    concatenated hdr+payload (`len==92`) → close (current `bmx160.c:1919-1930` body).
- `fs_do_log()` is the current `logger_write_internal` body
  (`logger.c:51-71`): wrap check → `vfs_lseek` → `vfs_write` → `vfs_sync` → advance
  `write_pos`, **with the per-logger `v_mutex_lock`/`unlock` removed** (single consumer).
  `write_pos[3]`, `wrap_count[3]`, the `*_FILE_SIZE` selection, and the 3 fds are all
  `fs_owner.c` statics now (set by `fs_owner_boot_init()`) — no getters needed.

**Stack 2048, priority 0** (lowest band, same as comm/telemetry/flush). It blocks on
the queue so it only runs when there is work, and never preempts control (prio 1) or
IMU (prio 2). Re-check stack via the perf high-water view after load (per `main.c:80`).

## Migration of call sites

- **Delete `src/logger/logger.c` entirely.** Its whole body moves into `fs_owner.c`:
  - `logger_init()` (`:95-137`) → `fs_owner_boot_init()` (drop the `v_mutex_create()`
    calls `:97-99` — single consumer, no per-logger mutexes).
  - `logger_write_internal()` (`:41-74`) → `fs_do_log()` (mutex-free, run by the FS task).
  - `logger_write()` (`:142-166`) → callers now call `fs_owner_enqueue_log()`.
  - `logger_wrap_count()` / `logger_wrap_count_total()` (`:169-183`) →
    `fs_owner_log_wrap_count[_total]()`; counters are `volatile uint32_t`, single writer
    (FS task) / multi reader — safe lock-free read.
  - `get_*_logger_fd()` (`:188-190`) → **dropped** (no external callers; fds are
    `fs_owner.c` statics).
- **Delete `include/logger/logger.h`.** Everything in it moves to `fs_owner.h`: the
  blackbox decls drop their old forms (`logger_init`, `logger_write`, `get_*_logger_fd`,
  `logger_wrap_count*`) and `logger_type_t`; the text-log API (`vayu_log`,
  `vayu_log_queue`, `VAYU_LOG_QUEUE_SIZE`) moves verbatim. `src/logger/log_text.c`
  (the `vayu_log` impl) is otherwise unchanged.
- **Include swap — every `#include "logger/logger.h"` → `#include "storage/fs_owner.h"`**
  (13 files): `src/utils/test_file.c`, `src/logger/log_text.c`, `src/control/pid_config.c`,
  `src/sensor/{bmx160,i2c_manager,bme280}.c`, `src/main.c`, `src/comm/telemetry_task.c`,
  `src/sys/{assert,state}.c`, `src/est/sensor_fusion.c`, `tools/sim_host/src/host_lifecycle.c`.
  (`src/logger/logger.c` is deleted, so its include goes with it.)
- **Symbol/API caller updates** (the full surface, from grep):
  - `src/main.c:153` `logger_init()` → `fs_owner_boot_init()`.
  - `src/utils/test_file.c:11` `logger_write(...)` → `fs_owner_enqueue_log(...)`.
  - `src/comm/telemetry_task.c:107` `logger_wrap_count_total()` →
    `fs_owner_log_wrap_count_total()`.
  - `include/control/pid_config.h:48` comment mentioning `logger_init` → `fs_owner_boot_init`.

- **`pid_config_save()`** (`src/control/pid_config.c:94-105`) becomes:
  ```c
  s_store.magic = PID_CONFIG_MAGIC;
  fs_owner_enqueue_pid_save(&s_store, sizeof s_store);
  ```
  The live-controller apply (`:147-161` / `:184-188`) stays synchronous and runs before
  the enqueue — unchanged. Boot load `pid_config_init()` (`:43-56`) stays direct `vfs_*`.

- **Calib save** (`src/sensor/bmx160.c:1918-1934`) becomes:
  ```c
  calib_file_header_t hdr = {CALIB_FILE_MAGIC, CALIB_FILE_VERSION,
                             (uint16_t)sizeof(bmx160_calibration_t)};
  calib_ok = fs_owner_enqueue_calib_save(&hdr, sizeof hdr,
                                         &bmx160_calib, sizeof bmx160_calibration_t);
  ```
  Boot calib load (`:188-205`) stays direct `vfs_*`. **ACK-timing change to note:**
  `calib_ok` / the terminal `CALIB_UPDATE_COMPLETE` telemetry (`:1945-1948`) now means
  "successfully enqueued," not "written to SD." Acceptable (calib save was never on the
  comm path), but called out for the GCS-wizard semantics.

## Boot vs runtime ordering

- Boot reads/setup stay direct `vfs_*` (scheduler off): `pid_config_init()`,
  `bmx160_init()` calib load, and `fs_owner_boot_init()` (the former `logger_init()`
  prealloc/open) — all in `main()`. `main.c:153` `logger_init()` becomes
  `fs_owner_boot_init()`; the rest of the boot sequence is unchanged.
- Add the FS task to `init_tasks()` in `src/main.c` (next to `flush_task`, `~:109`):
  ```c
  task_create_named(fs_owner_task, NULL, 2048, 0, "fs_owner");
  ```
  Created before `scheduler_start()` (`:164`) and after `logger_init()`/`pid_config_init()`
  (already guaranteed). Declare `fs_owner_task` in `include/vayu_tasks.h`.
- The `s_ready` flag (above) guards the window between `scheduler_start()` and the FS
  task's first run, so any early enqueue is a counted no-op rather than a crash.
- **`vfs_mutex` stays as-is.** With a single runtime writer it becomes uncontended (one
  take/give, near-free); removing it would mean touching off-limits kernel code for no
  real gain. The win comes from moving the *blocking* off the comm task, not the lock.

## Build wiring

Add `src/storage/fs_owner.c` and **remove `src/logger/logger.c`** from **both** the
firmware target and the SITL target (`CMakeLists.txt` and `tools/sim_host/CMakeLists.txt`
— wherever the other `src/**` sources are listed/globbed). Keep `src/logger/log_text.c`.
If sources are globbed rather than listed, only the add/keep matters; if listed
explicitly, drop the `logger.c` line. A stale `logger.c` reference is the most likely
build break.

## Verification

1. **Build** target + SITL (`-DVAYU_SIM`); confirm `logger.c` is gone from both targets,
   no dangling refs to `logger_init`/`logger_write`/`logger_wrap_count*`/`get_*_logger_fd`,
   and `log_text.c` (`vayu_log`) still links.
2. **Snapshot/torn-read unit (host/SITL):** enqueue a PID save, mutate `s_store`, let the
   FS task run, read back `0:pid.bin` → equals the *pre-mutation* snapshot.
3. **SITL boot** via the headless harness (`software/headless-sdk`, `vayu-headless`):
   confirm `pid.bin`/`cal.bin`/`v_*.bin` still created at boot and `fs_owner` appears in
   the task/perf list.
4. **Persistence round-trip:** send one `CMD_SET_PID` (0x000A), restart SITL, read gains
   back — they survived (FS task actually wrote the file).
5. **C1→C3 acceptance (core):** with logging active, send 20–50 `CMD_SET_PID` back-to-back
   faster than ~22 ms apart while other inbound traffic hits the UART6 RX ring. Assert
   **(a)** every command honored (query gains back), **(b)** the serializer RX-ring
   overflow/drop counter stays flat (previously the synchronous save dropped RX during the
   ~22 ms window).
6. **Drop accounting:** flood the log lane to force it full → `fs_owner_dropped_logs()`
   increments, and saves still complete (reserved lane untouched). Confirm
   `fs_owner_dropped_saves()` stays 0 under that flood.
7. **Stack high-water & priority:** after rapid saves + log flood, check `fs_owner` stack
   margin (≥2.5×) and confirm rate/attitude loop timing is unperturbed (prio-0 FS task
   never preempts control).

## Critical files

- `include/storage/fs_owner.h`, `src/storage/fs_owner.c` *(new)* — absorbs the whole
  binary blackbox logger (boot init, circular write, wrap counts, queues, saves)
- `src/logger/logger.c` — **deleted** (body moved into `fs_owner.c`)
- `include/logger/logger.h` — **deleted** (all decls moved into `fs_owner.h`)
- `src/logger/log_text.c` — body unchanged; include swapped to `storage/fs_owner.h`
- **13 files** swap `#include "logger/logger.h"` → `#include "storage/fs_owner.h"` (listed above)
- `src/control/pid_config.c` — `pid_config_save` → enqueue snapshot
- `src/sensor/bmx160.c` — calib save → enqueue snapshot
- `src/main.c` — `logger_init()` → `fs_owner_boot_init()`; create `fs_owner_task` in `init_tasks()`
- `src/comm/telemetry_task.c` — `logger_wrap_count_total()` → `fs_owner_log_wrap_count_total()`
- `src/utils/test_file.c` — `logger_write()` → `fs_owner_enqueue_log()`
- `include/vayu_tasks.h` — declare `fs_owner_task`
- `CMakeLists.txt` + `tools/sim_host/CMakeLists.txt` — add `fs_owner.c`, drop `logger.c`

## Risks

- **Calib ACK now means "enqueued," not "persisted"** — confirm GCS-wizard tolerance; if
  not, keep calib save synchronous (it's off the comm path) and migrate only PID + logs.
- **Save latency bounded by `FS_POLL_TICKS`** (~5–10 ms) since the task blocks on the log
  pop between save-lane sweeps — fine for persistence; tune if needed.
- **`logger_write()` is currently vestigial** (only `test_file.c`); the measured C1→C3
  contention is the PID/calib saves — verify the PID-save migration first as the real fix.
