/**
 * @file src/storage/fs_owner.c
 * @brief Centralised filesystem owner — sole runtime owner of all SD/VFS writes.
 *
 * Owns the binary blackbox logger and serialises PID/calib persistence plus xfer
 * uploads behind queues, so the blocking SD I/O never runs in the producer's task
 * context. See firmware/docs/plans/centralised-fs-owner.md and
 * firmware/docs/scratch/resource-ownership-map.md (the C1->C3 chain this fixes).
 *
 * fd-frugal by design: the underlying FatFS exposes only MAX_OPEN_FILES (4)
 * file slots, so this owner NEVER holds a file open across calls. Every handler
 * (log, save, write-at, read-at) does open -> seek -> write/read -> sync ->
 * close and releases the slot immediately. Holding the 3 blackbox files open for
 * the whole run would consume 3 of the 4 slots and starve uploads / saves of a
 * slot, so their opens fail silently and produce 0-byte files.
 *
 * Resilient writes: a write whose open/write fails (e.g. all 4 slots momentarily
 * busy) is re-queued onto a retry lane and re-attempted whenever no fresh write
 * is pending — repeatedly, until it lands or a bounded retry budget is spent.
 * Per-session bookkeeping (pending/committed/failed) lets an xfer upload report
 * true persistence to the GCS rather than mere enqueue.
 *
 * Lanes, drained in priority order each tick: reserved saves (never starved) ->
 * upload write-ats (+ their retries when idle) -> best-effort blackbox logs.
 */
#include "storage/fs_owner.h"

#include "memory.h"               /* v_malloc (heap-backed write-at lanes) */
#include "utils.h"                /* v_memcpy */
#include "vaios_config_default.h" /* PANIC */
#include "variables.h" /* *_LOGGING_FILENAME/_FILE_SIZE, CALIBRATION_FILE_PATH */
#include "vfs.h"
#include "storage/imu_hs_log.h"

/* ===========================================================================
 * Sizing
 * =========================================================================== */
#define FS_LOG_PAYLOAD_MAX 256u /* >= max navlink blackbox record           */
#define FS_SAVE_PAYLOAD_MAX                                                    \
  216u /* pid_store 216B (PID5 + notch + motor geom); calib hdr(8)+payload(84) */
/* Log lane depth. KEEP SMALL: each slot is FS_LOG_PAYLOAD_MAX+ bytes of static
 * BSS, and the STM32F401 (96 KiB SRAM) is RAM-starved — a too-large queue pushes
 * _heap_start up until the kernel heap's HEAP_SIZE memset runs off the top of RAM
 * (silent: the linker can't see it), corrupting memory at boot -> HardFault. 4 is
 * ample; do NOT raise without checking _heap_start + HEAP_SIZE <= top-of-RAM. */
#define FS_LOG_QUEUE_CAP                                                       \
  4u /* ~1 KiB; a 32-deep lane (8.3 KiB) overflows F401 SRAM. */
#define FS_SAVE_QUEUE_CAP 4u /* reserved — logs can never occupy this lane */
#define FS_POLL_TICKS 5u /* save/write-at latency bound while blocked on logs */

/* Upload write-at lane (xfer substrate). The backing buffers are HEAP-allocated
 * (v_malloc) not static, so adding these lanes keeps .bss flat and doesn't shrink
 * the F401 MSP margin (firmware/docs/plans/xfer-memory-budget.md). */
#define FS_WRITEAT_PAYLOAD_MAX                                                 \
  247u                          /* = XFER_CHUNK_MAX (one XFER_DATA chunk)     */
#define FS_WRITEAT_PATH_MAX 40u /* >= XFER_ARG_MAX(32) SD path                */
#define FS_WRITEAT_QUEUE_CAP                                                   \
  2u /* fresh-request lane (one chunk of pipelining)*/
#define FS_WRITEAT_RETRY_CAP                                                   \
  4u /* failed-write retry lane                     */
#define FS_WRITEAT_MAX_TRIES                                                   \
  16u /* attempts before a write is deemed permanent */
/* Idle loops (task iterations) the cached write fd may sit unused before it is
 * closed. The task loops ~every FS_POLL_TICKS (ms) when idle, so ~10 loops ≈
 * 50 ms — long enough to span an upload's inter-chunk pacing gaps (so the file is
 * NOT reopened per chunk) yet short enough to release the slot well before a
 * follow-up download/list reads it. */
#define FS_WRITEAT_IDLE_CLOSE_LOOPS 10u
/* Read fd is held across a whole (slow, flow-controlled) download; this is only a
 * coarse backstop to release a stale handle long after reads have stopped. Must be
 * far larger than the worst inter-chunk gap in polls (a few KiB/s download => tens
 * of empty 5ms polls between reads) so it never fires mid-download. ~2s. */
#define FS_READ_FD_MAX_HOLD_LOOPS 400u
/* Per-session write-at bookkeeping. Must be >= XFER_MAX_SESSIONS (navlink_xfer.h);
 * kept as a local constant so fs_owner stays independent of the xfer headers. */
/* One slot per xfer session, PLUS one reserved for firmware-internal writers
 * (FS_WA_SESSION_INTERNAL). Without the spare, an internal save had to borrow
 * session 0 and would corrupt a concurrent GCS upload's pending/committed/
 * failed accounting — the xfer flow control reads those to decide DONE. */
#define FS_WA_SESSIONS 3u

#define FS_SAVE_MAX_TRIES 8u /* save retry budget (slots free quickly)     */

#define FS_PID_FILE_PATH "0:pid.bin" /* mirrors pid_config.c boot reader */

typedef enum { FS_SAVE_PID = 0, FS_SAVE_CALIB } fs_save_type_t;

typedef struct {
  uint8_t logger_type; /* logger_type_t */
  uint16_t len;
  uint8_t payload[FS_LOG_PAYLOAD_MAX];
} fs_log_req_t;

typedef struct {
  uint8_t type;  /* fs_save_type_t */
  uint8_t tries; /* retry counter */
  uint16_t len;  /* bytes used (calib = hdr+payload concatenated) */
  uint8_t payload[FS_SAVE_PAYLOAD_MAX];
} fs_save_req_t;

typedef struct {
  char path[FS_WRITEAT_PATH_MAX];
  uint32_t offset;
  uint16_t len;
  uint8_t session; /* xfer session for per-session bookkeeping */
  uint8_t tries;   /* retry counter */
  uint8_t payload[FS_WRITEAT_PAYLOAD_MAX];
} fs_writeat_req_t;

/* ===========================================================================
 * State
 * =========================================================================== */
static mpmc_queue_t s_log_q;
static mpmc_queue_t s_save_q;
static mpmc_queue_t s_writeat_q; /* fresh write-at requests */
static mpmc_queue_t s_retry_q;   /* failed write-ats awaiting another attempt */
static fs_log_req_t s_log_buf[FS_LOG_QUEUE_CAP];
static fs_save_req_t s_save_buf[FS_SAVE_QUEUE_CAP];
static fs_writeat_req_t *s_writeat_buf; /* heap (v_malloc in fs_owner_init) */
static fs_writeat_req_t *s_retry_buf;   /* heap (v_malloc in fs_owner_init) */
static volatile bool s_ready = false;

/* Per-session write-at bookkeeping (single writer = the FS task; readers = the
 * xfer tick polling flush status. volatile is enough for this 1:1 hand-off). */
static volatile uint32_t s_wa_pending[FS_WA_SESSIONS];
static volatile uint32_t s_wa_committed[FS_WA_SESSIONS];
static volatile uint8_t s_wa_failed[FS_WA_SESSIONS];

/* Lazy-open write cache: the upload file is opened ONCE and kept open across all
 * its chunks (each chunk does lseek + write + sync on the held fd), then closed
 * when the lane goes idle. Repeated open/close of a growing file is unreliable on
 * this SD/FatFS and loses data; a held fd + per-chunk sync mirrors the proven
 * blackbox-logger pattern. Only one file is cached, so at most one slot is held
 * (and only during an active transfer — never across transfers). */
static vfs_fd_t s_wfd = -1;
static char s_wpath[FS_WRITEAT_PATH_MAX];
static uint32_t s_wfd_idle_loops;

/* Lazy-open READ cache — the read-path twin of s_wfd. A download reads a file in
 * many small chunks; opening/closing the file per chunk re-traverses the
 * directory each time and reads back CORRUPT/non-deterministic data on this
 * SD/FatFS (same failure the write path hit). So the read file is opened ONCE and
 * held open across its chunks (each chunk does lseek + read on the held fd),
 * closed when a different file is read, when the file is truncated, or after the
 * read lane goes idle. FS-task-owned, like s_wfd. */
static vfs_fd_t s_rfd = -1;
static char s_rpath[FS_WRITEAT_PATH_MAX];
static uint32_t s_rfd_idle_loops;
static uint32_t
    s_rpos; /* current file position of s_rfd (avoid redundant lseek) */

/* Blackbox file state (open-on-demand; positions persist across writes). */
static uint32_t navlink_write_pos = 0;
static uint32_t system_write_pos = 0;
static uint32_t general_write_pos = 0;
static volatile uint32_t navlink_wrap_count = 0;
static volatile uint32_t system_wrap_count = 0;
static volatile uint32_t general_wrap_count = 0;

static volatile uint32_t s_dropped_logs = 0;
static volatile uint32_t s_dropped_saves = 0;
static volatile uint32_t s_dropped_writeats = 0;

/* When set, the FS task stops draining best-effort blackbox logs to SD so an
 * in-flight bulk transfer is the SOLE multi-file SD actor. Interleaving log
 * writes (file A) with the transfer's reads/writes (file B) corrupts the
 * FatFS/SD read-back even when serialised through this one task; sequential
 * single-file access is the only clean pattern. Driven by the xfer service task
 * from xfer_active(). Queued log records simply drop (circular blackbox) until
 * the transfer ends, then draining resumes. */
static volatile bool s_logs_suppressed = false;

/* ===========================================================================
 * Boot-time file setup — direct vfs_*, scheduler off.
 * Preallocate + size the 3 circular files, then CLOSE them: no fd is held.
 * =========================================================================== */
/** @implements LOG-SD-001 — extend each ring file to its preallocated size. */
static void ensure_file_size(const char *path, uint32_t file_size) {
  vfs_fd_t fd = vfs_open(path, VFS_O_RDWR | VFS_O_CREAT);
  if (fd < 0) {
    return;
  }
  long current_size = vfs_lseek(fd, 0, VFS_SEEK_END);
  if (current_size < (long)file_size) {
    vfs_lseek(fd, (long)(file_size - 1u), VFS_SEEK_SET);
    uint8_t dummy = 0;
    vfs_write(fd, &dummy, 1);
    vfs_sync(fd);
  }
  vfs_close(fd);
}

/** @implements LOG-SD-001 — preallocate + size the 3 circular blackbox files. */
void fs_owner_boot_init(void) {
  if (vfs_preallocate(NAVLINK_LOGGING_FILENAME, NAVLINK_LOGGING_FILE_SIZE) !=
      0) {
    PANIC("Navlink prealloc failed");
  }
  if (vfs_preallocate(SYS_LOGGING_FILENAME, SYS_LOGGING_FILE_SIZE) != 0) {
    PANIC("System prealloc failed");
  }
  if (vfs_preallocate(GENERAL_LOGGING_FILENAME, GENERAL_LOGGING_FILE_SIZE) !=
      0) {
    PANIC("General prealloc failed");
  }

  ensure_file_size(NAVLINK_LOGGING_FILENAME, NAVLINK_LOGGING_FILE_SIZE);
  ensure_file_size(SYS_LOGGING_FILENAME, SYS_LOGGING_FILE_SIZE);
  ensure_file_size(GENERAL_LOGGING_FILENAME, GENERAL_LOGGING_FILE_SIZE);

  navlink_write_pos = 0;
  system_write_pos = 0;
  general_write_pos = 0;

  imu_hs_log_boot_init();
}

/* ===========================================================================
 * Work handlers (run by the FS task / the test pump — never the producer).
 * =========================================================================== */

/* Circular blackbox write, open-on-demand. LOG-SD-002: wrapping discards the
 * oldest records, counted so the loss is accountable rather than silent. No mutex
 * — single consumer. Logs are best-effort: a failed open just drops the record.
 * @implements LOG-SD-002, LOG-SD-102 */
static void fs_circular_write(const char *path, uint32_t *write_pos,
                              volatile uint32_t *wrap_count, uint32_t file_size,
                              const uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0u || len > file_size) {
    return;
  }
  vfs_fd_t fd = vfs_open(path, VFS_O_RDWR | VFS_O_CREAT);
  if (fd < 0) {
    s_dropped_logs++;
    return;
  }
  if (*write_pos > (file_size - len)) {
    *write_pos = 0;
    (*wrap_count)++;
  }
  vfs_lseek(fd, (long)(*write_pos), VFS_SEEK_SET);
  int res = vfs_write(fd, data, len);
  int sync_res = vfs_sync(fd);
  vfs_close(fd);
  if (res >= 0 && sync_res >= 0) {
    *write_pos = (*write_pos + len) % (file_size - len);
  } else {
    s_dropped_logs++;
  }
}

/** @noreq routes a blackbox log record to its per-type ring file. */
static void fs_do_log_req(const fs_log_req_t *req) {
  switch ((logger_type_t)req->logger_type) {
  case NAVLINK_LOGGER:
    fs_circular_write(NAVLINK_LOGGING_FILENAME, &navlink_write_pos,
                      &navlink_wrap_count, NAVLINK_LOGGING_FILE_SIZE,
                      req->payload, req->len);
    break;
  case SYSTEM_LOGGER:
    fs_circular_write(SYS_LOGGING_FILENAME, &system_write_pos,
                      &system_wrap_count, SYS_LOGGING_FILE_SIZE, req->payload,
                      req->len);
    break;
  case GENERAL_LOGGER:
  default:
    fs_circular_write(GENERAL_LOGGING_FILENAME, &general_write_pos,
                      &general_wrap_count, GENERAL_LOGGING_FILE_SIZE,
                      req->payload, req->len);
    break;
  }
}

/* @return true on success, false on open/write failure (caller retries).
 * @implements LOG-PERSIST-001 */
static bool fs_do_save(const fs_save_req_t *req) {
  const char *path = ((fs_save_type_t)req->type == FS_SAVE_PID)
                         ? FS_PID_FILE_PATH
                         : CALIBRATION_FILE_PATH;
  vfs_fd_t fd = vfs_open(path, VFS_O_WRONLY | VFS_O_CREAT | VFS_O_TRUNC);
  if (fd < 0) {
    return false;
  }
  int w = vfs_write(fd, req->payload, req->len);
  int sync_res = vfs_sync(fd);
  vfs_close(fd);
  return (w == (int)req->len && sync_res >= 0);
}

/* @implements LOG-PERSIST-001 */
static void fs_drain_saves(void) {
  fs_save_req_t req;
  while (mpmc_try_pop(&s_save_q, &req)) {
    if (!fs_do_save(&req)) {
      /* Re-queue for another attempt (a slot will free shortly); give up only
       * after the retry budget — then drop-and-count. */
      req.tries++;
      if (req.tries < FS_SAVE_MAX_TRIES && mpmc_try_push(&s_save_q, &req)) {
        break; /* leave it queued; next tick retries (paced by the task loop) */
      }
      s_dropped_saves++;
    }
  }
}

/* Forward decls: the read-fd helpers are defined below writeat_fd, but writeat_fd
 * must close a same-path read fd (and vice versa) to keep the read/write held fds
 * mutually exclusive on any one file. */
static bool rpath_is(const char *path);
static void read_fd_close(void);

/* True if the cached write fd is open for `path`. @noreq fd-cache helper. */
static bool wpath_is(const char *path) {
  if (s_wfd < 0)
    return false;
  for (uint32_t i = 0;; i++) {
    if (s_wpath[i] != path[i])
      return false;
    if (path[i] == '\0')
      return true;
  }
}

/* Flush + close the cached write fd (if any). @noreq fd-cache helper. */
static void writeat_fd_close(void) {
  if (s_wfd >= 0) {
    vfs_sync(s_wfd);
    vfs_close(s_wfd);
    s_wfd = -1;
    s_wpath[0] = '\0';
  }
}

/* Return the cached write fd for `path`, opening it (and closing any other cached
 * file) on a miss. <0 on open failure. @noreq fd-cache helper. */
static vfs_fd_t writeat_fd(const char *path) {
  if (wpath_is(path))
    return s_wfd;
  writeat_fd_close(); /* a different file was cached */
  /* Never hold a write AND read fd open on the SAME file (see read_fd). */
  if (rpath_is(path))
    read_fd_close();
  vfs_fd_t fd = vfs_open(path, VFS_O_RDWR | VFS_O_CREAT);
  if (fd < 0)
    return -1;
  s_wfd = fd;
  uint32_t i = 0;
  for (; i < FS_WRITEAT_PATH_MAX - 1u && path[i] != '\0'; i++)
    s_wpath[i] = path[i];
  s_wpath[i] = '\0';
  return s_wfd;
}

/* True if the cached read fd is open for `path`. @noreq fd-cache helper. */
static bool rpath_is(const char *path) {
  if (s_rfd < 0)
    return false;
  for (uint32_t i = 0;; i++) {
    if (s_rpath[i] != path[i])
      return false;
    if (path[i] == '\0')
      return true;
  }
}

/* Close the cached read fd (if any). @noreq fd-cache helper. */
static void read_fd_close(void) {
  if (s_rfd >= 0) {
    vfs_close(s_rfd);
    s_rfd = -1;
    s_rpath[0] = '\0';
    s_rpos = 0;
  }
}

/* Return the cached read fd for `path`, opening it RDONLY (and closing any other
 * cached read file) on a miss. <0 on open failure. @noreq fd-cache helper. */
static vfs_fd_t read_fd(const char *path) {
  if (rpath_is(path))
    return s_rfd;
  read_fd_close(); /* a different file was cached */
  /* Never hold a read AND write fd open on the SAME file: two FatFS FILs sharing
   * the volume's single FAT/dir window (FATFS.win) corrupt each other's cluster-
   * chain reads — the download-after-upload-of-same-path case. Flush + close the
   * writer so the reader sees one consistent FIL on the file. */
  if (wpath_is(path))
    writeat_fd_close();
  vfs_fd_t fd = vfs_open(path, VFS_O_RDONLY);
  if (fd < 0)
    return -1;
  s_rfd = fd;
  s_rpos = 0; /* fresh fd starts at BOF */
  uint32_t i = 0;
  for (; i < FS_WRITEAT_PATH_MAX - 1u && path[i] != '\0'; i++)
    s_rpath[i] = path[i];
  s_rpath[i] = '\0';
  return s_rfd;
}

/* Positioned write onto the lazily-held fd: seek, write, sync (the fd stays open
 * across chunks — see s_wfd). Writes at different offsets accumulate into one
 * file (an upload). @return true if the bytes were durably written.
 * @implements LOG-XFER-001 */
static bool fs_do_write_at(const fs_writeat_req_t *req) {
  vfs_fd_t fd = writeat_fd(req->path);
  if (fd < 0) {
    return false;
  }
  vfs_lseek(fd, (long)req->offset, VFS_SEEK_SET);
  int w = vfs_write(fd, req->payload, req->len);
  int sync_res = vfs_sync(fd);
  return (w == (int)req->len && sync_res >= 0);
}

/* Run one write-at, updating per-session bookkeeping. On failure the request is
 * re-queued onto the retry lane (up to FS_WRITEAT_MAX_TRIES); a permanent failure
 * (budget spent or the retry lane is full) marks the session failed so the xfer
 * upload reports FAILED to the GCS instead of hanging.
 * @implements LOG-XFER-001 */
static void fs_process_writeat(fs_writeat_req_t *req) {
  uint8_t sess = (req->session < FS_WA_SESSIONS) ? req->session : 0u;
  if (fs_do_write_at(req)) {
    s_wa_committed[sess] += req->len;
    if (s_wa_pending[sess] > 0u) {
      s_wa_pending[sess]--;
    }
    return;
  }
  req->tries++;
  if (req->tries < FS_WRITEAT_MAX_TRIES && s_retry_buf &&
      mpmc_try_push(&s_retry_q, req)) {
    return; /* still pending: it stays counted, awaiting another attempt */
  }
  /* Permanent failure: stop counting it as pending and flag the session. */
  s_wa_failed[sess] = 1u;
  if (s_wa_pending[sess] > 0u) {
    s_wa_pending[sess]--;
  }
  s_dropped_writeats++;
}

/* @implements LOG-XFER-001 */
static void fs_drain_writeats(void) {
  if (!s_writeat_buf) {
    return;
  }
  fs_writeat_req_t req;
  bool did = false;
  /* Fresh requests first. */
  while (mpmc_try_pop(&s_writeat_q, &req)) {
    fs_process_writeat(&req);
    did = true;
  }
  /* Then retry ONE failed write per call — "when no active req to write is there,
   * try again." One-per-call lets the task loop pace retries (no busy-spin while a
   * slot stays held by a concurrent reader); a re-failed write goes to the back. */
  if (s_retry_buf && mpmc_try_pop(&s_retry_q, &req)) {
    fs_process_writeat(&req);
    did = true;
  }
  /* Release the lazily-held write fd once the lane has been quiet, so no slot is
   * held across transfers (still never per-chunk open/close). */
  if (did) {
    s_wfd_idle_loops = 0;
  } else if (s_wfd >= 0 && ++s_wfd_idle_loops >= FS_WRITEAT_IDLE_CLOSE_LOOPS) {
    writeat_fd_close();
    s_wfd_idle_loops = 0;
  }
}

/* ===========================================================================
 * Synchronous READ/STAT/DIR/TRUNCATE routing (single-FS-task safety)
 *
 * The non-reentrant FatFS + the shared global v_fs descriptor table tolerate SD
 * access from only ONE task context. Uploads/saves/logs run on the FS task via
 * the async lanes above; reads/stat/dir/truncate must NOT call vfs_* directly on
 * the *producer's* task (xfer/comm) — two task contexts driving SDIO with context
 * switches between their ops corrupts FatFS (verified on hardware:
 * FR_INVALID_OBJECT under load). These ops are therefore funnelled to the FS task
 * too: the caller fills a request, hands it over, and blocks until the FS task
 * has executed the vfs_* call in its own context.
 *
 * Built on mpmc (not ipc semaphores) on purpose: the SITL `vayu_sitl_core` links
 * only kernel/structure.c, so referencing the v_semaphore / v_mutex API would
 * break the host link. Three cap-1 queues: a token serialises requesters, a request
 * queue hands work to the FS task, a result queue hands the return value back.
 * In SITL / at boot the FS task isn't running (s_task_mode == false) and every op
 * stays a direct vfs_* call — preserving the existing single-threaded behaviour.
 * =========================================================================== */
typedef enum {
  FS_SYNC_READ = 0,
  FS_SYNC_TRUNCATE,
  FS_SYNC_UNLINK,
  FS_SYNC_STAT,
  FS_SYNC_OPENDIR,
  FS_SYNC_READDIR,
  FS_SYNC_CLOSEDIR
} fs_sync_op_t;

typedef struct {
  uint8_t op;
  const char *path; /* caller-owned; valid while the caller blocks */
  uint32_t offset;
  void *buf; /* read dst / stat out / dirent out (caller-owned) */
  uint32_t len;
  vfs_dir_t dir; /* readdir/closedir handle */
} fs_sync_req_t;

typedef struct {
  int result;
} fs_sync_res_t;

#define FS_SYNC_TIMEOUT_TICKS 2000u /* generous; avoids a permanent hang */

static volatile bool s_task_mode = false; /* true only under the real FS task */
static mpmc_queue_t s_sync_lock_q; /* 1-token serialiser for requesters */
static mpmc_queue_t s_sync_req_q;  /* caller -> FS task */
static mpmc_queue_t s_sync_res_q;  /* FS task -> caller */
static uint8_t s_sync_lock_buf[1];
static fs_sync_req_t s_sync_req_buf[1];
static fs_sync_res_t s_sync_res_buf[1];

/* Execute one filesystem op directly (runs on the FS task via fs_owner_task,
 * OR inline on the caller in pump/boot mode).
 * @implements LOG-OWN-001 */
static int fs_exec_sync(const fs_sync_req_t *req) {
  switch ((fs_sync_op_t)req->op) {
  case FS_SYNC_READ: {
    /* Held-open read (see s_rfd): open once, then lseek + read per chunk. Opening
     * per chunk re-reads the directory and returns corrupt data on this SD. */
    vfs_fd_t fd = read_fd(req->path);
    if (fd < 0) {
      return -1;
    }
    s_rfd_idle_loops = 0;
    /* Seek only on an actual discontinuity (a download resume/rewind). A normal
     * sequential download reads contiguous offsets, so the held fd is already in
     * position — issuing a redundant lseek before every read is both wasteful and
     * has proven fragile on this FatFS/SD. */
    if (req->offset != s_rpos) {
      if (vfs_lseek(fd, (long)req->offset, VFS_SEEK_SET) < 0)
        return -1;
      s_rpos = req->offset;
    }
    int n = vfs_read(fd, req->buf, req->len);
    if (n > 0) {
      s_rpos += (uint32_t)n;
      return n;
    }
    /* Read shortfall/error. A FatFS FIL LATCHES a disk error — every later read on
     * the same handle then returns 0, permanently stalling a download. The error
     * is transient on this SD (a fresh handle re-reads the same offset fine), so
     * drop the handle here and let the SM retry next tick on a freshly reopened
     * fd. */
    read_fd_close();
    return n;
  }
  case FS_SYNC_UNLINK: {
    /* Same stale-handle problem as TRUNCATE, and worse here: unlinking a path
     * the lazy-write cache still holds open frees clusters a later flush would
     * write into. Drop both handles before the directory entry goes. */
    if (wpath_is(req->path)) {
      writeat_fd_close();
    }
    if (rpath_is(req->path)) {
      read_fd_close();
    }
    return (vfs_unlink(req->path) == 0) ? 0 : -1;
  }
  case FS_SYNC_TRUNCATE: {
    /* Drop a lingering lazy-write cache for this path so the truncate doesn't
     * race a stale held handle (s_wfd is FS-task-owned, so this is safe). */
    if (wpath_is(req->path)) {
      writeat_fd_close();
    }
    /* Same for a stale read handle on the file being replaced. */
    if (rpath_is(req->path)) {
      read_fd_close();
    }
    vfs_fd_t fd = vfs_open(req->path, VFS_O_WRONLY | VFS_O_CREAT | VFS_O_TRUNC);
    if (fd < 0) {
      return -1;
    }
    vfs_close(fd);
    return 0;
  }
  case FS_SYNC_STAT:
    return vfs_stat(req->path, (vfs_stat_t *)req->buf);
  case FS_SYNC_OPENDIR:
    return (int)vfs_opendir(req->path);
  case FS_SYNC_READDIR:
    return vfs_readdir(req->dir, (vfs_dirent_t *)req->buf);
  case FS_SYNC_CLOSEDIR:
    return vfs_closedir(req->dir);
  default:
    return -1;
  }
}

/* Caller side: hand `req` to the FS task and block for its result. Serialised so
 * only one cross-task FS request is in flight at a time. @noreq cross-task
 * request/response plumbing for the single-FS-owner funnel. */
static int fs_sync_call(const fs_sync_req_t *req) {
  uint8_t tok;
  if (!mpmc_pop_timeout(&s_sync_lock_q, &tok, FS_SYNC_TIMEOUT_TICKS)) {
    return -1; /* couldn't acquire the serialiser */
  }
  int rc = -1;
  if (mpmc_try_push(&s_sync_req_q, req)) {
    fs_sync_res_t res;
    if (mpmc_pop_timeout(&s_sync_res_q, &res, FS_SYNC_TIMEOUT_TICKS)) {
      rc = res.result;
    }
  }
  mpmc_try_push(&s_sync_lock_q, &tok); /* release */
  return rc;
}

/* ===========================================================================
 * Lifecycle
 * =========================================================================== */
/** @noreq queue/lane construction (lifecycle); idempotent. */
void fs_owner_init(void) {
  if (s_ready) {
    return;
  }
  mpmc_init(&s_log_q, s_log_buf, FS_LOG_QUEUE_CAP, sizeof(fs_log_req_t));
  mpmc_set_policy(&s_log_q, MPMC_POLICY_DROP);
  mpmc_init(&s_save_q, s_save_buf, FS_SAVE_QUEUE_CAP, sizeof(fs_save_req_t));
  mpmc_set_policy(&s_save_q, MPMC_POLICY_DROP);
  /* Write-at + retry lanes: heap-backed (keeps .bss flat — RAM budget). If the
   * alloc fails the lanes stay disabled; logs + saves still work. */
  if (!s_writeat_buf) {
    s_writeat_buf = (fs_writeat_req_t *)v_malloc(sizeof(fs_writeat_req_t) *
                                                 FS_WRITEAT_QUEUE_CAP);
  }
  if (!s_retry_buf) {
    s_retry_buf = (fs_writeat_req_t *)v_malloc(sizeof(fs_writeat_req_t) *
                                               FS_WRITEAT_RETRY_CAP);
  }
  if (s_writeat_buf) {
    mpmc_init(&s_writeat_q, s_writeat_buf, FS_WRITEAT_QUEUE_CAP,
              sizeof(fs_writeat_req_t));
    mpmc_set_policy(&s_writeat_q, MPMC_POLICY_DROP);
  }
  if (s_retry_buf) {
    mpmc_init(&s_retry_q, s_retry_buf, FS_WRITEAT_RETRY_CAP,
              sizeof(fs_writeat_req_t));
    mpmc_set_policy(&s_retry_q, MPMC_POLICY_DROP);
  }
  /* Cross-task sync-FS channel (reads/stat/dir/truncate). Seed the serialiser
   * with one token. Harmless in SITL — s_task_mode stays false so it's unused. */
  mpmc_init(&s_sync_lock_q, s_sync_lock_buf, 1u, sizeof(uint8_t));
  mpmc_set_policy(&s_sync_lock_q, MPMC_POLICY_DROP);
  mpmc_init(&s_sync_req_q, s_sync_req_buf, 1u, sizeof(fs_sync_req_t));
  mpmc_set_policy(&s_sync_req_q, MPMC_POLICY_DROP);
  mpmc_init(&s_sync_res_q, s_sync_res_buf, 1u, sizeof(fs_sync_res_t));
  mpmc_set_policy(&s_sync_res_q, MPMC_POLICY_DROP);
  uint8_t tok = 1u;
  mpmc_try_push(&s_sync_lock_q, &tok);
  s_ready = true;
}

/** @noreq test-only synchronous drain of all lanes in the caller's context. */
void fs_owner_pump(void) {
  if (!s_ready) {
    return;
  }
  fs_drain_saves();
  fs_drain_writeats();
  fs_log_req_t req;
  while (mpmc_try_pop(&s_log_q, &req)) {
    fs_do_log_req(&req);
  }
}

/** @implements LOG-OWN-001 */
void fs_owner_task(void *args) {
  (void)args;
  fs_owner_init();
  /* This task is the SOLE context that touches SD/VFS at runtime, so reads/
   * stat/dir/truncate from other tasks route here (see fs_sync_call). Must be set
   * only here — in SITL the FS task never runs and those ops stay direct. */
  s_task_mode = true;
  fs_log_req_t lreq;
  for (;;) {
    /* Reserved save lane drained fully first, then the upload write-at lane
     * (incl. one retry per loop); logs are best-effort and yield to both. */
    fs_drain_saves();
    fs_drain_writeats();
    /* Best-effort logs drain non-blocking (they no longer own the wait), UNLESS a
     * bulk transfer is active — then logging is quiesced so the transfer is the
     * sole multi-file SD actor (avoids the interleaved-multi-file corruption). */
    if (!s_logs_suppressed) {
      while (mpmc_try_pop(&s_log_q, &lreq)) {
        fs_do_log_req(&lreq);
      }
    }
    /* High-speed IMU stream: whole preallocated sectors, and the only SD writer
     * that must keep a steady 24 KB/s. Serviced here so SD/VFS stays
     * single-owner; it self-gates on the arm state. */
    imu_hs_log_drain();
    /* Block for a cross-task FS request (download reads etc.) so they're serviced
     * promptly; the timeout bounds save/write-at/retry/log latency when idle. */
    fs_sync_req_t sreq;
    if (mpmc_pop_timeout(&s_sync_req_q, &sreq, FS_POLL_TICKS)) {
      fs_sync_res_t res;
      res.result = fs_exec_sync(&sreq);
      mpmc_try_push(&s_sync_res_q, &res);
    }
    /* NOTE: the held read fd is deliberately NOT idle-closed here. A download is
     * flow-controlled to a few KiB/s — chunks arrive ~70ms apart, far slower than
     * any short idle window — so a timer-based close would fire BETWEEN chunks and
     * collapse the held-open back into the corrupting open-per-chunk pattern. The
     * fd is released instead on the next different-path read or a truncate, plus a
     * coarse age cap below as a backstop so a stale handle can't live forever. */
    if (s_rfd >= 0 && ++s_rfd_idle_loops >= FS_READ_FD_MAX_HOLD_LOOPS) {
      read_fd_close();
      s_rfd_idle_loops = 0;
    }
  }
}

/** @noreq trivial setter for the log-suppression flag. */
void fs_owner_suppress_logs(bool suppress) { s_logs_suppressed = suppress; }

/** @noreq trivial getter; the HS logger quiesces on the same flag. */
bool fs_owner_logs_suppressed(void) { return s_logs_suppressed; }

/* ===========================================================================
 * Producers — snapshot and return immediately.
 * =========================================================================== */
/** @noreq blackbox log producer; thin snapshot-and-enqueue onto the log lane. */
bool fs_owner_enqueue_log(logger_type_t type, const void *data, uint32_t len) {
  if (!s_ready || data == NULL || len == 0u || len > FS_LOG_PAYLOAD_MAX) {
    s_dropped_logs++;
    return false;
  }
  fs_log_req_t req;
  req.logger_type = (uint8_t)type;
  req.len = (uint16_t)len;
  v_memcpy(req.payload, data, len);
  if (!mpmc_try_push(&s_log_q, &req)) {
    s_dropped_logs++;
    return false;
  }
  return true;
}

/** @implements LOG-PERSIST-001 */
bool fs_owner_enqueue_pid_save(const void *store, uint32_t len) {
  if (!s_ready || store == NULL || len == 0u || len > FS_SAVE_PAYLOAD_MAX) {
    s_dropped_saves++;
    return false;
  }
  fs_save_req_t req;
  req.type = (uint8_t)FS_SAVE_PID;
  req.tries = 0;
  req.len = (uint16_t)len;
  v_memcpy(req.payload, store, len);
  if (!mpmc_try_push(&s_save_q, &req)) {
    s_dropped_saves++;
    return false;
  }
  return true;
}

/** @implements LOG-PERSIST-001 */
bool fs_owner_enqueue_calib_save(const void *header, uint32_t hlen,
                                 const void *payload, uint32_t plen) {
  uint32_t total = hlen + plen;
  if (!s_ready || header == NULL || payload == NULL || total == 0u ||
      total > FS_SAVE_PAYLOAD_MAX) {
    s_dropped_saves++;
    return false;
  }
  fs_save_req_t req;
  req.type = (uint8_t)FS_SAVE_CALIB;
  req.tries = 0;
  req.len = (uint16_t)total;
  v_memcpy(req.payload, header, hlen);
  v_memcpy(req.payload + hlen, payload, plen);
  if (!mpmc_try_push(&s_save_q, &req)) {
    s_dropped_saves++;
    return false;
  }
  return true;
}

/** @implements LOG-XFER-001 */
bool fs_owner_enqueue_write_at(uint8_t session, const char *path,
                               uint32_t offset, const void *data,
                               uint32_t len) {
  if (!s_ready || s_writeat_buf == NULL || path == NULL || data == NULL ||
      len == 0u || len > FS_WRITEAT_PAYLOAD_MAX || session >= FS_WA_SESSIONS) {
    s_dropped_writeats++;
    return false;
  }
  fs_writeat_req_t req;
  /* Bounded copy of the path (must fit with a NUL). */
  uint32_t i = 0;
  for (; i < FS_WRITEAT_PATH_MAX - 1u && path[i] != '\0'; i++) {
    req.path[i] = path[i];
  }
  if (path[i] != '\0') { /* path too long to store safely */
    s_dropped_writeats++;
    return false;
  }
  req.path[i] = '\0';
  req.offset = offset;
  req.len = (uint16_t)len;
  req.session = session;
  req.tries = 0;
  v_memcpy(req.payload, data, len);
  if (!mpmc_try_push(&s_writeat_q, &req)) {
    s_dropped_writeats++; /* full lane => backpressure (cursor holds, GCS resends) */
    return false;
  }
  s_wa_pending[session]++;
  return true;
}

/** @noreq trivial reset of a session's write-at bookkeeping counters. */
void fs_owner_writeat_reset(uint8_t session) {
  if (session >= FS_WA_SESSIONS) {
    return;
  }
  s_wa_pending[session] = 0;
  s_wa_committed[session] = 0;
  s_wa_failed[session] = 0;
}

/** @noreq trivial write-at status accessor. */
uint32_t fs_owner_writeat_pending(uint8_t session) {
  return (session < FS_WA_SESSIONS) ? s_wa_pending[session] : 0u;
}

/** @noreq trivial write-at status accessor. */
uint32_t fs_owner_writeat_committed(uint8_t session) {
  return (session < FS_WA_SESSIONS) ? s_wa_committed[session] : 0u;
}

/** @noreq trivial write-at status accessor. */
bool fs_owner_writeat_failed(uint8_t session) {
  return (session < FS_WA_SESSIONS) ? (s_wa_failed[session] != 0u) : false;
}

/* The read/stat/dir/truncate entry points below all funnel through the FS task
 * (fs_sync_call) once it is running, so SD/VFS is touched from exactly one task
 * context. Before the task starts and in SITL (s_task_mode == false) they execute
 * the op inline (direct vfs_*). */

/** @implements LOG-XFER-002 */
int fs_owner_truncate(const char *path) {
  if (path == NULL) {
    return -1;
  }
  fs_sync_req_t req = {.op = FS_SYNC_TRUNCATE, .path = path};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/** @implements LOG-XFER-002 */
int fs_owner_unlink(const char *path) {
  if (path == NULL) {
    return -1;
  }
  fs_sync_req_t req = {.op = FS_SYNC_UNLINK, .path = path};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/** @implements LOG-XFER-002 */
int fs_owner_read_at(const char *path, uint32_t offset, void *buf,
                     uint32_t len) {
  if (path == NULL || buf == NULL || len == 0u) {
    return -1;
  }
  fs_sync_req_t req = {.op = FS_SYNC_READ,
                       .path = path,
                       .offset = offset,
                       .buf = buf,
                       .len = len};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/** @implements LOG-FS-001 */
int fs_owner_stat(const char *path, vfs_stat_t *st) {
  fs_sync_req_t req = {.op = FS_SYNC_STAT, .path = path, .buf = st};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/** @implements LOG-FS-001 */
vfs_dir_t fs_owner_opendir(const char *path) {
  fs_sync_req_t req = {.op = FS_SYNC_OPENDIR, .path = path};
  return (vfs_dir_t)(s_task_mode ? fs_sync_call(&req) : fs_exec_sync(&req));
}

/** @implements LOG-FS-001 */
int fs_owner_readdir(vfs_dir_t d, vfs_dirent_t *ent) {
  fs_sync_req_t req = {.op = FS_SYNC_READDIR, .dir = d, .buf = ent};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/** @implements LOG-FS-001 */
int fs_owner_closedir(vfs_dir_t d) {
  fs_sync_req_t req = {.op = FS_SYNC_CLOSEDIR, .dir = d};
  if (s_task_mode) {
    return fs_sync_call(&req);
  }
  return fs_exec_sync(&req);
}

/* ===========================================================================
 * Accounting
 * =========================================================================== */
/** @implements LOG-SD-002 — per-log wrap counter surfaced to HEALTH status. */
uint32_t fs_owner_log_wrap_count(logger_type_t type) {
  switch (type) {
  case NAVLINK_LOGGER:
    return navlink_wrap_count;
  case SYSTEM_LOGGER:
    return system_wrap_count;
  case GENERAL_LOGGER:
  default:
    return general_wrap_count;
  }
}

/** @implements LOG-SD-002 — aggregate wrap counter across the 3 ring logs. */
uint32_t fs_owner_log_wrap_count_total(void) {
  return navlink_wrap_count + system_wrap_count + general_wrap_count;
}

/** @noreq drop-count accessor. */
uint32_t fs_owner_dropped_logs(void) { return s_dropped_logs; }
/** @noreq drop-count accessor. */
uint32_t fs_owner_dropped_saves(void) { return s_dropped_saves; }
/** @noreq drop-count accessor. */
uint32_t fs_owner_dropped_writeats(void) { return s_dropped_writeats; }
