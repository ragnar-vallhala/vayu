/**
 * @file src/storage/fs_owner.c
 * @brief Centralised filesystem owner — sole runtime owner of all SD/VFS writes.
 *
 * Absorbs the former binary blackbox logger (boot prealloc/open of the 3
 * circular files, the fds, write positions, wrap counters and the circular
 * wrap/seek/write/sync) and serialises PID/calib persistence behind a queue,
 * so the blocking SD I/O no longer runs in the producer's task context. See
 * docs/plans/centralised-fs-owner.md and docs/scratch/resource-ownership-map.md
 * (the C1->C3 chain this fixes).
 *
 * Two lanes: a high-volume log lane (drop-and-count) and a small reserved save
 * lane that logging can never starve. The task drains saves first.
 */
#include "storage/fs_owner.h"

#include "memory.h"               /* v_malloc (heap-backed write-at lane) */
#include "utils.h"                /* v_memcpy */
#include "vaios_config_default.h" /* PANIC */
#include "variables.h" /* *_LOGGING_FILENAME/_FILE_SIZE, CALIBRATION_FILE_PATH */
#include "vfs.h"

/* ===========================================================================
 * Sizing
 * =========================================================================== */
#define FS_LOG_PAYLOAD_MAX 256u  /* >= max navlink blackbox record           */
#define FS_SAVE_PAYLOAD_MAX 160u /* pid_store ~140B; calib hdr(8)+payload(84) */
/* Log lane depth. KEEP SMALL: each slot is FS_LOG_PAYLOAD_MAX+4 bytes of static
 * BSS, and the STM32F401 (96 KiB SRAM) is RAM-starved — a too-large queue pushes
 * _heap_start up until the kernel heap's HEAP_SIZE memset runs off the top of RAM
 * (silent: the linker can't see it), corrupting memory at boot -> HardFault. 4 is
 * ample for the (currently unused) blackbox path; do NOT raise without checking
 * _heap_start + HEAP_SIZE <= top-of-RAM on the real build. */
#define FS_LOG_QUEUE_CAP 4u      /* ~1 KiB. Was 32 (8.3 KiB) — overflowed F401 SRAM. */
#define FS_SAVE_QUEUE_CAP 4u     /* reserved — logs can never occupy this lane */
#define FS_POLL_TICKS 5u         /* save-lane latency bound while blocked on logs */

/* Upload write-at lane (xfer substrate). Each slot ~296 B; the backing buffer is
 * HEAP-allocated (v_malloc) not static, so adding this lane keeps .bss flat and
 * doesn't shrink the F401 MSP margin (docs/plans/xfer-memory-budget.md). CAP=2
 * gives one chunk of pipelining; CAP=1 reclaims a slot if the heap ever tightens. */
#define FS_WRITEAT_PAYLOAD_MAX 247u /* = XFER_CHUNK_MAX (one XFER_DATA chunk)     */
#define FS_WRITEAT_PATH_MAX 40u     /* >= XFER_ARG_MAX(32) SD path                */
#define FS_WRITEAT_QUEUE_CAP 2u

#define FS_PID_FILE_PATH "0:pid.bin" /* mirrors pid_config.c boot reader */

typedef enum { FS_SAVE_PID = 0, FS_SAVE_CALIB } fs_save_type_t;

typedef struct {
  uint8_t logger_type; /* logger_type_t */
  uint16_t len;
  uint8_t payload[FS_LOG_PAYLOAD_MAX];
} fs_log_req_t;

typedef struct {
  uint8_t type; /* fs_save_type_t */
  uint16_t len; /* bytes used (calib = hdr+payload concatenated) */
  uint8_t payload[FS_SAVE_PAYLOAD_MAX];
} fs_save_req_t;

typedef struct {
  char path[FS_WRITEAT_PATH_MAX];
  uint32_t offset;
  uint16_t len;
  uint8_t payload[FS_WRITEAT_PAYLOAD_MAX];
} fs_writeat_req_t;

/* ===========================================================================
 * State
 * =========================================================================== */
static mpmc_queue_t s_log_q;
static mpmc_queue_t s_save_q;
static mpmc_queue_t s_writeat_q;
static fs_log_req_t s_log_buf[FS_LOG_QUEUE_CAP];
static fs_save_req_t s_save_buf[FS_SAVE_QUEUE_CAP];
static fs_writeat_req_t *s_writeat_buf; /* heap (v_malloc in fs_owner_init) */
static volatile bool s_ready = false;

/* Blackbox file state (moved out of logger.c; single writer = the FS task). */
static volatile vfs_fd_t navlink_logger_fd = -1;
static volatile vfs_fd_t system_logger_fd = -1;
static volatile vfs_fd_t general_logger_fd = -1;
static uint32_t navlink_write_pos = 0;
static uint32_t system_write_pos = 0;
static uint32_t general_write_pos = 0;
static volatile uint32_t navlink_wrap_count = 0;
static volatile uint32_t system_wrap_count = 0;
static volatile uint32_t general_wrap_count = 0;

static volatile uint32_t s_dropped_logs = 0;
static volatile uint32_t s_dropped_saves = 0;
static volatile uint32_t s_dropped_writeats = 0;

/* ===========================================================================
 * Boot-time file setup (formerly logger_init) — direct vfs_*, scheduler off.
 * =========================================================================== */
static void ensure_file_size(vfs_fd_t fd, uint32_t file_size) {
  long current_size = vfs_lseek(fd, 0, VFS_SEEK_END);
  if (current_size < (long)file_size) {
    vfs_lseek(fd, (long)(file_size - 1u), VFS_SEEK_SET);
    uint8_t dummy = 0;
    vfs_write(fd, &dummy, 1);
    vfs_sync(fd);
  }
  vfs_lseek(fd, 0, VFS_SEEK_SET);
}

void fs_owner_boot_init(void) {
  if (vfs_preallocate(NAVLINK_LOGGING_FILENAME, NAVLINK_LOGGING_FILE_SIZE) != 0) {
    PANIC("Navlink prealloc failed");
  }
  if (vfs_preallocate(SYS_LOGGING_FILENAME, SYS_LOGGING_FILE_SIZE) != 0) {
    PANIC("System prealloc failed");
  }
  if (vfs_preallocate(GENERAL_LOGGING_FILENAME, GENERAL_LOGGING_FILE_SIZE) != 0) {
    PANIC("General prealloc failed");
  }

  navlink_logger_fd = vfs_open(NAVLINK_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);
  system_logger_fd = vfs_open(SYS_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);
  general_logger_fd = vfs_open(GENERAL_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);

  if (navlink_logger_fd < 0 || system_logger_fd < 0 || general_logger_fd < 0) {
    PANIC("Logger open failed");
  }

  ensure_file_size(navlink_logger_fd, NAVLINK_LOGGING_FILE_SIZE);
  ensure_file_size(system_logger_fd, SYS_LOGGING_FILE_SIZE);
  ensure_file_size(general_logger_fd, GENERAL_LOGGING_FILE_SIZE);

  navlink_write_pos = 0;
  system_write_pos = 0;
  general_write_pos = 0;
}

/* ===========================================================================
 * Work handlers (run by the FS task / the test pump — never the producer).
 * =========================================================================== */

/* Circular blackbox write. LOG-SD-002: wrapping discards the oldest records,
 * counted so the loss is accountable rather than silent. No mutex — single
 * consumer. */
static void fs_circular_write(volatile vfs_fd_t *fdp, uint32_t *write_pos,
                              volatile uint32_t *wrap_count, uint32_t file_size,
                              const uint8_t *data, uint32_t len) {
  vfs_fd_t fd = *fdp;
  if (fd < 0 || data == NULL || len == 0u || len > file_size) {
    return;
  }
  if (*write_pos > (file_size - len)) {
    *write_pos = 0;
    (*wrap_count)++;
  }
  vfs_lseek(fd, (long)(*write_pos), VFS_SEEK_SET);
  int res = vfs_write(fd, data, len);
  int sync_res = vfs_sync(fd);
  if (res >= 0 && sync_res >= 0) {
    *write_pos = (*write_pos + len) % (file_size - len);
  }
}

static void fs_do_log_req(const fs_log_req_t *req) {
  switch ((logger_type_t)req->logger_type) {
  case NAVLINK_LOGGER:
    fs_circular_write(&navlink_logger_fd, &navlink_write_pos, &navlink_wrap_count,
                      NAVLINK_LOGGING_FILE_SIZE, req->payload, req->len);
    break;
  case SYSTEM_LOGGER:
    fs_circular_write(&system_logger_fd, &system_write_pos, &system_wrap_count,
                      SYS_LOGGING_FILE_SIZE, req->payload, req->len);
    break;
  case GENERAL_LOGGER:
  default:
    fs_circular_write(&general_logger_fd, &general_write_pos, &general_wrap_count,
                      GENERAL_LOGGING_FILE_SIZE, req->payload, req->len);
    break;
  }
}

static void fs_do_save(const fs_save_req_t *req) {
  const char *path = ((fs_save_type_t)req->type == FS_SAVE_PID)
                         ? FS_PID_FILE_PATH
                         : CALIBRATION_FILE_PATH;
  vfs_fd_t fd = vfs_open(path, VFS_O_WRONLY | VFS_O_CREAT | VFS_O_TRUNC);
  if (fd < 0) {
    vayu_log("[FSOWN] open %s for write failed", path);
    return;
  }
  vfs_write(fd, req->payload, req->len);
  vfs_sync(fd);
  vfs_close(fd);
}

static void fs_drain_saves(void) {
  fs_save_req_t req;
  while (mpmc_try_pop(&s_save_q, &req)) {
    fs_do_save(&req);
  }
}

/* Positioned write: open-or-create, seek, write, sync, close. Open with RDWR (not
 * TRUNC) so writes at different offsets accumulate into one file (an upload). */
static void fs_do_write_at(const fs_writeat_req_t *req) {
  vfs_fd_t fd = vfs_open(req->path, VFS_O_RDWR | VFS_O_CREAT);
  if (fd < 0) {
    vayu_log("[FSOWN] write-at open %s failed", req->path);
    return;
  }
  vfs_lseek(fd, (long)req->offset, VFS_SEEK_SET);
  vfs_write(fd, req->payload, req->len);
  vfs_sync(fd);
  vfs_close(fd);
}

static void fs_drain_writeats(void) {
  if (!s_writeat_buf) {
    return;
  }
  fs_writeat_req_t req;
  while (mpmc_try_pop(&s_writeat_q, &req)) {
    fs_do_write_at(&req);
  }
}

/* ===========================================================================
 * Lifecycle
 * =========================================================================== */
void fs_owner_init(void) {
  if (s_ready) {
    return;
  }
  mpmc_init(&s_log_q, s_log_buf, FS_LOG_QUEUE_CAP, sizeof(fs_log_req_t));
  mpmc_set_policy(&s_log_q, MPMC_POLICY_DROP);
  mpmc_init(&s_save_q, s_save_buf, FS_SAVE_QUEUE_CAP, sizeof(fs_save_req_t));
  mpmc_set_policy(&s_save_q, MPMC_POLICY_DROP);
  /* Write-at lane: heap-backed (keeps .bss flat — RAM budget). If the alloc
   * fails the lane stays disabled; logs + saves still work. */
  if (!s_writeat_buf) {
    s_writeat_buf =
        (fs_writeat_req_t *)v_malloc(sizeof(fs_writeat_req_t) * FS_WRITEAT_QUEUE_CAP);
  }
  if (s_writeat_buf) {
    mpmc_init(&s_writeat_q, s_writeat_buf, FS_WRITEAT_QUEUE_CAP,
              sizeof(fs_writeat_req_t));
    mpmc_set_policy(&s_writeat_q, MPMC_POLICY_DROP);
  }
  s_ready = true;
}

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

void fs_owner_task(void *args) {
  (void)args;
  fs_owner_init();
  fs_log_req_t lreq;
  for (;;) {
    /* Reserved save lane drained fully first, then the upload write-at lane;
     * logs are best-effort and yield to both. */
    fs_drain_saves();
    fs_drain_writeats();
    /* Then block briefly for a log; the timeout bounds save/write-at latency. */
    if (mpmc_pop_timeout(&s_log_q, &lreq, FS_POLL_TICKS)) {
      fs_do_log_req(&lreq);
    }
  }
}

/* ===========================================================================
 * Producers — snapshot and return immediately.
 * =========================================================================== */
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

bool fs_owner_enqueue_pid_save(const void *store, uint32_t len) {
  if (!s_ready || store == NULL || len == 0u || len > FS_SAVE_PAYLOAD_MAX) {
    s_dropped_saves++;
    return false;
  }
  fs_save_req_t req;
  req.type = (uint8_t)FS_SAVE_PID;
  req.len = (uint16_t)len;
  v_memcpy(req.payload, store, len);
  if (!mpmc_try_push(&s_save_q, &req)) {
    s_dropped_saves++;
    return false;
  }
  return true;
}

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
  req.len = (uint16_t)total;
  v_memcpy(req.payload, header, hlen);
  v_memcpy(req.payload + hlen, payload, plen);
  if (!mpmc_try_push(&s_save_q, &req)) {
    s_dropped_saves++;
    return false;
  }
  return true;
}

bool fs_owner_enqueue_write_at(const char *path, uint32_t offset,
                               const void *data, uint32_t len) {
  if (!s_ready || s_writeat_buf == NULL || path == NULL || data == NULL ||
      len == 0u || len > FS_WRITEAT_PAYLOAD_MAX) {
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
  v_memcpy(req.payload, data, len);
  if (!mpmc_try_push(&s_writeat_q, &req)) {
    s_dropped_writeats++;
    return false;
  }
  return true;
}

int fs_owner_read_at(const char *path, uint32_t offset, void *buf,
                     uint32_t len) {
  if (path == NULL || buf == NULL || len == 0u) {
    return -1;
  }
  vfs_fd_t fd = vfs_open(path, VFS_O_RDONLY);
  if (fd < 0) {
    return -1;
  }
  vfs_lseek(fd, (long)offset, VFS_SEEK_SET);
  int n = vfs_read(fd, buf, len);
  vfs_close(fd);
  return n;
}

/* ===========================================================================
 * Accounting
 * =========================================================================== */
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

uint32_t fs_owner_log_wrap_count_total(void) {
  return navlink_wrap_count + system_wrap_count + general_wrap_count;
}

uint32_t fs_owner_dropped_logs(void) { return s_dropped_logs; }
uint32_t fs_owner_dropped_saves(void) { return s_dropped_saves; }
uint32_t fs_owner_dropped_writeats(void) { return s_dropped_writeats; }
