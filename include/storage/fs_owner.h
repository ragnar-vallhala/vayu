#ifndef VAYU_STORAGE_FS_OWNER_H
#define VAYU_STORAGE_FS_OWNER_H

/**
 * @file include/storage/fs_owner.h
 * @brief Centralised filesystem owner — the sole runtime owner of all SD/VFS
 *        writes, plus the logging declarations that used to live in logger.h.
 *
 * Fixes the C1->C3 contention chain (docs/scratch/resource-ownership-map.md):
 * PID/calib saves and blackbox log writes used to run synchronously in the
 * caller's task context under the global vfs_mutex. A save in comm_processor_task
 * would stall the UART6 RX-ring drain and silently drop GCS commands. Here a
 * single low-priority task drains a queue and performs vfs_open/write/sync/close
 * off the critical path; producers snapshot their payload and return immediately.
 *
 * Two lanes (the "reserved save slots" decision): a high-volume log lane that
 * drops-and-counts, and a small dedicated save lane that logging can never
 * starve. The FS task drains saves first.
 *
 * This header absorbs the former logger.h verbatim: the text-log producer
 * (vayu_log, impl in src/logger/log_text.c — streams over telemetry, never SD)
 * and the binary blackbox API (the 3 circular SD files).
 */

#include "structure.h" /* mpmc_queue_t (for vayu_log_queue) */
#include <stdbool.h>
#include <stdint.h>

/* ===========================================================================
 * Text log (impl in src/logger/log_text.c) — moved verbatim from logger.h.
 * The telemetry task drains vayu_log_queue to the LOG channel (LOG-TXT-002).
 * This path never touches the filesystem; it lives here only so there is a
 * single logging/storage header. @implements LOG-TXT-001
 * =========================================================================== */
extern mpmc_queue_t vayu_log_queue;
#define VAYU_LOG_QUEUE_SIZE 128
void vayu_log(const char *fmt, ...);

/* ===========================================================================
 * Binary blackbox: the 3 circular SD files.
 * =========================================================================== */
typedef enum {
  NAVLINK_LOGGER,
  SYSTEM_LOGGER,
  GENERAL_LOGGER,
} logger_type_t;

/* ===========================================================================
 * FS owner lifecycle.
 * =========================================================================== */

/**
 * @brief Boot-time setup: preallocate + open the 3 circular blackbox files.
 *        Replaces the former logger_init(). Runs from main() BEFORE
 *        scheduler_start() — single-threaded, direct vfs_* (no queue yet).
 */
void fs_owner_boot_init(void);

/**
 * @brief Create the two request queues. Idempotent. Called as the first line
 *        of fs_owner_task() (lazy, because mpmc_init creates a mutex +
 *        semaphores and needs the scheduler running). Tests may call it
 *        directly before enqueuing.
 */
void fs_owner_init(void);

/** @brief The storage task entry (also declared in vayu_tasks.h). */
void fs_owner_task(void *args);

/**
 * @brief Drain everything currently queued, synchronously, in the caller's
 *        context (saves first, then all pending logs). Used by tests to run
 *        the same handlers the task runs, deterministically and without
 *        spawning the task. Safe (but pointless) in production.
 */
void fs_owner_pump(void);

/* ===========================================================================
 * Producers — all snapshot (memcpy) their payload into the queue and return
 * immediately. Return true on enqueue, false if dropped (queue full / not
 * ready). Persistence is best-effort and asynchronous; the live apply at the
 * call site is authoritative.
 * =========================================================================== */

/** @brief Enqueue a blackbox log record (was logger_write). */
bool fs_owner_enqueue_log(logger_type_t type, const void *data, uint32_t len);

/** @brief Enqueue a snapshot of the PID store for persistence to 0:pid.bin. */
bool fs_owner_enqueue_pid_save(const void *store, uint32_t len);

/** @brief Enqueue a snapshot of the calib header+payload for 0:cal.bin. */
bool fs_owner_enqueue_calib_save(const void *header, uint32_t hlen,
                                 const void *payload, uint32_t plen);

/* ===========================================================================
 * Positioned write/read for the xfer (bulk-transfer) substrate.
 * =========================================================================== */

/**
 * @brief Enqueue a positioned write (open-or-create the path, seek to offset,
 *        write len bytes, sync) for asynchronous execution by the FS task.
 *
 * A dedicated THIRD lane, drained AFTER the reserved save lane but BEFORE the
 * log lane, so file uploads can never starve PID/calib saves yet still take
 * priority over best-effort blackbox logging. Snapshots the payload like the
 * other producers (the caller — an xfer upload handler on the comm task — may
 * reuse its buffer immediately). Backing store is heap-allocated in
 * fs_owner_init (docs/plans/xfer-memory-budget.md), so .bss stays flat.
 *
 * @return true if queued; false if dropped (path too long, len out of range,
 *         lane full, or not ready) — drop-and-counted like the other lanes. A
 *         full lane is the upload's backpressure signal (the cursor stalls and
 *         the next XFER_ACK tells the GCS to pause/retransmit).
 */
bool fs_owner_enqueue_write_at(const char *path, uint32_t offset,
                               const void *data, uint32_t len);

/**
 * @brief Truncate-or-create a file to empty. **Called ONLY from
 *        xfer_service_task** (a fresh upload at offset 0 replacing a file).
 *        Same vfs_mutex-serialised safety as fs_owner_read_at. Returns 0 on
 *        success, <0 on open failure.
 */
int fs_owner_truncate(const char *path);

/**
 * @brief Synchronous positioned read. **Called ONLY from xfer_service_task.**
 *
 * fs_owner is the sole *writer* of the SD; this is the one sanctioned reader.
 * Safe because the kernel vfs_* ops already take the global vfs_mutex
 * (extern/vaios/kernel/vfs.c), so this read serialises against fs_owner_task's
 * writes — the SD stays one-transaction-at-a-time with no new locks.
 *
 * @return bytes read (>=0), or <0 on open/seek/read error.
 */
int fs_owner_read_at(const char *path, uint32_t offset, void *buf,
                     uint32_t len);

/* ===========================================================================
 * Accounting.
 * =========================================================================== */

/* Wrap accounting (LOG-SD-002), moved here from logger.c: incremented each
 * time a log's circular write position wraps, i.e. the oldest records are
 * overwritten — so the loss is accountable rather than silent. */
uint32_t fs_owner_log_wrap_count(logger_type_t type);
uint32_t fs_owner_log_wrap_count_total(void);

/* Drop accounting (mirrors COMM-CH-002): requests lost to a full lane. */
uint32_t fs_owner_dropped_logs(void);
uint32_t fs_owner_dropped_saves(void);
uint32_t fs_owner_dropped_writeats(void);

#endif /* VAYU_STORAGE_FS_OWNER_H */
