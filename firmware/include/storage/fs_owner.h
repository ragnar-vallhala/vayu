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
#ifndef VAYU_STORAGE_FS_OWNER_H
#define VAYU_STORAGE_FS_OWNER_H

/**
 * @file include/storage/fs_owner.h
 * @brief Centralised filesystem owner — the sole runtime owner of all SD/VFS
 *        writes, plus the logging declarations.
 *
 * Fixes the C1->C3 contention chain (firmware/docs/scratch/resource-ownership-map.md):
 * PID/calib saves and blackbox log writes must NOT run synchronously in the
 * caller's task context under the global vfs_mutex — a save in comm_processor_task
 * would stall the UART6 RX-ring drain and silently drop GCS commands. Instead a
 * single low-priority task drains a queue and performs vfs_open/write/sync/close
 * off the critical path; producers snapshot their payload and return immediately.
 *
 * Two lanes (the "reserved save slots" decision): a high-volume log lane that
 * drops-and-counts, and a small dedicated save lane that logging can never
 * starve. The FS task drains saves first.
 *
 * This header also declares the text-log producer (vayu_log, impl in
 * src/logger/log_text.c — streams over telemetry, never SD) and the binary
 * blackbox API (the 3 circular SD files).
 */

#include "structure.h" /* mpmc_queue_t (for vayu_log_queue) */
#include "vfs.h"       /* vfs_stat_t, vfs_dir_t, vfs_dirent_t */
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
 * @brief Quiesce best-effort blackbox logging while a bulk transfer runs.
 *
 * When @p suppress is true the FS task stops draining the log lane to SD, so an
 * in-flight xfer is the sole multi-file SD actor (interleaving log writes with
 * the transfer's I/O corrupts the FatFS/SD read-back). Driven by the xfer
 * service task from xfer_active(). Queued records drop (circular blackbox) until
 * the transfer ends. Safe to call from any task (single volatile flag).
 */
void fs_owner_suppress_logs(bool suppress);

/** @brief Is best-effort SD logging currently quiesced for a bulk transfer? */
bool fs_owner_logs_suppressed(void);

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
 * fs_owner_init (firmware/docs/plans/xfer-memory-budget.md), so .bss stays flat.
 *
 * A write whose vfs_open/write fails (e.g. all 4 FatFS slots momentarily busy)
 * is NOT lost: fs_do_write_at re-queues it onto a separate retry lane that the
 * task drains, again and again, whenever no fresh write-at is pending — until it
 * succeeds or a bounded retry budget is exhausted (then the session is marked
 * failed). Per-session bookkeeping (pending / committed / failed) lets the xfer
 * upload report true persistence to the GCS (see fs_owner_writeat_* below).
 *
 * `session` tags the write to an xfer session (0..XFER_MAX_SESSIONS-1) for that
 * bookkeeping. Firmware-internal writers MUST pass FS_WA_SESSION_INTERNAL
 * instead of borrowing 0 — sharing a slot with a live upload corrupts the
 * per-session counters its flow control depends on.
 *
 * @return true if queued; false if the main lane is full (backpressure: the
 *         cursor stalls and the next XFER_ACK tells the GCS to pause/retransmit)
 *         or the request is malformed (path too long, len out of range, not
 *         ready) — drop-and-counted.
 */
/* Reserved write-at slot for firmware-internal saves (never an xfer session),
 * so their bookkeeping cannot collide with a GCS upload in flight. */
#define FS_WA_SESSION_INTERNAL 2u

bool fs_owner_enqueue_write_at(uint8_t session, const char *path,
                               uint32_t offset, const void *data, uint32_t len);

/* ---- per-session write-at status (the xfer upload's confirmed-write report) --
 * fs_owner_writeat_reset() zeroes a session's counters at the start of an upload.
 * pending()  = chunks queued or retrying but not yet durably written.
 * committed()= bytes successfully written (== upload size when fully flushed).
 * failed()   = a write exhausted its retry budget (permanent failure).
 * The xfer FILE provider polls these in its flush() hook: pending==0 && !failed
 * => DONE; failed => FAILED. */
void fs_owner_writeat_reset(uint8_t session);
uint32_t fs_owner_writeat_pending(uint8_t session);
uint32_t fs_owner_writeat_committed(uint8_t session);
bool fs_owner_writeat_failed(uint8_t session);

/**
 * @brief Truncate-or-create a file to empty. **Called ONLY from
 *        xfer_service_task** (a fresh upload at offset 0 replacing a file).
 *        Same vfs_mutex-serialised safety as fs_owner_read_at. Returns 0 on
 *        success, <0 on open failure.
 */
int fs_owner_truncate(const char *path);

/**
 * @brief Delete a file. **Called ONLY from xfer_service_task** (FS_DELETE).
 *        Same vfs_mutex-serialised safety as fs_owner_truncate, and it drops
 *        any cached write/read handle on the path first so no later flush can
 *        write into clusters the unlink has freed. Returns 0 on success, <0 if
 *        the path is absent or the unlink failed. Enforces no policy: which
 *        paths may be deleted is fs_query's business, not the owner's.
 */
int fs_owner_unlink(const char *path);

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
 * Directory browse + file status (the filesystem-navigation feature).
 * Synchronous, **xfer_service_task only** — same vfs_mutex-serialised safety as
 * fs_owner_read_at (these are reads). Thin pass-throughs to the VFS so the SD
 * stays single-transaction with no new locks.
 * =========================================================================== */

/** @brief Status of a path. Returns 0 if it exists (st->exists=1), <0 if not
 *         (st->exists=0) — lets callers report "path does not exist" distinctly. */
int fs_owner_stat(const char *path, vfs_stat_t *st);

/** @brief Open a directory for iteration. >=0 handle, or <0 if the path is not a
 *         directory / does not exist. */
vfs_dir_t fs_owner_opendir(const char *path);

/** @brief Next entry: 1 = entry filled, 0 = end of directory, <0 = error. */
int fs_owner_readdir(vfs_dir_t d, vfs_dirent_t *ent);

/** @brief Close a directory handle. */
int fs_owner_closedir(vfs_dir_t d);

/* ===========================================================================
 * Accounting.
 * =========================================================================== */

/* Wrap accounting (LOG-SD-002): incremented each time a log's circular write
 * position wraps, i.e. the oldest records are overwritten — so the loss is
 * accountable rather than silent. */
uint32_t fs_owner_log_wrap_count(logger_type_t type);
uint32_t fs_owner_log_wrap_count_total(void);

/* Drop accounting (mirrors COMM-CH-002): requests lost to a full lane. */
uint32_t fs_owner_dropped_logs(void);
uint32_t fs_owner_dropped_saves(void);
uint32_t fs_owner_dropped_writeats(void);

#endif /* VAYU_STORAGE_FS_OWNER_H */
