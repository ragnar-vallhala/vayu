/**
 * @file src/comm/xfer/providers/file_provider.c
 * @brief FILE xfer provider — generic read/write of any SD path (up + down).
 *
 * Backed entirely by the fs_owner SD gateway: downloads read via
 * fs_owner_read_at (sync, xfer-task-only, vfs_mutex-serialised); uploads enqueue
 * via fs_owner_enqueue_write_at (async write-at lane). open() runs on the xfer
 * task, so the one direct vfs call (sizing a download) is off the comm task. The
 * file path is the XFER_OPEN `arg` (s->arg). Design: docs/plans/navlink-xfer-substrate.md.
 */
#include "comm/xfer/xfer_providers.h"

#include "storage/fs_owner.h"
#include "vfs.h"

/** @noreq file-size helper */
static long file_size_of(const char *path) {
  vfs_fd_t fd = vfs_open(path, VFS_O_RDONLY);
  if (fd < 0)
    return -1;
  long sz = vfs_lseek(fd, 0, VFS_SEEK_END);
  vfs_close(fd);
  return sz;
}

/** @noreq FILE provider open (fs_owner-backed adapter) */
static int file_open(xfer_session_t *s, const xfer_open_args_t *a,
                     uint32_t *total_out) {
  (void)a;
  if (s->arg[0] == '\0')
    return -1; /* no path -> FAILED */
  if (s->dir == XFER_DIR_DOWNLOAD) {
    long sz = file_size_of(s->arg);
    if (sz < 0)
      return -2; /* not found / unreadable */
    if (total_out)
      *total_out = (uint32_t)sz;
  } else {
    /* Fresh upload (offset 0) replaces the file; a resume (offset > 0) appends
     * into the existing one, so only truncate at the start. */
    if (s->offset_start == 0)
      fs_owner_truncate(s->arg);
    /* Start the per-session write-at bookkeeping so flush() can report true
     * persistence (pending/committed/failed) to the GCS. */
    fs_owner_writeat_reset(s->session);
  }
  return 0;
}

/** @noreq fs_owner_read_at adapter */
static int file_read(xfer_session_t *s, uint32_t off, uint8_t *buf,
                     uint16_t max) {
  return fs_owner_read_at(s->arg, off, buf, max);
}

/** @noreq fs_owner write-at adapter */
static int file_write(xfer_session_t *s, uint32_t off, const uint8_t *buf,
                      uint16_t len) {
  /* true => accepted (return len); false => write-at lane full => 0 = backpressure
   * (the SM holds the cursor, the next XFER_ACK pauses the GCS). len is bounded
   * by chunk_size <= FS_WRITEAT_PAYLOAD_MAX, so a false is never a permanent
   * reject here. The write is fire-and-forget into fs_owner; file_flush() below
   * reports its eventual persistence. */
  return fs_owner_enqueue_write_at(s->session, s->arg, off, buf, len) ? (int)len
                                                                      : 0;
}

/* Upload completion: 1 = every chunk durably written (-> DONE), 0 = still
 * flushing (fs_owner draining/retrying), <0 = a write permanently failed
 * (-> FAILED). The xfer SM polls this after EOF before the terminal ack, so the
 * GCS's DONE means the bytes are on the SD card, not merely enqueued. */
/** @noreq fs_owner persistence-poll adapter */
static int file_flush(xfer_session_t *s) {
  if (fs_owner_writeat_failed(s->session))
    return -1;
  return (fs_owner_writeat_pending(s->session) == 0u) ? 1 : 0;
}

static const xfer_provider_t FILE_PROVIDER = {
    .service_id = XFER_SVC_FILE,
    .name = "file",
    .open = file_open,
    .read = file_read,
    .write = file_write,
    .poll = NULL,
    .close = NULL,
    .flush = file_flush,
};

/** @noreq provider registration */
int file_provider_register(void) {
  return xfer_register_provider(&FILE_PROVIDER);
}

/** @noreq provider registration glue */
int xfer_providers_register_all(void) {
  int rc = 0;
  rc |= file_provider_register();
  rc |= log_provider_register();
  rc |= stream_provider_register();
  return rc;
}
