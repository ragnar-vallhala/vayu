/**
 * @file src/comm/xfer/providers/log_provider.c
 * @brief LOG xfer provider — download-only dump of the circular blackbox files.
 *
 * A thin specialisation of the file path: the XFER_OPEN `arg` names one of the
 * three blackbox logs ("navlink" / "system" / "general"); the provider resolves
 * it to the fixed SD path and serves a raw circular dump (the GCS de-wraps using
 * the wrap counters in SYSTEM_HEALTH). write is NULL, so an upload is rejected
 * DENIED by the SM's capability gate. Design: docs/plans/navlink-xfer-substrate.md.
 */
#include "comm/xfer/xfer_providers.h"

#include "storage/fs_owner.h"
#include "variables.h" /* *_LOGGING_FILENAME */
#include "vfs.h"

/* Resolve the arg name to a blackbox path (default: general). */
/** @noreq blackbox-name to path resolver helper */
static const char *log_path_for(const char *name) {
  if (name[0] == 'n' || name[0] == 'N')
    return NAVLINK_LOGGING_FILENAME;
  if (name[0] == 's' || name[0] == 'S')
    return SYS_LOGGING_FILENAME;
  return GENERAL_LOGGING_FILENAME;
}

/** @noreq LOG provider open (fs_owner-backed adapter) */
static int log_open(xfer_session_t *s, const xfer_open_args_t *a,
                    uint32_t *total_out) {
  (void)a;
  const char *path = log_path_for(s->arg);
  s->user = (void *)path; /* stash the resolved path for read() */
  vfs_fd_t fd = vfs_open(path, VFS_O_RDONLY);
  if (fd < 0)
    return -1;
  long sz = vfs_lseek(fd, 0, VFS_SEEK_END);
  vfs_close(fd);
  if (sz < 0)
    return -2;
  if (total_out)
    *total_out = (uint32_t)sz;
  return 0;
}

/** @noreq fs_owner_read_at adapter */
static int log_read(xfer_session_t *s, uint32_t off, uint8_t *buf,
                    uint16_t max) {
  return fs_owner_read_at((const char *)s->user, off, buf, max);
}

static const xfer_provider_t LOG_PROVIDER = {
    .service_id = XFER_SVC_LOG,
    .name = "log",
    .open = log_open,
    .read = log_read,
    .write = NULL, /* download-only: uploads -> DENIED */
    .poll = NULL,
    .close = NULL,
};

/** @noreq provider registration */
int log_provider_register(void) {
  return xfer_register_provider(&LOG_PROVIDER);
}
