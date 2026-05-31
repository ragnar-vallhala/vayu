/**
 * @file tools/sim_host/src/host_vfs.c
 * @brief RAM-backed VFS shim for the host SITL build.
 *
 * The firmware persists tunables (IMU calibration, PID gains) to an SD
 * card through the vaios VFS. On the host there is no SD card, so this
 * shim keeps a small in-memory file table instead. It is enough for the
 * firmware to link and for tests to exercise a real save -> reload
 * round-trip within one process; files do not persist across runs.
 */
#define _GNU_SOURCE
#include "vfs.h"

#include <string.h>

#define HOST_VFS_MAX_FILES   8
#define HOST_VFS_MAX_HANDLES 8
#define HOST_VFS_FILE_CAP    4096

typedef struct {
  char     path[64];
  uint8_t  data[HOST_VFS_FILE_CAP];
  size_t   size;
  int      used;
} host_vfs_file_t;

typedef struct {
  int    used;
  int    file_idx;
  size_t cursor;
} host_vfs_handle_t;

static host_vfs_file_t   s_files[HOST_VFS_MAX_FILES];
static host_vfs_handle_t s_handles[HOST_VFS_MAX_HANDLES];

int vfs_init(void) { return 0; }

static int find_file(const char *path) {
  for (int i = 0; i < HOST_VFS_MAX_FILES; i++) {
    if (s_files[i].used && strncmp(s_files[i].path, path,
                                   sizeof s_files[i].path) == 0) {
      return i;
    }
  }
  return -1;
}

static int alloc_file(const char *path) {
  for (int i = 0; i < HOST_VFS_MAX_FILES; i++) {
    if (!s_files[i].used) {
      s_files[i].used = 1;
      s_files[i].size = 0;
      strncpy(s_files[i].path, path, sizeof s_files[i].path - 1);
      s_files[i].path[sizeof s_files[i].path - 1] = '\0';
      return i;
    }
  }
  return -1;
}

vfs_fd_t vfs_open(const char *path, int flags) {
  if (path == NULL) {
    return -1;
  }
  int fidx = find_file(path);
  if (fidx < 0) {
    if (!(flags & VFS_O_CREAT)) {
      return -1; /* mirror real VFS: missing file without CREAT fails */
    }
    fidx = alloc_file(path);
    if (fidx < 0) {
      return -1;
    }
  }
  if (flags & VFS_O_TRUNC) {
    s_files[fidx].size = 0;
  }
  for (int h = 0; h < HOST_VFS_MAX_HANDLES; h++) {
    if (!s_handles[h].used) {
      s_handles[h].used = 1;
      s_handles[h].file_idx = fidx;
      s_handles[h].cursor =
          (flags & VFS_O_APPEND) ? s_files[fidx].size : 0u;
      return h;
    }
  }
  return -1;
}

static host_vfs_handle_t *handle_of(vfs_fd_t fd) {
  if (fd < 0 || fd >= HOST_VFS_MAX_HANDLES || !s_handles[fd].used) {
    return NULL;
  }
  return &s_handles[fd];
}

int vfs_close(vfs_fd_t fd) {
  host_vfs_handle_t *h = handle_of(fd);
  if (h == NULL) {
    return -1;
  }
  h->used = 0;
  return 0;
}

int vfs_read(vfs_fd_t fd, void *buf, size_t count) {
  host_vfs_handle_t *h = handle_of(fd);
  if (h == NULL || buf == NULL) {
    return -1;
  }
  host_vfs_file_t *f = &s_files[h->file_idx];
  size_t avail = (h->cursor < f->size) ? f->size - h->cursor : 0u;
  size_t n = (count < avail) ? count : avail;
  memcpy(buf, &f->data[h->cursor], n);
  h->cursor += n;
  return (int)n;
}

int vfs_write(vfs_fd_t fd, const void *buf, size_t count) {
  host_vfs_handle_t *h = handle_of(fd);
  if (h == NULL || buf == NULL) {
    return -1;
  }
  host_vfs_file_t *f = &s_files[h->file_idx];
  if (h->cursor + count > HOST_VFS_FILE_CAP) {
    count = HOST_VFS_FILE_CAP - h->cursor;
  }
  memcpy(&f->data[h->cursor], buf, count);
  h->cursor += count;
  if (h->cursor > f->size) {
    f->size = h->cursor;
  }
  return (int)count;
}

long vfs_lseek(vfs_fd_t fd, long offset, int whence) {
  host_vfs_handle_t *h = handle_of(fd);
  if (h == NULL) {
    return -1;
  }
  host_vfs_file_t *f = &s_files[h->file_idx];
  long base = (whence == VFS_SEEK_CUR)   ? (long)h->cursor
              : (whence == VFS_SEEK_END) ? (long)f->size
                                         : 0L;
  long pos = base + offset;
  if (pos < 0) {
    return -1;
  }
  h->cursor = (size_t)pos;
  return pos;
}

int vfs_mkdir(const char *path) {
  (void)path;
  return 0;
}

int vfs_unlink(const char *path) {
  int fidx = find_file(path);
  if (fidx < 0) {
    return -1;
  }
  s_files[fidx].used = 0;
  return 0;
}

int vfs_sync(vfs_fd_t fd) {
  return handle_of(fd) ? 0 : -1;
}

int vfs_preallocate(const char *path, uint32_t size) {
  (void)size;
  int fidx = find_file(path);
  if (fidx < 0) {
    fidx = alloc_file(path);
  }
  return fidx < 0 ? -1 : 0;
}

long vfs_size(vfs_fd_t fd) {
  host_vfs_handle_t *h = handle_of(fd);
  return h ? (long)s_files[h->file_idx].size : -1;
}
