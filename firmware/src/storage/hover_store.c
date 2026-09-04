#include "storage/hover_store.h"

#include "est/hover_estimate.h"
#include "storage/fs_owner.h"
#include "vfs.h"

#include <stdint.h>

/* 8 bytes: magic + value. */
typedef struct {
  uint32_t magic;
  float hover;
} hover_store_t;

/** @noreq Boot-time load of the persisted hover collective. */
float hover_store_load(float fallback) {
  hover_store_t s = {0};
  vfs_fd_t fd = vfs_open(HOVER_STORE_PATH, VFS_O_RDONLY);
  if (fd < 0) {
    return fallback; /* no card, or never saved — the guess stands */
  }
  int n = vfs_read(fd, &s, sizeof s);
  vfs_close(fd);

  if (n != (int)sizeof s || s.magic != HOVER_STORE_MAGIC) {
    vayu_log("hover: store absent/bad magic, using %d/1000", (int)(fallback * 1000));
    return fallback;
  }
  /* Range-check before trusting it: a plausible-looking file with a wild value
   * would open the next lift-off at that value. */
  if (!(s.hover > HOVER_EST_MIN) || !(s.hover < HOVER_EST_MAX)) {
    vayu_log("hover: stored %d/1000 out of band, using %d/1000",
             (int)(s.hover * 1000), (int)(fallback * 1000));
    return fallback;
  }
  vayu_log("hover: restored %d/1000 from SD", (int)(s.hover * 1000));
  return s.hover;
}

/** @noreq Queue a hover save through the FS owner. */
bool hover_store_save(float hover) {
  if (!(hover > HOVER_EST_MIN) || !(hover < HOVER_EST_MAX)) {
    return false;
  }
  hover_store_t s = {HOVER_STORE_MAGIC, hover};
  /* Internal slot, not session 0: a disarm during a GCS upload would otherwise
   * corrupt that transfer's pending/committed accounting. */
  return fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, HOVER_STORE_PATH, 0,
                                   &s, sizeof s);
}
