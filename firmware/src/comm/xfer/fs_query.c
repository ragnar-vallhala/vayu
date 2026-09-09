/**
 * @file src/comm/xfer/fs_query.c
 * @brief Filesystem-navigation service (see header).
 *
 * One list slot + one info slot (the GCS drives one browse at a time). The walk
 * and emission run entirely in fs_query_tick() on the xfer task; the comm-task
 * handlers only copy the request in.
 */
#include "comm/xfer/fs_query.h"

#include "storage/fs_owner.h"

static const fs_query_tx_ops_t *s_tx;

typedef struct {
  bool active;    /* a request is pending/streaming */
  bool acked;     /* COMMAND_ACK emitted */
  bool opened;    /* directory opened (or open attempted) */
  bool not_found; /* path is not a listable directory */
  uint8_t req_seq;
  uint8_t gcs_sys, gcs_comp;
  uint16_t start_index;
  uint16_t index; /* next entry index to emit */
  vfs_dir_t dir;
  char path[FS_QUERY_PATH_MAX];
} fs_list_t;

typedef struct {
  bool active;
  bool acked;
  uint8_t req_seq;
  uint8_t gcs_sys, gcs_comp;
  char path[FS_QUERY_PATH_MAX];
} fs_info_t;

static fs_list_t s_list;
static fs_info_t s_info;

/** @noreq bounded string-copy helper */
static void copy_path(char *dst, const char *src) {
  uint32_t i = 0;
  for (; i < FS_QUERY_PATH_MAX - 1u && src[i] != '\0'; i++)
    dst[i] = src[i];
  dst[i] = '\0';
}

/** @noreq init glue (binds the emitter, clears slots) */
void fs_query_init(const fs_query_tx_ops_t *tx) {
  s_tx = tx;
  s_list.active = false;
  s_info.active = false;
}

/** @noreq state predicate */
bool fs_query_busy(void) { return s_list.active || s_info.active; }

/** @implements COMM-FS-001 */
int fs_query_on_list(uint8_t req_seq, uint8_t gcs_sys, uint8_t gcs_comp,
                     const char *path, uint16_t start_index) {
  if (path == NULL)
    return FSQ_RES_DENIED;
  /* Idempotent retransmit of the in-flight request. */
  if (s_list.active && s_list.req_seq == req_seq)
    return FS_QUERY_DEFERRED;
  if (s_list.active)
    return FSQ_RES_BUSY;
  s_list.active = true;
  s_list.acked = false;
  s_list.opened = false;
  s_list.not_found = false;
  s_list.req_seq = req_seq;
  s_list.gcs_sys = gcs_sys;
  s_list.gcs_comp = gcs_comp;
  s_list.start_index = start_index;
  s_list.index = 0;
  copy_path(s_list.path, path);
  return FS_QUERY_DEFERRED;
}

/** @implements COMM-FS-001 */
int fs_query_on_info(uint8_t req_seq, uint8_t gcs_sys, uint8_t gcs_comp,
                     const char *path) {
  if (path == NULL)
    return FSQ_RES_DENIED;
  if (s_info.active && s_info.req_seq == req_seq)
    return FS_QUERY_DEFERRED;
  if (s_info.active)
    return FSQ_RES_BUSY;
  s_info.active = true;
  s_info.acked = false;
  s_info.req_seq = req_seq;
  s_info.gcs_sys = gcs_sys;
  s_info.gcs_comp = gcs_comp;
  copy_path(s_info.path, path);
  return FS_QUERY_DEFERRED;
}

/* ---- tick: do the blocking VFS work + emit ------------------------------- */
/** @implements COMM-FS-001 */
static int tick_list(int budget) {
  if (!s_list.active)
    return 0;
  if (!s_list.acked) {
    if (s_tx && s_tx->command_ack)
      s_tx->command_ack(FS_WIRE_MSGID_LIST, s_list.req_seq, FSQ_RES_OK);
    s_list.acked = true;
  }
  if (!s_list.opened) {
    s_list.opened = true;
    s_list.dir = fs_owner_opendir(s_list.path);
    if (s_list.dir < 0) {
      s_list.not_found = true;
    } else {
      /* Skip to the resume point. */
      vfs_dirent_t skip;
      while (s_list.index < s_list.start_index &&
             fs_owner_readdir(s_list.dir, &skip) == 1)
        s_list.index++;
    }
  }
  if (s_list.not_found) {
    if (s_tx && s_tx->entry)
      s_tx->entry(s_list.req_seq, FSQ_RES_DENIED, 0, 0, 0, 0, "");
    s_list.active = false;
    return 1;
  }

  int emitted = 0;
  vfs_dirent_t ent;
  while (emitted < budget) {
    int n = fs_owner_readdir(s_list.dir, &ent);
    if (n == 1) {
      if (s_tx && s_tx->entry)
        s_tx->entry(s_list.req_seq, FSQ_RES_OK, s_list.index, 0,
                    ent.is_dir ? 1u : 0u, ent.size, ent.name);
      s_list.index++;
      emitted++;
    } else {
      /* End of directory: terminal entry carries the total count + empty name. */
      if (s_tx && s_tx->entry)
        s_tx->entry(s_list.req_seq, FSQ_RES_OK, s_list.index, s_list.index, 0,
                    0, "");
      fs_owner_closedir(s_list.dir);
      s_list.active = false;
      emitted++;
      break;
    }
  }
  return emitted;
}

/** @implements COMM-FS-001 */
static int tick_info(void) {
  if (!s_info.active)
    return 0;
  if (!s_info.acked) {
    if (s_tx && s_tx->command_ack)
      s_tx->command_ack(FS_WIRE_MSGID_INFO, s_info.req_seq, FSQ_RES_OK);
    s_info.acked = true;
  }
  vfs_stat_t st;
  int r = fs_owner_stat(s_info.path, &st);
  if (r == 0 && st.exists) {
    if (s_tx && s_tx->info_reply)
      s_tx->info_reply(s_info.req_seq, FSQ_RES_OK, st.is_dir ? 1u : 0u, st.size,
                       st.mtime);
  } else {
    if (s_tx && s_tx->info_reply)
      s_tx->info_reply(s_info.req_seq, FSQ_RES_DENIED, 0, 0, 0);
  }
  s_info.active = false;
  return 1;
}

/** @implements COMM-FS-001 */
int fs_query_tick(int budget) {
  int emitted = tick_list(budget);
  emitted += tick_info();
  return emitted;
}
