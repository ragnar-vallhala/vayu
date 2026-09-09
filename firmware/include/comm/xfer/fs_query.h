/**
 * @file include/comm/xfer/fs_query.h
 * @brief Filesystem-navigation service: list a directory + stat a path.
 *
 * The query half of the storage feature set (the xfer substrate moves bytes;
 * this browses). FS_LIST streams FS_ENTRY rows for a directory; FS_INFO replies
 * with one FS_INFO_REPLY. A path that does not exist is reported distinctly
 * (result = DENIED) rather than as an empty success.
 *
 * Same threading contract as xfer (the C1->C3 invariant): the comm-task handlers
 * only stash the request; the blocking VFS walk (fs_owner_opendir/readdir/stat)
 * and all emission run in xfer_service_task via fs_query_tick(). Codec-blind:
 * emission goes through an injected fs_query_tx_ops_t (firmware wires the codec
 * seam; the host test wires a capture). docs/reference/messages/fs_nav.md.
 */
#ifndef FS_QUERY_H
#define FS_QUERY_H

#include <stdbool.h>
#include <stdint.h>

#define FS_QUERY_PATH_MAX 48 /* = FS_LIST/FS_INFO.path char[48] */

/* result codes mirror command_result (navlink/dialect.json) by value. */
#define FSQ_RES_OK 0u     /* ACCEPTED */
#define FSQ_RES_DENIED 2u /* path does not exist */
#define FSQ_RES_BUSY 1u /* TEMPORARILY_REJECTED (a query is already running) */

/* wire msgids mirrored from the dialect (for the deferred COMMAND_ACK). */
#define FS_WIRE_MSGID_LIST 8203u
#define FS_WIRE_MSGID_INFO 8204u

/* Sentinel: the request was accepted and deferred to the tick (the router
 * returns navlink_ack_deferred()); any other value is an immediate result. */
#define FS_QUERY_DEFERRED 255

typedef struct {
  void (*command_ack)(uint32_t acked_msgid, uint8_t req_seq, uint8_t result);
  void (*entry)(uint8_t req_seq, uint8_t result, uint16_t index, uint16_t count,
                uint8_t type, uint32_t size, const char *name);
  void (*info_reply)(uint8_t req_seq, uint8_t result, uint8_t type,
                     uint32_t size, uint32_t mtime);
} fs_query_tx_ops_t;

void fs_query_init(const fs_query_tx_ops_t *tx);

/* COMM task: validate + stash. Returns FS_QUERY_DEFERRED on accept, else an
 * immediate result code (FSQ_RES_BUSY). */
int fs_query_on_list(uint8_t req_seq, uint8_t gcs_sys, uint8_t gcs_comp,
                     const char *path, uint16_t start_index);
int fs_query_on_info(uint8_t req_seq, uint8_t gcs_sys, uint8_t gcs_comp,
                     const char *path);

/* XFER task: run pending queries; `budget` caps FS_ENTRY rows emitted this call.
 * Returns rows emitted. */
int fs_query_tick(int budget);

/* Test helper. */
bool fs_query_busy(void);

#endif /* FS_QUERY_H */
