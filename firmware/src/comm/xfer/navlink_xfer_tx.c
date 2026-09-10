/**
 * @file src/comm/xfer/navlink_xfer_tx.c
 * @brief Codec/channel seam for the xfer substrate (see header).
 *
 * Mirrors navlink_tx.c: build a generated message struct, encode to a frame,
 * write_channel(g_telemetry_channel). This is the only xfer translation unit
 * that includes navlink_msgs.h — the core SM and providers stay codec-blind.
 */
#include "comm/xfer/navlink_xfer_tx.h"

#include "comm/channel.h"  /* write_channel, channel_t */
#include "sys/sys_utils.h" /* get_device_id */
#include "navlink_msgs.h"  /* generated codec — included ONLY here in xfer/ */

extern channel_t g_telemetry_channel; /* defined in telemetry_task.c */

/* One TX seq across all xfer frames (cosmetic; reliability is offset-based). */
static uint8_t s_seq;

/** @noreq codec build helper (wire seam) */
size_t xfer_build_command_ack(uint8_t *frame, uint32_t acked_msgid,
                              uint8_t req_seq, uint8_t result, int32_t param2) {
  navlink_command_ack_t m = {0};
  m.command = acked_msgid;
  m.req_seq = req_seq;
  m.result = result;
  m.progress = (result == XFER_RES_IN_PROGRESS) ? 0u : 100u;
  m.result_param2 = param2;
  return navlink_command_ack_encode(frame, &m, s_seq++, get_device_id(), 1);
}

/** @noreq codec build helper (wire seam) */
size_t xfer_build_info(uint8_t *frame, const xfer_session_t *s, uint8_t result,
                       uint16_t chunk_size, uint32_t total_size,
                       uint32_t mtime) {
  navlink_xfer_info_t m = {0};
  m.session = s->session;
  m.result = result;
  m.chunk_size = chunk_size;
  m.total_size = total_size;
  m.mtime = mtime;
  return navlink_xfer_info_encode(frame, &m, s_seq++, get_device_id(), 1);
}

/** @noreq codec build helper (wire seam) */
size_t xfer_build_data(uint8_t *frame, const xfer_session_t *s, uint8_t flags,
                       uint8_t len, uint32_t offset, const uint8_t *buf) {
  navlink_xfer_data_t m = {0};
  m.session = s->session;
  m.flags = flags;
  m.len = len;
  m.offset = offset;
  for (uint8_t i = 0; i < len && i < XFER_CHUNK_MAX; i++)
    m.data[i] = buf[i];
  return navlink_xfer_data_encode(frame, &m, s_seq++, get_device_id(), 1);
}

/** @noreq codec build helper (wire seam) */
size_t xfer_build_ack(uint8_t *frame, const xfer_session_t *s, uint8_t flags,
                      uint8_t result, uint32_t next_offset) {
  navlink_xfer_ack_t m = {0};
  m.session = s->session;
  m.flags = flags;
  m.result = result;
  m.next_offset = next_offset;
  return navlink_xfer_ack_encode(frame, &m, s_seq++, get_device_id(), 1);
}

/* ---- xfer_tx_ops_t: build + send ----------------------------------------- */
/** @noreq tx-op glue (build + write_channel) */
static void op_command_ack(const xfer_session_t *s, uint32_t acked_msgid,
                           uint8_t req_seq, uint8_t result, int32_t param2) {
  (void)s;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      xfer_build_command_ack(frame, acked_msgid, req_seq, result, param2);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}
/** @noreq tx-op glue (build + write_channel) */
static void op_info(const xfer_session_t *s, uint8_t result,
                    uint16_t chunk_size, uint32_t total_size, uint32_t mtime) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_info(frame, s, result, chunk_size, total_size, mtime);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}
/** @noreq tx-op glue (build + write_channel_xfer) */
static int op_data(const xfer_session_t *s, uint8_t flags, uint8_t len,
                   uint32_t offset, const uint8_t *buf) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_data(frame, s, flags, len, offset, buf);
  /* Use the xfer-reserved tail of the TX ring so a saturating telemetry stream
   * can't starve the download. NONE == accepted (will transmit). ERROR == even
   * the reserved headroom is momentarily full -> report it so the SM holds the
   * cursor and retries next tick (no silent chunk loss, self-paced to the link). */
  return write_channel_xfer(g_telemetry_channel, frame, (uint16_t)n) == NONE
             ? 1
             : 0;
}
/** @noreq tx-op glue (build + write_channel) */
static void op_ack(const xfer_session_t *s, uint8_t flags, uint8_t result,
                   uint32_t next_offset) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_ack(frame, s, flags, result, next_offset);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

const xfer_tx_ops_t g_xfer_tx_ops = {op_command_ack, op_info, op_data, op_ack};

/* ---- fs_query (filesystem navigation) seam ------------------------------- */
/** @noreq fs-query tx-op glue (build + write_channel) */
static void fsq_command_ack(uint32_t acked_msgid, uint8_t req_seq,
                            uint8_t result) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_command_ack(frame, acked_msgid, req_seq, result, 0);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

/** @noreq fs-query tx-op glue (build + write_channel) */
static void fsq_entry(uint8_t req_seq, uint8_t result, uint16_t index,
                      uint16_t count, uint8_t type, uint32_t size,
                      const char *name) {
  navlink_fs_entry_t m = {0};
  m.req_seq = req_seq;
  m.result = result;
  m.index = index;
  m.count = count;
  m.type = type;
  m.size = size;
  for (uint8_t i = 0; i < sizeof(m.name) && name[i]; i++)
    m.name[i] = name[i];
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_fs_entry_encode(frame, &m, s_seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

/** @noreq fs-query tx-op glue (build + write_channel) */
static void fsq_info_reply(uint8_t req_seq, uint8_t result, uint8_t type,
                           uint32_t size, uint32_t mtime) {
  navlink_fs_info_reply_t m = {0};
  m.req_seq = req_seq;
  m.result = result;
  m.type = type;
  m.size = size;
  m.mtime = mtime;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_fs_info_reply_encode(frame, &m, s_seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

const fs_query_tx_ops_t g_fs_query_tx_ops = {fsq_command_ack, fsq_entry,
                                             fsq_info_reply};
