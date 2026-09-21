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
/**
 * @file sim/host/tests/test_xfer_e2e.c
 * @brief End-to-end SITL test of the xfer substrate through the REAL codec seam.
 *
 * Where test_xfer_sm.c captures the SM's abstract emit ops, this drives a full
 * RAM up+down transfer through the actual firmware encoders (navlink_xfer_tx.c's
 * xfer_build_*) and DECODES every frame with the generated navlink codec — so it
 * verifies the exact wire-field mapping the flight code uses, not just the SM
 * logic. The only thing it doesn't exercise is the final write_channel/UART hop
 * (the channel flushes to a pty in SITL); that byte-loopback lands with the
 * Python XferServer in a later phase.
 *
 *   @verifies docs/plans/navlink-xfer-substrate.md Phase C:
 *     - SM + provider registry + codec seam compose into a working RAM transfer
 *     - XFER_DATA / XFER_INFO / COMMAND_ACK fields survive encode->decode intact
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "comm/xfer/navlink_xfer.h"
#include "comm/xfer/navlink_xfer_tx.h"
#include "navlink_msgs.h" /* decode the firmware-encoded frames */

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond)                                                                  \
      printf("    ok   %s\n", (msg));                                          \
    else {                                                                     \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

#define NAVLINK_HDR_LEN                                                        \
  10u /* sync,ver,len,incompat,seq,sysid,compid,msgid[3] */

/* ---- RAM provider (proves the substrate generalises beyond SD) ----------- */
#define RAM_CAP 4096u
static struct {
  uint8_t buf[RAM_CAP];
  uint32_t size;
} RAM;
static int ram_open(xfer_session_t *s, const xfer_open_args_t *a,
                    uint32_t *total_out) {
  (void)a;
  if (s->dir == XFER_DIR_UPLOAD)
    RAM.size = 0;
  if (total_out)
    *total_out = RAM.size;
  return 0;
}
static int ram_read(xfer_session_t *s, uint32_t off, uint8_t *buf,
                    uint16_t max) {
  (void)s;
  if (off >= RAM.size)
    return 0;
  uint32_t n = RAM.size - off;
  if (n > max)
    n = max;
  memcpy(buf, RAM.buf + off, n);
  return (int)n;
}
static int ram_write(xfer_session_t *s, uint32_t off, const uint8_t *buf,
                     uint16_t len) {
  (void)s;
  if (off + len > RAM_CAP)
    return -1;
  memcpy(RAM.buf + off, buf, len);
  if (off + len > RAM.size)
    RAM.size = off + len;
  return len;
}
static void ram_close(xfer_session_t *s, int r) {
  (void)s;
  (void)r;
}
static const xfer_provider_t RAM_PROV = {.service_id = 9,
                                         .name = "ram",
                                         .open = ram_open,
                                         .read = ram_read,
                                         .write = ram_write,
                                         .close = ram_close};

/* ---- capturing emitter: encode via the REAL seam, then decode ------------ */
static struct {
  uint8_t blob[RAM_CAP];
  uint32_t assembled;
  int n_data, n_info, n_cmd_ack, n_ack;
  uint16_t last_info_chunk;
  uint32_t last_info_total;
  uint8_t last_ack_result;
} CAP;
static void cap_reset(void) { memset(&CAP, 0, sizeof CAP); }

/* strip the 10-byte header, return payload ptr + len */
static const uint8_t *payload_of(const uint8_t *frame, uint8_t *len_out) {
  *len_out = frame[2];
  return frame + NAVLINK_HDR_LEN;
}

static void cap_command_ack(const xfer_session_t *s, uint32_t msgid,
                            uint8_t req, uint8_t result, int32_t p2) {
  (void)s;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_command_ack(frame, msgid, req, result, p2);
  (void)n;
  uint8_t plen;
  const uint8_t *p = payload_of(frame, &plen);
  navlink_command_ack_wire_t w;
  navlink_command_ack_t a;
  navlink_command_ack_unpack(&w, p, plen);
  navlink_command_ack_to_aligned(&a, &w);
  CHECK(a.command == msgid && a.req_seq == req && a.result == result,
        "COMMAND_ACK round-trips (command/req_seq/result)");
  CAP.n_cmd_ack++;
}
static void cap_info(const xfer_session_t *s, uint8_t result, uint16_t cs,
                     uint32_t total, uint32_t mtime) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_info(frame, s, result, cs, total, mtime);
  (void)n;
  uint8_t plen;
  const uint8_t *p = payload_of(frame, &plen);
  navlink_xfer_info_wire_t w;
  navlink_xfer_info_t a;
  navlink_xfer_info_unpack(&w, p, plen);
  navlink_xfer_info_to_aligned(&a, &w);
  CAP.last_info_chunk = a.chunk_size;
  CAP.last_info_total = a.total_size;
  CAP.n_info++;
}
static int cap_data(const xfer_session_t *s, uint8_t flags, uint8_t len,
                    uint32_t offset, const uint8_t *buf) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_data(frame, s, flags, len, offset, buf);
  (void)n;
  uint8_t plen;
  const uint8_t *p = payload_of(frame, &plen);
  navlink_xfer_data_wire_t w;
  navlink_xfer_data_t a;
  navlink_xfer_data_unpack(&w, p, plen);
  navlink_xfer_data_to_aligned(&a, &w);
  /* trust a.len, not the array tail (the contract) */
  if (a.len && a.offset + a.len <= RAM_CAP) {
    memcpy(CAP.blob + a.offset, a.data, a.len);
    if (a.offset + a.len > CAP.assembled)
      CAP.assembled = a.offset + a.len;
  }
  CAP.n_data++;
  return 1; /* fake channel always accepts */
}
static void cap_ack(const xfer_session_t *s, uint8_t flags, uint8_t result,
                    uint32_t next_offset) {
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = xfer_build_ack(frame, s, flags, result, next_offset);
  (void)n;
  uint8_t plen;
  const uint8_t *p = payload_of(frame, &plen);
  navlink_xfer_ack_wire_t w;
  navlink_xfer_ack_t a;
  navlink_xfer_ack_unpack(&w, p, plen);
  navlink_xfer_ack_to_aligned(&a, &w);
  CAP.last_ack_result = a.result;
  (void)flags;
  (void)next_offset;
  CAP.n_ack++;
}
static const xfer_tx_ops_t CAP_TX = {cap_command_ack, cap_info, cap_data,
                                     cap_ack};

static xfer_open_args_t mkargs(uint8_t sess, uint8_t dir, uint16_t svc) {
  xfer_open_args_t a = {0};
  a.session = sess;
  a.dir = dir;
  a.mode = XFER_MODE_FILE;
  a.service_id = svc;
  a.req_seq = (uint8_t)(0x50u + sess);
  a.gcs_sys = 1;
  a.gcs_comp = 1;
  return a;
}

static void test_ram_roundtrip_through_codec(void) {
  printf("  test_ram_roundtrip_through_codec\n");
  memset(&RAM, 0, sizeof RAM);
  xfer_reset_all();
  cap_reset();

  uint8_t src[1500];
  for (uint32_t i = 0; i < sizeof src; i++)
    src[i] = (uint8_t)(i * 53u + 11u);

  /* upload via the SM (chunks come in as the GCS would send them) */
  xfer_open_args_t up = mkargs(0, XFER_DIR_UPLOAD, 9);
  CHECK(xfer_on_open(&up) == XFER_OPEN_DEFERRED, "upload open deferred");
  xfer_tick(0, 0, 8);
  uint32_t off = 0;
  while (off < sizeof src) {
    uint16_t k = (uint16_t)(sizeof src - off);
    if (k > XFER_CHUNK_MAX)
      k = XFER_CHUNK_MAX;
    uint8_t fl = (off + k >= sizeof src) ? XFER_F_EOF : XFER_F_NONE;
    xfer_on_data(0, off, src + off, (uint8_t)k, fl);
    off += k;
  }
  CHECK(RAM.size == sizeof src && memcmp(RAM.buf, src, sizeof src) == 0,
        "upload landed in RAM provider");

  /* download back, decoding every emitted frame with the real codec */
  cap_reset();
  xfer_open_args_t dn = mkargs(1, XFER_DIR_DOWNLOAD, 9);
  xfer_on_open(&dn);
  for (int t = 0; t < 40 && xfer_session_active(1); t++) {
    xfer_tick(10 + (uint32_t)t, 0, 8);
    xfer_on_ack(1, CAP.assembled, XFER_F_NONE);
  }
  CHECK(CAP.n_info >= 1 && CAP.last_info_total == sizeof src,
        "decoded XFER_INFO carries the true total_size");
  CHECK(CAP.last_info_chunk == XFER_CHUNK_MAX,
        "decoded XFER_INFO advertises chunk_size 247");
  CHECK(CAP.assembled == sizeof src, "downloaded every byte (decoded)");
  CHECK(memcmp(CAP.blob, src, sizeof src) == 0,
        "round-tripped content matches through the real codec");
}

int main(void) {
  printf("== xfer end-to-end (real codec seam) ==\n");
  xfer_init(&CAP_TX);
  xfer_register_provider(&RAM_PROV);
  test_ram_roundtrip_through_codec();
  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
