/**
 * @file sim/host/tests/test_xfer_providers.c
 * @brief SITL test of the real xfer providers (file / log / stream).
 *
 * Drives the full firmware stack — SM + provider + fs_owner (file/log) or a
 * live source (stream) — through the abstract emit ops (the codec wire is
 * covered by test_xfer_e2e). Uses the host VFS, so file sizes stay under its
 * 4 KB per-file cap; multi-KB / circular-wrap paths are on-target only (a
 * documented SITL gap, same as test_fs_owner).
 *
 *   @verifies docs/plans/navlink-xfer-substrate.md Phase E:
 *     - FILE: upload then download a file byte-identically through fs_owner
 *     - LOG: download a blackbox path; upload is DENIED (no write vtable)
 *     - STREAM: a named live source emits paced samples with monotonic offsets
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm/xfer/navlink_xfer.h"
#include "comm/xfer/xfer_providers.h"
#include "storage/fs_owner.h"
#include "variables.h" /* NAVLINK_LOGGING_FILENAME */

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

/* ---- capturing emitter (reassembles downloaded/streamed data) ------------ */
#define CAPBUF 4096u
static struct {
  uint8_t blob[CAPBUF];
  uint32_t assembled;
  int n_data, n_info, n_cmd_ack;
  uint8_t last_open_result;
  uint32_t data_offsets[256];
  int data_lens[256];
} CAP;
static void cap_reset(void) { memset(&CAP, 0, sizeof CAP); }

static void e_cmd_ack(const xfer_session_t *s, uint32_t msgid, uint8_t req,
                      uint8_t result, int32_t p2) {
  (void)s;
  (void)msgid;
  (void)req;
  (void)p2;
  CAP.last_open_result = result;
  CAP.n_cmd_ack++;
}
static void e_info(const xfer_session_t *s, uint8_t result, uint16_t cs,
                   uint32_t total, uint32_t mtime) {
  (void)s;
  (void)result;
  (void)cs;
  (void)total;
  (void)mtime;
  CAP.n_info++;
}
static int e_data(const xfer_session_t *s, uint8_t flags, uint8_t len,
                  uint32_t offset, const uint8_t *buf) {
  (void)s;
  (void)flags;
  if (CAP.n_data < 256) {
    CAP.data_offsets[CAP.n_data] = offset;
    CAP.data_lens[CAP.n_data] = len;
  }
  if (len && offset + len <= CAPBUF) {
    memcpy(CAP.blob + offset, buf, len);
    if (offset + len > CAP.assembled)
      CAP.assembled = offset + len;
  }
  CAP.n_data++;
  return 1; /* fake channel always accepts */
}
static void e_ack(const xfer_session_t *s, uint8_t flags, uint8_t result,
                  uint32_t next_offset) {
  (void)s;
  (void)flags;
  (void)result;
  (void)next_offset;
}
static const xfer_tx_ops_t CAP_TX = {e_cmd_ack, e_info, e_data, e_ack};

static xfer_open_args_t mkargs(uint8_t sess, uint8_t dir, uint8_t mode,
                               uint16_t svc, const char *arg, uint16_t rate) {
  xfer_open_args_t a = {0};
  a.session = sess;
  a.dir = dir;
  a.mode = mode;
  a.service_id = svc;
  a.rate_hz = rate;
  a.req_seq = (uint8_t)(0x60u + sess);
  a.gcs_sys = 1;
  a.gcs_comp = 1;
  if (arg)
    snprintf(a.arg, XFER_ARG_MAX, "%s", arg);
  return a;
}

/* drive a download to completion, acking cumulative progress like a GCS */
static void run_download(uint8_t sess, uint32_t t0) {
  for (int t = 0; t < 60 && xfer_session_active(sess); t++) {
    xfer_tick(t0 + (uint32_t)t, 0, 8);
    xfer_on_ack(sess, CAP.assembled, XFER_F_NONE);
  }
}

/* ================================ FILE ================================== */
static void test_file_roundtrip(void) {
  printf("  test_file_roundtrip\n");
  const char *path = "0:xfer_file.bin";
  uint8_t src[2000];
  for (uint32_t i = 0; i < sizeof src; i++)
    src[i] = (uint8_t)(i * 17u + 3u);

  /* upload */
  xfer_reset_all();
  cap_reset();
  xfer_open_args_t up =
      mkargs(0, XFER_DIR_UPLOAD, XFER_MODE_FILE, XFER_SVC_FILE, path, 0);
  CHECK(xfer_on_open(&up) == XFER_OPEN_DEFERRED, "FILE upload open deferred");
  xfer_tick(0, 0, 8); /* provider->open truncates */
  CHECK(CAP.last_open_result == XFER_RES_ACCEPTED, "FILE upload ACCEPTED");
  uint32_t off = 0;
  while (off < sizeof src) {
    uint16_t k = (uint16_t)(sizeof src - off);
    if (k > XFER_CHUNK_MAX)
      k = XFER_CHUNK_MAX;
    uint8_t fl = (off + k >= sizeof src) ? XFER_F_EOF : XFER_F_NONE;
    xfer_on_data(0, off, src + off, (uint8_t)k, fl);
    fs_owner_pump(); /* the FS task drains the write-at lane */
    off += k;
  }
  uint8_t chk[2000];
  int n = fs_owner_read_at(path, 0, chk, sizeof chk);
  CHECK(n == (int)sizeof src && memcmp(chk, src, sizeof src) == 0,
        "FILE upload persisted to SD via fs_owner");

  /* download the same file back */
  cap_reset();
  xfer_open_args_t dn =
      mkargs(1, XFER_DIR_DOWNLOAD, XFER_MODE_FILE, XFER_SVC_FILE, path, 0);
  xfer_on_open(&dn);
  run_download(1, 100);
  CHECK(CAP.assembled == sizeof src, "FILE download fetched every byte");
  CHECK(memcmp(CAP.blob, src, sizeof src) == 0,
        "FILE round-trip byte-identical");
}

/* ================================ LOG =================================== */
static void test_log_provider(void) {
  printf("  test_log_provider\n");
  /* Seed a small record at the navlink blackbox path (the 64 KB prealloc is
   * skipped on host; we just write a sub-4KB file there directly). */
  uint8_t rec[200]; /* one write-at chunk (<= FS_WRITEAT_PAYLOAD_MAX) */
  for (uint32_t i = 0; i < sizeof rec; i++)
    rec[i] = (uint8_t)(0x40u + (i & 0x3Fu));
  CHECK(fs_owner_enqueue_write_at(0, NAVLINK_LOGGING_FILENAME, 0, rec,
                                  sizeof rec),
        "seed navlink log record");
  fs_owner_pump();

  /* download via the LOG provider (arg selects "navlink") */
  xfer_reset_all();
  cap_reset();
  xfer_open_args_t dn =
      mkargs(0, XFER_DIR_DOWNLOAD, XFER_MODE_FILE, XFER_SVC_LOG, "navlink", 0);
  xfer_on_open(&dn);
  run_download(0, 0);
  CHECK(CAP.assembled == sizeof rec, "LOG download read the seeded record");
  CHECK(memcmp(CAP.blob, rec, sizeof rec) == 0, "LOG content matches");

  /* upload to the LOG provider is DENIED (no write vtable). */
  xfer_reset_all();
  xfer_open_args_t up =
      mkargs(0, XFER_DIR_UPLOAD, XFER_MODE_FILE, XFER_SVC_LOG, "navlink", 0);
  CHECK(xfer_on_open(&up) == XFER_RES_DENIED, "LOG upload DENIED");
}

/* ================================ STREAM ================================ */
static uint32_t s_stream_ctr;
static int count_source(uint8_t *buf, uint16_t max) {
  if (max < 4)
    return 0;
  buf[0] = (uint8_t)s_stream_ctr;
  buf[1] = (uint8_t)(s_stream_ctr >> 8);
  buf[2] = (uint8_t)(s_stream_ctr >> 16);
  buf[3] = (uint8_t)(s_stream_ctr >> 24);
  s_stream_ctr++;
  return 4;
}

static void test_stream_provider(void) {
  printf("  test_stream_provider\n");
  s_stream_ctr = 0;
  CHECK(xfer_stream_register_source("count", count_source) == 0,
        "registered synthetic stream source");

  xfer_reset_all();
  cap_reset();
  xfer_open_args_t op = mkargs(0, XFER_DIR_DOWNLOAD, XFER_MODE_STREAM,
                               XFER_SVC_STREAM, "count", 100 /* Hz -> 10ms */);
  CHECK(xfer_on_open(&op) == XFER_OPEN_DEFERRED, "STREAM open deferred");

  /* ~1 s at 100 Hz, stepping 10 ms/tick -> ~100 samples (one per tick). */
  for (uint32_t t = 0; t <= 200; t += 10)
    xfer_tick(t, 0, 8);

  CHECK(CAP.n_data >= 15 && CAP.n_data <= 30,
        "STREAM emitted ~one sample per 10 ms tick");
  bool monotonic = true;
  for (int i = 1; i < CAP.n_data; i++)
    if (CAP.data_offsets[i] <= CAP.data_offsets[i - 1] || CAP.data_lens[i] != 4)
      monotonic = false;
  CHECK(monotonic, "STREAM offsets strictly increase, 4 B samples");

  /* close cleanly */
  xfer_on_close(0, 0x70, XFER_RES_ACCEPTED);
  xfer_tick(250, 0, 8);
  CHECK(!xfer_session_active(0), "STREAM closed cleanly");
}

int main(void) {
  printf("== xfer providers SITL verification ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_xfer_prov_test", 1);
  xfer_init(&CAP_TX);
  fs_owner_init();
  CHECK(xfer_providers_register_all() == 0, "all providers registered");

  test_file_roundtrip();
  test_log_provider();
  test_stream_provider();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
