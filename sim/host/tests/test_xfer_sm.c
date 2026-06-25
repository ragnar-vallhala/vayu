/**
 * @file sim/host/tests/test_xfer_sm.c
 * @brief SITL unit suite for the NavLink xfer (FTP) substrate core SM.
 *
 * Links the real firmware SM (src/comm/xfer/navlink_xfer.c in vayu_sitl_core) and
 * drives its public contract directly with a fake in-memory provider and a
 * capturing emitter (xfer_tx_ops_t). No codec, no channel, no scheduler — the
 * comm-task handlers (on_open/on_data/on_ack/on_close) and the xfer-task tick are
 * called synchronously so every branch is deterministic.
 *
 *   @verifies docs/plans/navlink-xfer-substrate.md "Verification / SM unit":
 *     - up -> down roundtrip (memcmp)
 *     - resume after a dropped chunk (NAK rewinds the cursor)
 *     - out-of-order upload only advances on contiguous offset
 *     - session-table-full / slot busy -> TEMPORARILY_REJECTED
 *     - unknown service -> UNSUPPORTED; bad dir/mode -> DENIED
 *     - offset past EOF -> immediate DONE
 *     - provider open error -> FAILED (deferred ACK carries the result)
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "comm/xfer/navlink_xfer.h"

static int g_checks = 0;
static int g_fails = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    g_checks++;                                                                 \
    if (cond) {                                                                 \
      printf("    ok   %s\n", (msg));                                           \
    } else {                                                                    \
      g_fails++;                                                                \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);             \
    }                                                                           \
  } while (0)

/* ============================ capturing emitter ========================== */
typedef struct {
  uint32_t offset;
  uint8_t len;
  uint8_t flags;
  uint8_t data[XFER_CHUNK_MAX];
} cap_data_t;

static struct {
  cap_data_t data[256];
  int n_data;
  struct {
    uint32_t msgid;
    uint8_t req_seq, result;
    int32_t param2;
  } cmd_ack[16];
  int n_cmd_ack;
  struct {
    uint8_t result;
    uint16_t chunk_size;
    uint32_t total;
  } info[16];
  int n_info;
  struct {
    uint8_t flags, result;
    uint32_t next_offset;
  } ack[64];
  int n_ack;
  int data_by_session[XFER_MAX_SESSIONS];
} CAP;

static void cap_reset(void) { memset(&CAP, 0, sizeof CAP); }

static void e_command_ack(const xfer_session_t *s, uint32_t msgid, uint8_t req,
                          uint8_t result, int32_t p2) {
  (void)s;
  int i = CAP.n_cmd_ack++;
  CAP.cmd_ack[i].msgid = msgid;
  CAP.cmd_ack[i].req_seq = req;
  CAP.cmd_ack[i].result = result;
  CAP.cmd_ack[i].param2 = p2;
}
static void e_info(const xfer_session_t *s, uint8_t result, uint16_t cs,
                   uint32_t total, uint32_t mtime) {
  (void)s;
  (void)mtime;
  int i = CAP.n_info++;
  CAP.info[i].result = result;
  CAP.info[i].chunk_size = cs;
  CAP.info[i].total = total;
}
static int e_data(const xfer_session_t *s, uint8_t flags, uint8_t len,
                  uint32_t offset, const uint8_t *buf) {
  int i = CAP.n_data++;
  CAP.data[i].offset = offset;
  CAP.data[i].len = len;
  CAP.data[i].flags = flags;
  if (s->session < XFER_MAX_SESSIONS)
    CAP.data_by_session[s->session]++;
  if (len)
    memcpy(CAP.data[i].data, buf, len);
  return 1; /* fake channel always accepts */
}
static void e_ack(const xfer_session_t *s, uint8_t flags, uint8_t result,
                  uint32_t next_offset) {
  (void)s;
  int i = CAP.n_ack++;
  CAP.ack[i].flags = flags;
  CAP.ack[i].result = result;
  CAP.ack[i].next_offset = next_offset;
}
static const xfer_tx_ops_t TX = {e_command_ack, e_info, e_data, e_ack};

/* ============================ fake provider ============================== */
/* One shared in-memory file: upload writes into it, download reads from it. */
#define FAKE_CAP 4096u
static struct {
  uint8_t buf[FAKE_CAP];
  uint32_t size;
  int open_rc; /* injectable: make open() fail */
  int closes;
} FAKE;

static int fake_open(xfer_session_t *s, const xfer_open_args_t *a,
                     uint32_t *total_out) {
  (void)a;
  if (FAKE.open_rc < 0)
    return FAKE.open_rc;
  if (s->dir == XFER_DIR_UPLOAD)
    FAKE.size = 0; /* truncate on upload-open */
  if (total_out)
    *total_out = FAKE.size;
  return 0;
}
static int fake_read(xfer_session_t *s, uint32_t off, uint8_t *buf,
                     uint16_t max) {
  (void)s;
  if (off >= FAKE.size)
    return 0;
  uint32_t n = FAKE.size - off;
  if (n > max)
    n = max;
  memcpy(buf, FAKE.buf + off, n);
  return (int)n;
}
static int fake_write(xfer_session_t *s, uint32_t off, const uint8_t *buf,
                      uint16_t len) {
  (void)s;
  if (off + len > FAKE_CAP)
    return -1;
  memcpy(FAKE.buf + off, buf, len);
  if (off + len > FAKE.size)
    FAKE.size = off + len;
  return len;
}
static void fake_close(xfer_session_t *s, int result) {
  (void)s;
  (void)result;
  FAKE.closes++;
}
static const xfer_provider_t FAKE_PROV = {
    .service_id = 7,
    .name = "fake",
    .open = fake_open,
    .read = fake_read,
    .write = fake_write,
    .poll = NULL,
    .close = fake_close,
};
/* A download-only provider (no write) to exercise the DENIED capability gate. */
static const xfer_provider_t RDONLY_PROV = {
    .service_id = 8, .name = "ro", .open = fake_open, .read = fake_read};

static void fake_reset(void) {
  memset(&FAKE, 0, sizeof FAKE);
  xfer_reset_all();
  cap_reset();
}

/* Convenience: open + run one tick so the deferred open completes. */
static xfer_open_args_t mkargs(uint8_t sess, uint8_t dir, uint16_t svc,
                               uint32_t off) {
  xfer_open_args_t a = {0};
  a.session = sess;
  a.dir = dir;
  a.mode = XFER_MODE_FILE;
  a.service_id = svc;
  a.offset_start = off;
  a.req_seq = (uint8_t)(0x40u + sess);
  a.gcs_sys = 1;
  a.gcs_comp = 1;
  return a;
}

/* ================================ tests ================================= */
static void test_roundtrip_up_then_down(void) {
  printf("  test_roundtrip_up_then_down\n");
  fake_reset();

  /* Source payload spanning several chunks (247 B each). */
  uint8_t src[1000];
  for (uint32_t i = 0; i < sizeof src; i++)
    src[i] = (uint8_t)(i * 31u + 7u);

  /* ---- upload ---- */
  xfer_open_args_t up = mkargs(0, XFER_DIR_UPLOAD, 7, 0);
  CHECK(xfer_on_open(&up) == XFER_OPEN_DEFERRED, "upload open deferred");
  xfer_tick(0, 0, 8); /* runs provider->open, emits ACK+INFO */
  CHECK(CAP.n_cmd_ack == 1 && CAP.cmd_ack[0].result == XFER_RES_ACCEPTED,
        "upload open ACCEPTED");

  uint32_t off = 0;
  while (off < sizeof src) {
    uint16_t n = (uint16_t)(sizeof src - off);
    if (n > XFER_CHUNK_MAX)
      n = XFER_CHUNK_MAX;
    uint8_t flags = (off + n >= sizeof src) ? XFER_F_EOF : XFER_F_NONE;
    xfer_on_data(0, off, src + off, (uint8_t)n, flags);
    off += n;
  }
  CHECK(FAKE.size == sizeof src, "all uploaded bytes landed in the provider");
  CHECK(memcmp(FAKE.buf, src, sizeof src) == 0, "uploaded content matches src");

  /* ---- download the same file back ---- */
  cap_reset();
  xfer_open_args_t dn = mkargs(1, XFER_DIR_DOWNLOAD, 7, 0);
  CHECK(xfer_on_open(&dn) == XFER_OPEN_DEFERRED, "download open deferred");
  xfer_tick(10, 0, 8);
  CHECK(CAP.n_info == 1 && CAP.info[0].total == sizeof src,
        "download INFO advertises total size");

  /* Drain with a big budget over a few ticks; ack progress like a GCS. */
  uint8_t got[1000];
  uint32_t assembled = 0;
  for (int t = 0; t < 20 && xfer_session_active(1); t++) {
    int before = CAP.n_data;
    xfer_tick(20 + (uint32_t)t, 0, 8);
    for (int i = before; i < CAP.n_data; i++) {
      memcpy(got + CAP.data[i].offset, CAP.data[i].data, CAP.data[i].len);
      if (CAP.data[i].offset + CAP.data[i].len > assembled)
        assembled = CAP.data[i].offset + CAP.data[i].len;
    }
    xfer_on_ack(1, assembled, XFER_F_NONE); /* cumulative ack */
  }
  CHECK(assembled == sizeof src, "downloaded every byte");
  CHECK(memcmp(got, src, sizeof src) == 0, "round-tripped content matches");

  bool saw_eof = false;
  for (int i = 0; i < CAP.n_data; i++)
    if (CAP.data[i].flags & XFER_F_EOF)
      saw_eof = true;
  CHECK(saw_eof, "download emitted an EOF marker");
}

static void test_resume_after_drop(void) {
  printf("  test_resume_after_drop\n");
  fake_reset();
  for (uint32_t i = 0; i < 800; i++)
    FAKE.buf[i] = (uint8_t)(0xC0u ^ i);
  FAKE.size = 800;

  xfer_open_args_t dn = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  xfer_on_open(&dn);
  xfer_tick(0, 0, 2); /* open + emit up to 2 chunks */
  CHECK(CAP.n_data >= 1, "emitted initial chunks");

  /* Pretend chunk after offset 247 was lost: ack only the first 247, NAK. */
  cap_reset();
  xfer_on_ack(0, 247, XFER_F_NAK);
  xfer_tick(5, 0, 1);
  CHECK(CAP.n_data >= 1, "re-emitted after NAK");
  CHECK(CAP.data[0].offset == 247, "cursor rewound to the lowest missing byte");

  /* Finish it. */
  uint8_t got[800];
  memset(got, 0, sizeof got);
  uint32_t assembled = 247;
  memcpy(got, FAKE.buf, 247); /* the prefix acked before the NAK */
  /* Fold in chunks already re-emitted after the NAK (index 0..). */
  for (int i = 0; i < CAP.n_data; i++) {
    memcpy(got + CAP.data[i].offset, CAP.data[i].data, CAP.data[i].len);
    if (CAP.data[i].offset + CAP.data[i].len > assembled)
      assembled = CAP.data[i].offset + CAP.data[i].len;
  }
  for (int t = 0; t < 20 && xfer_session_active(0); t++) {
    int before = CAP.n_data;
    xfer_tick(10 + (uint32_t)t, 0, 4);
    for (int i = before; i < CAP.n_data; i++) {
      memcpy(got + CAP.data[i].offset, CAP.data[i].data, CAP.data[i].len);
      if (CAP.data[i].offset + CAP.data[i].len > assembled)
        assembled = CAP.data[i].offset + CAP.data[i].len;
    }
    xfer_on_ack(0, assembled, XFER_F_NONE);
  }
  CHECK(assembled == 800, "resumed to completion");
  CHECK(memcmp(got, FAKE.buf, 800) == 0, "resumed content is correct");
}

static void test_out_of_order_upload(void) {
  printf("  test_out_of_order_upload\n");
  fake_reset();
  xfer_open_args_t up = mkargs(0, XFER_DIR_UPLOAD, 7, 0);
  xfer_on_open(&up);
  xfer_tick(0, 0, 4);

  uint8_t a[100], b[100];
  memset(a, 0xA1, sizeof a);
  memset(b, 0xB2, sizeof b);

  /* A gap chunk (offset 100, before 0..100 arrived) must be ignored. */
  xfer_on_data(0, 100, b, sizeof b, XFER_F_NONE);
  CHECK(FAKE.size == 0, "gap chunk (offset > cursor) ignored");
  const xfer_session_t *s = xfer_session_get(0);
  CHECK(s->cursor == 0, "cursor unchanged after gap chunk");

  /* The contiguous chunk advances the cursor. */
  xfer_on_data(0, 0, a, sizeof a, XFER_F_NONE);
  CHECK(FAKE.size == 100 && s->cursor == 100, "contiguous chunk advances cursor");

  /* A duplicate of an already-written region is ignored (no rewind). */
  xfer_on_data(0, 0, a, sizeof a, XFER_F_NONE);
  CHECK(s->cursor == 100, "duplicate chunk does not move cursor");

  /* Now the previously-dropped chunk fits contiguously. */
  xfer_on_data(0, 100, b, sizeof b, XFER_F_EOF);
  CHECK(FAKE.size == 200 && s->cursor == 200, "late contiguous chunk accepted");
  CHECK(memcmp(FAKE.buf, a, 100) == 0 && memcmp(FAKE.buf + 100, b, 100) == 0,
        "reassembled upload is correct");
}

static void test_rejections(void) {
  printf("  test_rejections\n");
  fake_reset();

  /* unknown service -> UNSUPPORTED */
  xfer_open_args_t bad = mkargs(0, XFER_DIR_DOWNLOAD, 999, 0);
  CHECK(xfer_on_open(&bad) == XFER_RES_UNSUPPORTED, "unknown service UNSUPPORTED");

  /* upload to a read-only provider -> DENIED */
  xfer_open_args_t ro = mkargs(0, XFER_DIR_UPLOAD, 8, 0);
  CHECK(xfer_on_open(&ro) == XFER_RES_DENIED, "upload to read-only DENIED");

  /* out-of-range session slot -> TEMPORARILY_REJECTED */
  xfer_open_args_t oor = mkargs((uint8_t)XFER_MAX_SESSIONS, XFER_DIR_DOWNLOAD, 7, 0);
  CHECK(xfer_on_open(&oor) == XFER_RES_TEMPORARILY_REJECTED,
        "out-of-range session TEMPORARILY_REJECTED");

  /* occupy slot 0, then a different req_seq on the same slot is rejected */
  xfer_open_args_t a = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  CHECK(xfer_on_open(&a) == XFER_OPEN_DEFERRED, "first open on slot 0 deferred");
  xfer_open_args_t a2 = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  a2.req_seq = 0xEE;
  CHECK(xfer_on_open(&a2) == XFER_RES_TEMPORARILY_REJECTED,
        "busy slot, different req_seq TEMPORARILY_REJECTED");
  /* same (session, req_seq) is idempotent */
  CHECK(xfer_on_open(&a) == XFER_OPEN_DEFERRED, "retransmit same open is idempotent");
}

static void test_offset_past_eof(void) {
  printf("  test_offset_past_eof\n");
  fake_reset();
  for (uint32_t i = 0; i < 100; i++)
    FAKE.buf[i] = (uint8_t)i;
  FAKE.size = 100;

  /* Open a download starting at offset 100 == total -> immediate DONE. */
  xfer_open_args_t dn = mkargs(0, XFER_DIR_DOWNLOAD, 7, 100);
  xfer_on_open(&dn);
  xfer_tick(0, 0, 4);
  bool eof0 = false;
  for (int i = 0; i < CAP.n_data; i++)
    if ((CAP.data[i].flags & XFER_F_EOF) && CAP.data[i].len == 0)
      eof0 = true;
  CHECK(eof0, "offset-past-EOF emits a zero-length EOF immediately");
  CHECK(!xfer_session_active(0) ||
            xfer_session_get(0)->state == XFER_ST_DONE_LINGER,
        "session went terminal");
}

static void test_open_error_failed(void) {
  printf("  test_open_error_failed\n");
  fake_reset();
  FAKE.open_rc = -42; /* provider->open fails */

  xfer_open_args_t dn = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  CHECK(xfer_on_open(&dn) == XFER_OPEN_DEFERRED, "open still deferred (error is async)");
  xfer_tick(0, 0, 4);
  CHECK(CAP.n_cmd_ack == 1 && CAP.cmd_ack[0].result == XFER_RES_FAILED,
        "provider open error -> COMMAND_ACK FAILED");
  CHECK(CAP.cmd_ack[0].param2 == -42, "error sub-code carried in result_param2");
  CHECK(!xfer_session_active(0), "failed-open session freed");
}

static void test_close_acks_and_frees(void) {
  printf("  test_close_acks_and_frees\n");
  fake_reset();
  xfer_open_args_t dn = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  xfer_on_open(&dn);
  xfer_tick(0, 0, 1);
  cap_reset();
  int closes0 = FAKE.closes;
  CHECK(xfer_on_close(0, 0x77, XFER_RES_ACCEPTED) == XFER_OPEN_DEFERRED,
        "close deferred to tick");
  xfer_tick(5, 0, 1);
  CHECK(CAP.n_cmd_ack == 1 && CAP.cmd_ack[0].msgid == XFER_WIRE_MSGID_CLOSE &&
            CAP.cmd_ack[0].req_seq == 0x77,
        "close COMMAND_ACK emitted for XFER_CLOSE/req_seq");
  CHECK(FAKE.closes == closes0 + 1, "provider close called");
  CHECK(!xfer_session_active(0), "session freed after close");
}

/* Hardening: a vanished GCS mid-upload is reaped by the idle timeout (the FC
 * keeps acking but cursor stalls; with no fresh chunks last_rx ages out). */
static void test_idle_timeout_reaps_stalled_upload(void) {
  printf("  test_idle_timeout_reaps_stalled_upload\n");
  fake_reset();
  int closes0 = FAKE.closes;
  xfer_open_args_t up = mkargs(0, XFER_DIR_UPLOAD, 7, 0);
  xfer_on_open(&up);
  xfer_tick(0, 0, 4);
  uint8_t blk[100];
  memset(blk, 0x33, sizeof blk);
  xfer_on_data(0, 0, blk, sizeof blk, XFER_F_NONE); /* one chunk, then silence */
  xfer_tick(10, 0, 4);                              /* refreshes liveness @10 */
  CHECK(xfer_session_active(0), "upload alive while chunks arrive");
  /* No more chunks; tick well past the 5 s idle bound. */
  xfer_tick(10 + 4000, 0, 4);
  CHECK(xfer_session_active(0), "still alive before the idle bound");
  xfer_tick(10 + 6000, 0, 4);
  CHECK(!xfer_session_active(0), "stalled upload reaped after idle timeout");
  CHECK(FAKE.closes == closes0 + 1, "provider close called on idle reap");
}

/* Hardening: two concurrent downloads share the chunk budget (round-robin), so
 * neither slot starves the other. */
static void test_concurrent_budget_fairness(void) {
  printf("  test_concurrent_budget_fairness\n");
  fake_reset();
  for (uint32_t i = 0; i < 2000; i++)
    FAKE.buf[i] = (uint8_t)i;
  FAKE.size = 2000;

  xfer_open_args_t a = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  xfer_open_args_t b = mkargs(1, XFER_DIR_DOWNLOAD, 7, 0);
  xfer_on_open(&a);
  xfer_on_open(&b);
  /* Budget 2/tick shared across both sessions; ack both so they keep flowing. */
  for (int t = 0; t < 6; t++) {
    xfer_tick(10 + (uint32_t)t, 0, 2);
    xfer_on_ack(0, 2000, XFER_F_NONE); /* keep them ACTIVE (don't finish early) */
    xfer_on_ack(1, 2000, XFER_F_NONE);
  }
  CHECK(CAP.data_by_session[0] > 0 && CAP.data_by_session[1] > 0,
        "both concurrent downloads emitted (neither starved)");
  int d = CAP.data_by_session[0] - CAP.data_by_session[1];
  if (d < 0)
    d = -d;
  CHECK(d <= 1, "round-robin keeps the two within one chunk of each other");
}

static void test_download_active_flag(void) {
  printf("  test_download_active_flag\n");
  fake_reset();
  FAKE.size = 500;
  CHECK(!xfer_download_active(), "no download active initially");
  xfer_open_args_t dn = mkargs(0, XFER_DIR_DOWNLOAD, 7, 0);
  xfer_on_open(&dn);
  xfer_tick(0, 0, 1); /* -> ACTIVE file download */
  CHECK(xfer_download_active(), "file download flips xfer_download_active()");
  /* An upload must NOT count as a download. */
  fake_reset();
  xfer_open_args_t up = mkargs(0, XFER_DIR_UPLOAD, 7, 0);
  xfer_on_open(&up);
  xfer_tick(0, 0, 1);
  CHECK(!xfer_download_active(), "upload does not set download-active");
}

int main(void) {
  printf("== xfer SM SITL verification ==\n");
  xfer_init(&TX);
  xfer_register_provider(&FAKE_PROV);
  xfer_register_provider(&RDONLY_PROV);

  test_roundtrip_up_then_down();
  test_resume_after_drop();
  test_out_of_order_upload();
  test_rejections();
  test_offset_past_eof();
  test_open_error_failed();
  test_close_acks_and_frees();
  test_idle_timeout_reaps_stalled_upload();
  test_concurrent_budget_fairness();
  test_download_active_flag();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
