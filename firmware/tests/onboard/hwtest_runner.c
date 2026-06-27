/* On-hardware bench runner. Runs the check registry once, writes a text report
 * to 0:hwtest.txt on the SD card (durable, downloadable over the xfer
 * substrate), and emits a single HW_TEST_DONE status frame over NavLink. Kept
 * deliberately light on the radio: the detailed results live on SD, not the
 * link. C1. */
#include "hwtest_runner.h"

#include "comm/channel.h"     /* channel_t, write_channel */
#include "storage/fs_owner.h" /* fs_owner_enqueue_write_at, *_writeat_* */
#include "sys/sys_utils.h"    /* get_device_id */
#include "task.h"             /* v_delay */

#include "navlink_msgs.h" /* test-profile codec (has HW_TEST_*) */
#include "utils.h"        /* vaprint_fmt_buf (bare-metal printf, no newlib stdio) */

#include <stdarg.h> /* va_list */

extern channel_t g_telemetry_channel; /* UART6 telemetry channel */

#define HWTEST_REPORT_PATH "0:hwtest.txt"
#define HWTEST_REPORT_MAX 1024u
#define HWTEST_WRITE_CHUNK 200u

static char s_report[HWTEST_REPORT_MAX];
static uint32_t s_report_len;
static uint8_t s_seq;

static void report_line(const char *fmt, ...) {
  if (s_report_len >= HWTEST_REPORT_MAX - 1) {
    return;
  }
  va_list ap;
  va_start(ap, fmt);
  int n = vaprint_fmt_buf(s_report + s_report_len,
                          HWTEST_REPORT_MAX - s_report_len, fmt, ap);
  va_end(ap);
  if (n > 0) {
    s_report_len += (uint32_t)n;
  }
}

/* Persist the accumulated report to SD via the FS owner's write-at lane
 * (session 0 = non-xfer caller). Truncate, write in chunks, wait for commit. */
static void write_report_sd(void) {
  fs_owner_writeat_reset(0);
  if (fs_owner_truncate(HWTEST_REPORT_PATH) != 0) {
    return;
  }
  uint32_t off = 0;
  while (off < s_report_len) {
    uint32_t n = s_report_len - off;
    if (n > HWTEST_WRITE_CHUNK) {
      n = HWTEST_WRITE_CHUNK;
    }
    fs_owner_enqueue_write_at(0, HWTEST_REPORT_PATH, off, s_report + off, n);
    off += n;
    v_delay(20); /* let fs_owner_task drain this chunk */
  }
  for (int i = 0; i < 100 && fs_owner_writeat_pending(0); i++) {
    v_delay(20);
  }
}

static void emit_done(uint16_t passed, uint16_t failed, uint16_t skipped) {
  navlink_hw_test_done_t m = {0};
  m.passed = passed;
  m.failed = failed;
  m.skipped = skipped;
  uint8_t fr[NAVLINK_MAX_FRAME];
  size_t n = navlink_hw_test_done_encode(fr, &m, s_seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, (byte *)fr, (uint16_t)n);
}

void hwtest_run_all(void) {
  s_report_len = 0;
  uint16_t total = 0;
  while (hwtest_registry[total].fn) {
    total++;
  }
  /* Compact, single-chunk-sized report (<=247 B) — the FC's multi-chunk xfer
   * download path hangs the board (LED freeze; 1-chunk C1 file downloaded fine,
   * 2-chunk C2 file wedged it), so keep the whole report inside one XFER_DATA
   * chunk until that multi-chunk download bug is fixed. Format per check:
   * "name=value<P|F|S> ". */
  report_line("vayu bench %u:\n", total);
  write_report_sd(); /* mark start; rewritten after every check below */

  uint16_t passed = 0, failed = 0, skipped = 0;
  for (uint16_t i = 0; i < total; i++) {
    hw_result_t r = hwtest_registry[i].fn();
    char tag = r.status == HW_PASS ? 'P' : r.status == HW_SKIP ? 'S' : 'F';
    report_line("%s=%d%c ", hwtest_registry[i].name, (int)r.value, tag);
    if (r.status == HW_PASS) {
      passed++;
    } else if (r.status == HW_SKIP) {
      skipped++;
    } else {
      failed++;
    }
    /* Rewrite after every check so a check that wedges/crashes still leaves the
     * results up to that point on SD — the file names exactly where it stopped. */
    write_report_sd();
  }
  report_line("\nDONE %u/%u/%u\n", passed, failed, skipped);

  write_report_sd();
  emit_done(passed, failed, skipped); /* one-shot status over the link */
}
