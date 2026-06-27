/* On-hardware bench runner — emits results over the NavLink test dialect
 * (HW_TEST_*, msgid 0x800000+, present only in this test image) and mirrors a
 * human line to vayu_log (which persists to the SD text log). C1 skeleton. */
#include "hwtest_runner.h"

#include "comm/channel.h"   /* channel_t, write_channel */
#include "storage/fs_owner.h" /* vayu_log */
#include "sys/sys_utils.h"  /* get_device_id */

#include "navlink_msgs.h"   /* test-profile codec (has HW_TEST_*) */

#include <string.h>

/* Defined in telemetry setup (init_sensors); the UART6 telemetry channel. */
extern channel_t g_telemetry_channel;

static uint8_t s_seq = 0;

static void send(const uint8_t *frame, uint16_t n) {
  write_channel(g_telemetry_channel, (byte *)frame, n);
}

static void emit_begin(uint16_t total) {
  navlink_hw_test_begin_t m = {0};
  m.total = total;
  uint8_t fr[NAVLINK_MAX_FRAME];
  size_t n = navlink_hw_test_begin_encode(fr, &m, s_seq++, get_device_id(), 1);
  send(fr, (uint16_t)n);
}

static void emit_result(uint16_t id, const hw_check_t *c, hw_result_t r) {
  navlink_hw_test_result_t m = {0};
  m.id = id;
  m.pass = (uint8_t)r.status;
  m.value = r.value;
  /* char[] fields are fixed-width; strncpy zero-pads, no NUL needed on the wire. */
  strncpy(m.name, c->name, sizeof m.name);
  strncpy(m.units, r.units ? r.units : "", sizeof m.units);
  uint8_t fr[NAVLINK_MAX_FRAME];
  size_t n = navlink_hw_test_result_encode(fr, &m, s_seq++, get_device_id(), 1);
  send(fr, (uint16_t)n);
  vayu_log("[HWTEST] %-20s %s (%d)", c->name,
           r.status == HW_PASS ? "ok"
           : r.status == HW_SKIP ? "skip"
                                 : "FAIL",
           (int)r.value);
}

static void emit_done(uint16_t passed, uint16_t failed, uint16_t skipped) {
  navlink_hw_test_done_t m = {0};
  m.passed = passed;
  m.failed = failed;
  m.skipped = skipped;
  uint8_t fr[NAVLINK_MAX_FRAME];
  size_t n = navlink_hw_test_done_encode(fr, &m, s_seq++, get_device_id(), 1);
  send(fr, (uint16_t)n);
  vayu_log("[HWTEST] done: %d passed, %d failed, %d skipped", passed, failed,
           skipped);
}

void hwtest_run_all(void) {
  uint16_t total = 0;
  while (hwtest_registry[total].fn) {
    total++;
  }
  emit_begin(total);

  uint16_t passed = 0, failed = 0, skipped = 0;
  for (uint16_t i = 0; i < total; i++) {
    hw_result_t r = hwtest_registry[i].fn();
    emit_result(i, &hwtest_registry[i], r);
    if (r.status == HW_PASS) {
      passed++;
    } else if (r.status == HW_SKIP) {
      skipped++;
    } else {
      failed++;
    }
  }
  emit_done(passed, failed, skipped);
}
