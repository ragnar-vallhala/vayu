/* Bench check: SD/FatFS write -> read-back integrity through the FS owner. */
#include "hwtest_runner.h"
#include "storage/fs_owner.h"
#include "task.h" /* v_delay */

#include <string.h>

#define SD_TEST_PATH "0:hwtest_sd.bin"

/* Write a known 16-byte pattern via the FS-owner write-at lane, then read it
 * back (synchronous VFS pass-through) and compare. Exercises the SD/FatFS write
 * + sync + read path end to end.
 * @verifies LOG-SD-001, LOG-OWN-001 */
hw_result_t check_sd_readback(void) {
  static const uint8_t pat[16] = {0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0xDE, 0xAD,
                                  0xBE, 0xEF, 0x10, 0x20, 0x30, 0x40, 0xFF, 0x00};
  fs_owner_writeat_reset(0);
  fs_owner_truncate(SD_TEST_PATH);
  if (!fs_owner_enqueue_write_at(0, SD_TEST_PATH, 0, pat, sizeof pat)) {
    return hw_fail(0.0f, "wq");
  }
  for (int i = 0; i < 100 && fs_owner_writeat_pending(0); i++) {
    v_delay(20);
  }
  if (fs_owner_writeat_failed(0)) {
    return hw_fail(0.0f, "wr");
  }
  uint8_t rb[16] = {0};
  int n = fs_owner_read_at(SD_TEST_PATH, 0, rb, sizeof rb);
  if (n != (int)sizeof rb) {
    return hw_fail((float)n, "rd");
  }
  return memcmp(rb, pat, sizeof pat) == 0 ? hw_pass(1.0f, "ok")
                                          : hw_fail(0.0f, "cmp");
}
