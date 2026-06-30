/* math_utils_unit_test.c — host unit test for sys/math_utils.c.
 *
 * float32_to_float16: IEEE-754 half-precision packing used for telemetry
 * payload compression. Pure bit-twiddling — exact values are checkable.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/math_utils_unit_test.c \
 *       src/sys/math_utils.c -o /tmp/mu && /tmp/mu
 */
#include "sys/math_utils.h"

#include <stdio.h>

static int fails = 0;
static void eq(const char *what, uint16_t got, uint16_t want) {
  int ok = got == want;
  printf("  [%s] %s (got 0x%04X want 0x%04X)\n", ok ? "PASS" : "FAIL", what, got,
         want);
  if (!ok) fails++;
}

int main(void) {
  printf("float32_to_float16:\n");
  eq("+0.0 -> 0x0000", float32_to_float16(0.0f), 0x0000);
  eq("-0.0 -> 0x8000 (sign)", float32_to_float16(-0.0f), 0x8000);
  eq("1.0 -> 0x3C00", float32_to_float16(1.0f), 0x3C00);
  eq("2.0 -> 0x4000", float32_to_float16(2.0f), 0x4000);
  eq("0.5 -> 0x3800", float32_to_float16(0.5f), 0x3800);
  eq("-1.0 -> 0xBC00", float32_to_float16(-1.0f), 0xBC00);
  eq("1e30 overflow -> +inf 0x7C00", float32_to_float16(1e30f), 0x7C00);
  eq("-1e30 overflow -> -inf 0xFC00", float32_to_float16(-1e30f), 0xFC00);
  eq("1e-10 underflow -> 0x0000", float32_to_float16(1e-10f), 0x0000);
  eq("+inf -> 0x7C00", float32_to_float16(1.0f / 0.0f), 0x7C00);
  /* NaN: exp all-ones + nonzero mantissa; sign is preserved (platform-dependent
   * which NaN 0.0/0.0 yields), so check the pattern, not an exact word. */
  {
    uint16_t h = float32_to_float16(0.0f / 0.0f);
    int is_nan = (h & 0x7C00) == 0x7C00 && (h & 0x03FF) != 0;
    printf("  [%s] NaN -> half-NaN pattern (got 0x%04X)\n",
           is_nan ? "PASS" : "FAIL", h);
    if (!is_nan) fails++;
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
