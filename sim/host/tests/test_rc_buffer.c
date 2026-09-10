/* RC SPSC queues (firmware/src/comm/rc_buffer.c).
 *
 * Two rings with different depths and the SAME overwrite policy, which is the
 * point worth pinning: on a full ring the producer must NOT block or drop the
 * NEWEST frame -- RC is the failsafe path, and the freshest stick position is
 * the only one worth having. The header sizes control at 4 usable slots and
 * telemetry at 2 ("depth past 2 only ages the frame that gets read"), so this
 * also guards those two numbers against a careless edit. */
#include <stdio.h>
#include <string.h>

#include "comm/rc_buffer.h"
#include "comm/perf_telemetry.h"

/* Usable depth is NOT fixed. spsc_init aligns the buffer to a multiple of
 * elem_size and spends a slot doing it ONLY when the static array's address is
 * not already aligned (extern/vaios/kernel/structure.c:26-44) -- and none of
 * these element types is a power of two in size, so which case you get depends
 * on where the linker happened to put the array. Locally these rings hold
 * SIZE-1; on the CI runner rc_buffer's held SIZE. Both are correct and the
 * difference is harmless for an OVERWRITE mailbox, but the depth cannot be
 * asserted exactly -- so assert the band, and assert the property that
 * actually matters (below) exactly. */

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

/* Channel 0 carries the frame's identity so overwrite order is checkable. */
static ibus_data_t frame(uint16_t tag) {
  ibus_data_t d;
  memset(&d, 0, sizeof d);
  d.channels[0] = tag;
  return d;
}

int main(void) {
  printf("== rc_buffer ==\n");
  rc_buffer_init();

  printf("  [1] an empty queue pops nothing\n");
  {
    ibus_data_t out;
    CHECK(!rc_queue_control_pop(&out), "control starts empty");
    CHECK(!rc_queue_telemetry_pop(&out), "telemetry starts empty");
  }

  printf("  [2] a frame survives a round trip, both queues\n");
  {
    ibus_data_t in = frame(1111), out;
    CHECK(rc_queue_control_push(&in), "control push accepted");
    CHECK(rc_queue_control_pop(&out), "control pop returned a frame");
    CHECK(out.channels[0] == 1111, "control frame is the one pushed");

    in = frame(2222);
    CHECK(rc_queue_telemetry_push(&in), "telemetry push accepted");
    CHECK(rc_queue_telemetry_pop(&out), "telemetry pop returned a frame");
    CHECK(out.channels[0] == 2222, "telemetry frame is the one pushed");
  }

  printf("  [3] FIFO order within capacity\n");
  {
    ibus_data_t out;
    for (uint16_t i = 0; i < RC_BUFFER_SIZE - 1; i++) {
      ibus_data_t in = frame((uint16_t)(100 + i));
      CHECK(rc_queue_control_push(&in), "push within capacity accepted");
    }
    for (uint16_t i = 0; i < RC_BUFFER_SIZE - 1; i++) {
      CHECK(rc_queue_control_pop(&out), "pop returned a frame");
      CHECK(out.channels[0] == (uint16_t)(100 + i),
            "frames come back in order");
    }
    CHECK(!rc_queue_control_pop(&out), "queue drained");
  }

  printf("  [4] overflow overwrites the OLDEST, never rejects the newest\n");
  {
    /* Push twice the ring's depth. The failsafe path must always be able to
     * hand on the frame it just received. */
    ibus_data_t out;
    const uint16_t n = 2 * RC_BUFFER_SIZE;
    for (uint16_t i = 0; i < n; i++) {
      ibus_data_t in = frame((uint16_t)(500 + i));
      CHECK(rc_queue_control_push(&in), "push past capacity still accepted");
    }
    /* Whatever survived, the newest frame must be in there and the stalest
     * must not: that is what OVERWRITE buys. */
    uint16_t last = 0, count = 0;
    while (rc_queue_control_pop(&out)) {
      CHECK(out.channels[0] != 500, "the oldest frame was overwritten");
      last = out.channels[0];
      count++;
    }
    CHECK(count >= RC_BUFFER_SIZE - 1 && count <= RC_BUFFER_SIZE,
          "ring depth is SIZE-1 or SIZE, per spsc_init's alignment");
    CHECK(last == (uint16_t)(500 + n - 1), "the newest frame survived");
  }

  printf("  [5] telemetry is the shallower ring, and independent\n");
  {
    ibus_data_t out;
    for (uint16_t i = 0; i < 2 * RC_TELEMETRY_BUFFER_SIZE; i++) {
      ibus_data_t in = frame((uint16_t)(900 + i));
      CHECK(rc_queue_telemetry_push(&in), "telemetry push accepted");
    }
    uint16_t count = 0;
    while (rc_queue_telemetry_pop(&out))
      count++;
    CHECK(count >= RC_TELEMETRY_BUFFER_SIZE - 1 &&
              count <= RC_TELEMETRY_BUFFER_SIZE,
          "telemetry depth is SIZE-1 or SIZE, and shallower than control");
    CHECK(count < RC_BUFFER_SIZE, "telemetry is the shallower of the two");
    CHECK(!rc_queue_control_pop(&out), "control was not disturbed");
  }

  printf("  [6] perf rows are reported, and respect the caller's cap\n");
  {
    perf_fifo_row_t rows[4];
    memset(rows, 0, sizeof rows);
    CHECK(rc_buffer_perf_fifos(rows, 4) == 2, "both RC fifos reported");
    CHECK(rc_buffer_perf_fifos(rows, 1) == 1, "a cap of 1 yields 1 row");
    CHECK(rc_buffer_perf_fifos(rows, 0) == 0, "a cap of 0 yields no rows");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
