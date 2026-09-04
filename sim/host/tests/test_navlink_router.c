/**
 * @file sim/host/tests/test_navlink_router.c
 * @brief RX seam (firmware/src/comm/navlink_router.c) — dispatch + the §10.5 gate.
 *
 * navlink_router.c is NOT generated; the generated half is the codec
 * (navlink/generated/c/navlink_msgs.c). This covers the hand-written half: the
 * handler table, and above all the §10.5 security gate — an unsynchronised FC
 * MUST reject every command (router_command_gate -> ACK_BUSY until
 * time_sync_is_synced()).
 *
 * That gate has bitten before: a harness that skipped TIME_SYNC had every
 * command silently rejected and its sweeps came back inert. This pins it.
 *
 * Frames are built with the generated *_encode() helpers (real CRC) and pushed
 * through comm_rx_raw_inject(), the sim-only RX seam, so no pty or sleep is
 * involved and the test is deterministic.
 */
#include <stdio.h>
#include <string.h>

#include "comm/navlink_router.h"
#include "comm/serializer.h"
#include "control/flight_mode.h"
#include "sys/sys_utils.h"
#include "navlink_msgs.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

static void feed(const uint8_t *frame, size_t n) {
  comm_rx_raw_inject(frame, (uint16_t)n);
  navlink_router_poll();
  /* A GCS mode command sets an OVERRIDE; the effective mode is only promoted
   * when the RC task next resolves (flight_mode.h: "last resolved effective
   * mode"). Resolve here so the assertions see the settled state. */
  (void)flight_mode_resolve_acro(false);
}

static size_t build_set_mode(uint8_t *buf, uint8_t mode) {
  navlink_cmd_set_flight_mode_t m = {0};
  m.target_sys = 1;
  m.target_comp = 1;
  m.req_seq = 7;
  m.mode = mode;
  m.source = 1; /* GCS */
  return navlink_cmd_set_flight_mode_encode(buf, &m, 0, 255, 1);
}

int main(void) {
  printf("test_navlink_router: RX dispatch + the 10.5 command gate\n");
  uint8_t frame[128];

  navlink_router_init();

  printf("  [1] an unsynchronised FC rejects commands (10.5 gate)\n");
  {
    CHECK(time_sync_is_synced() == 0, "starts unsynchronised");
    flight_mode_t before = flight_mode_get();
    size_t n = build_set_mode(frame, (uint8_t)(before == FLIGHT_MODE_ANGLE
                                                   ? FLIGHT_MODE_ACRO
                                                   : FLIGHT_MODE_ANGLE));
    CHECK(n > 0, "CMD_SET_FLIGHT_MODE encoded");
    feed(frame, n);
    CHECK(flight_mode_get() == before,
          "command did NOT take effect while unsynced");
    CHECK(flight_mode_get_source() == FLIGHT_MODE_SRC_RC,
          "no GCS override was recorded while unsynced");
  }

  printf("  [2] a TIME_SYNC frame clears the gate\n");
  {
    navlink_time_sync_t ts = {0};
    ts.role = 0;   /* GCS request */
    ts.seq = 1;
    ts.t1_gcs_tx = 1000000ull;
    size_t n = navlink_time_sync_encode(frame, &ts, 0, 255, 1);
    CHECK(n > 0, "TIME_SYNC encoded");
    feed(frame, n);
    CHECK(time_sync_is_synced() != 0, "FC reports synchronised");
  }

  printf("  [3] once synced, the same command is applied\n");
  {
    flight_mode_t before = flight_mode_get();
    flight_mode_t want = (before == FLIGHT_MODE_ANGLE) ? FLIGHT_MODE_ACRO
                                                           : FLIGHT_MODE_ANGLE;
    size_t n = build_set_mode(frame, (uint8_t)want);
    feed(frame, n);
    CHECK(flight_mode_get() == want, "flight mode changed after sync");
    CHECK(flight_mode_get_source() == FLIGHT_MODE_SRC_GCS,
          "source is now the GCS override");
  }

  printf("  [4] a corrupted frame is dropped, not dispatched\n");
  {
    flight_mode_t before = flight_mode_get();
    flight_mode_t other = (before == FLIGHT_MODE_ANGLE) ? FLIGHT_MODE_ACRO
                                                            : FLIGHT_MODE_ANGLE;
    size_t n = build_set_mode(frame, (uint8_t)other);
    frame[n - 1] ^= 0xFFu; /* wreck the CRC */
    feed(frame, n);
    CHECK(flight_mode_get() == before, "bad-CRC frame had no effect");
  }

  printf("  [5] leading link noise does not wedge the parser\n");
  {
    /* Garbage with NO sync byte in it — the realistic case of line noise ahead
     * of a frame. The parser must skip it and pick up the next real frame.
     *
     * Deliberately NOT asserted: junk that happens to contain 0x56 followed by
     * a large length byte makes the parser wait for that many payload bytes and
     * swallow the frame that follows. That is inherent to a length-prefixed
     * protocol (it recovers on the frame after), not a defect, so pinning it as
     * a guarantee would be asserting an accident. */
    const uint8_t junk[] = {0x00, 0xFF, 0x11, 0x22, 0x33};
    feed(junk, sizeof junk);
    flight_mode_t want = (flight_mode_get() == FLIGHT_MODE_ANGLE)
                             ? FLIGHT_MODE_ACRO
                             : FLIGHT_MODE_ANGLE;
    size_t n = build_set_mode(frame, (uint8_t)want);
    feed(frame, n);
    CHECK(flight_mode_get() == want, "parser resynced after leading noise");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
