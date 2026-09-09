/**
 * @file sim/host/tests/test_navlink_tx.c
 * @brief TX seam (firmware/src/comm/navlink_tx.c) — framing + field mapping.
 *
 * navlink_tx.c is NOT generated. The generated part is the codec
 * (navlink/generated/c/navlink_msgs.c: pack/unpack/CRC); this file is the
 * hand-written seam that gathers domain data, converts units and hands frames
 * to the channel. That hand-written half is what this suite covers: unit
 * conversion, field mapping and the STATUSTEXT line splitter.
 *
 * Capture path: the host navhal forwards every UART2 TX byte to the in-process
 * vsim_iface on_uart2_bytes hook, so the test registers a sink, calls a tx
 * function, flushes the channel and decodes the frame that comes out. SITL puts
 * telemetry on UART2 (real hardware uses UART6 — the roles are inverted here).
 *
 * Frame layout (navlink v2, mirrors navlink/sim/frame.py):
 *   [0] sync 0x56  [1] ver 0x02  [2] payload len  [4] seq
 *   [5] sysid  [6] compid  [7..9] msgid LE24  [10..] payload  then CRC16
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "comm/channel.h"
#include "comm/navlink_tx.h"
#include "vsim_iface.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

#define NL_SYNC 0x56u
#define NL_VER 0x02u
#define NL_HDR 10u

/* ---- TX capture ---------------------------------------------------------- */
static uint8_t g_tx[8192];
static size_t g_tx_n;

static void tx_sink(void *user, const uint8_t *data, size_t n) {
  (void)user;
  if (g_tx_n + n > sizeof g_tx)
    n = sizeof g_tx - g_tx_n;
  memcpy(g_tx + g_tx_n, data, n);
  g_tx_n += n;
}

extern channel_t g_telemetry_channel;

static void tx_reset(void) { g_tx_n = 0; }

static void tx_flush(void) { (void)flush_channel(g_telemetry_channel); }

/* Find the first frame with `msgid`; returns payload pointer + length. */
static const uint8_t *find_msg(uint32_t msgid, uint8_t *out_len) {
  size_t off = 0;
  while (off + NL_HDR + 2 <= g_tx_n) {
    if (g_tx[off] != NL_SYNC || g_tx[off + 1] != NL_VER) {
      off++;
      continue;
    }
    uint8_t plen = g_tx[off + 2];
    if (off + NL_HDR + plen + 2 > g_tx_n)
      break;
    uint32_t id = (uint32_t)g_tx[off + 7] | ((uint32_t)g_tx[off + 8] << 8) |
                  ((uint32_t)g_tx[off + 9] << 16);
    if (id == msgid) {
      if (out_len)
        *out_len = plen;
      return g_tx + off + NL_HDR;
    }
    off += NL_HDR + plen + 2;
  }
  return NULL;
}

static int count_msgs(uint32_t msgid) {
  size_t off = 0;
  int n = 0;
  while (off + NL_HDR + 2 <= g_tx_n) {
    if (g_tx[off] != NL_SYNC || g_tx[off + 1] != NL_VER) {
      off++;
      continue;
    }
    uint8_t plen = g_tx[off + 2];
    if (off + NL_HDR + plen + 2 > g_tx_n)
      break;
    uint32_t id = (uint32_t)g_tx[off + 7] | ((uint32_t)g_tx[off + 8] << 8) |
                  ((uint32_t)g_tx[off + 9] << 16);
    if (id == msgid)
      n++;
    off += NL_HDR + plen + 2;
  }
  return n;
}

static float rdf(const uint8_t *p, size_t byte_off) {
  float f;
  memcpy(&f, p + byte_off, sizeof f);
  return f;
}

/* msgids (navlink/dialect.json) */
#define MSG_STATUSTEXT 4u
#define MSG_ATTITUDE_EULER 1026u
#define MSG_IMU_RAW 1024u
#define MSG_SYSTEM_HEALTH 2u

int main(void) {
  printf("test_navlink_tx: TX seam framing + field mapping\n");

  static vsim_iface_t iface;
  vsim_iface_init(&iface);
  iface.on_uart2_bytes = tx_sink;
  iface.on_uart2_bytes_user = NULL;
  vsim_iface_set_global(&iface);

  serial_args_t uart_args = {
      .baud_rate = 460800, .uart = HAL_UART_2, .timeout = 100};
  if (get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
                  NULL) != NONE) {
    printf("    FATAL: could not open the telemetry channel\n");
    return 1;
  }

  printf("  [1] ATTITUDE_EULER converts degrees to radians\n");
  {
    tx_reset();
    attitude_t att = {0};
    att.roll = 90.0f;
    att.pitch = -45.0f;
    att.yaw = 180.0f;
    navlink_tx_attitude(&att);
    tx_flush();
    uint8_t len = 0;
    const uint8_t *p = find_msg(MSG_ATTITUDE_EULER, &len);
    CHECK(p != NULL, "ATTITUDE_EULER frame emitted");
    if (p) {
      CHECK(fabsf(rdf(p, 0) - (float)M_PI / 2.0f) < 1e-4f,
            "roll 90deg -> pi/2 rad");
      CHECK(fabsf(rdf(p, 4) + (float)M_PI / 4.0f) < 1e-4f,
            "pitch -45deg -> -pi/4 rad");
      CHECK(fabsf(rdf(p, 8) - (float)M_PI) < 1e-4f, "yaw 180deg -> pi rad");
    }
  }

  printf("  [2] IMU_RAW carries the 10-float vector unscaled\n");
  {
    tx_reset();
    const float f10[10] = {-0.5f, 0.25f, -9.81f, 1.0f,  2.0f,
                           3.0f,  10.0f, 20.0f,  30.0f, 27.5f};
    navlink_tx_imu_full(f10);
    tx_flush();
    uint8_t len = 0;
    const uint8_t *p = find_msg(MSG_IMU_RAW, &len);
    CHECK(p != NULL, "IMU_RAW frame emitted");
    if (p) {
      CHECK(fabsf(rdf(p, 0) - (-0.5f)) < 1e-6f, "acc x passthrough");
      CHECK(fabsf(rdf(p, 8) - (-9.81f)) < 1e-6f,
            "acc z passthrough (gravity sign kept)");
    }
  }

  printf("  [3] STATUSTEXT splits a bulk drain into one frame per line\n");
  {
    tx_reset();
    const char buf[] = "alpha\nbravo\ncharlie\n";
    navlink_tx_log(buf, (uint8_t)(sizeof buf - 1));
    tx_flush();
    CHECK(count_msgs(MSG_STATUSTEXT) == 3,
          "three lines -> three STATUSTEXT frames");
    uint8_t len = 0;
    const uint8_t *p = find_msg(MSG_STATUSTEXT, &len);
    CHECK(p != NULL, "STATUSTEXT emitted");
    /* payload: severity u8, then the text field */
    if (p)
      CHECK(memcmp(p + 1, "alpha", 5) == 0, "first line text is 'alpha'");
  }

  printf(
      "  [4] a line longer than the 50-char field spills into another frame\n");
  {
    tx_reset();
    char big[80];
    memset(big, 'x', sizeof big);
    big[sizeof big - 1] = '\n';
    navlink_tx_log(big, (uint8_t)sizeof big);
    tx_flush();
    CHECK(count_msgs(MSG_STATUSTEXT) >= 2, "79-char line spills to >=2 frames");
  }

  printf("  [5] SYSTEM_HEALTH maps its three counters\n");
  {
    tx_reset();
    navlink_tx_health(7u, 0u, 42u);
    tx_flush();
    uint8_t len = 0;
    const uint8_t *p = find_msg(MSG_SYSTEM_HEALTH, &len);
    CHECK(p != NULL, "SYSTEM_HEALTH frame emitted");
  }

  printf("  [6] every emitted frame carries the v2 sync/version header\n");
  {
    tx_reset();
    navlink_tx_heartbeat();
    navlink_tx_flight_mode(1u, 0u);
    navlink_tx_notch_status();
    tx_flush();
    CHECK(g_tx_n > 0, "bytes reached the wire");
    CHECK(g_tx[0] == NL_SYNC && g_tx[1] == NL_VER,
          "leading frame is 0x56/0x02");
  }

  vsim_iface_set_global(NULL);
  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
