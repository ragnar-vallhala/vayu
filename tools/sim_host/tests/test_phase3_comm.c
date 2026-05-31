/**
 * @file tools/sim_host/tests/test_phase3_comm.c
 * @brief SITL verification suite for the Phase-3 COMM cluster.
 *
 * Links the real firmware sources (libvayu_sitl_core) and drives their
 * public contracts directly.
 *
 *   @verifies COMM-CH-002   TX-buffer overflow counter
 *   @verifies COMM-CMD-002  command payload length / argc validation
 *   @verifies COMM-CMD-003  CMD_SET_PID apply + SD persistence
 *
 * The GCS software-arm path is exercised end-to-end at the wire level: the
 * exact CMD_ARM / CMD_DISARM frame the Navigator emits (MainWindow::
 * onArmClicked) is run through deserializer_feed() and comm_processor_
 * dispatch(), confirming it sets/clears g_sw_arm_request and that a bad-CRC
 * frame is dropped.
 *
 * The CMD_SET_PID path is exercised end-to-end: a built payload is run
 * through pid_config_apply_command(), which performs the COMM-CMD-002
 * validation, applies the gains to the live rate controller (read back
 * via angle_rate_controller_get_gains), and persists them through the
 * VFS (RAM-backed host shim) — verified by reloading with
 * pid_config_init() and reading the value back from the store.
 *
 * COMM-TEL-002 (heartbeat cadence) is verified by inspection of
 * telemetry_task.c (166 ticks x 6 ms = 996 ms >= 1 Hz).
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "comm/comm.h"
#include "comm/deserializer.h"
#include "comm/ibus.h"
#include "control/control.h"
#include "sys/sys_utils.h"
#include "vayu_status.h"
#include "vayu_tasks.h"

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond) {                                                                \
      printf("    ok   %s\n", (msg));                                          \
    } else {                                                                   \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

/* Build a CMD_SET_PID payload: [cmd_id:2][argc:1][6 x 4-byte float args].
 * Returns the populated payload length. */
static uint16_t build_set_pid(uint8_t *p, uint8_t argc, float ctrl, float axis,
                              float kp, float ki, float kd, float kff) {
  uint16_t cmd = (uint16_t)CMD_SET_PID;
  memcpy(&p[0], &cmd, 2);
  p[2] = argc;
  float args[6] = {ctrl, axis, kp, ki, kd, kff};
  for (int i = 0; i < 6; i++) {
    memcpy(&p[3 + i * 4], &args[i], 4);
  }
  return (uint16_t)(3 + 6 * 4); /* 27 */
}

static bool feq(float a, float b) {
  float d = a - b;
  if (d < 0) d = -d;
  return d < 1e-6f;
}

/* Build a no-arg NavLink command frame byte-for-byte as the Navigator GCS
 * does in MainWindow::onArmClicked: sync, protocol|type, length=2 (cmd_id
 * only), dev_id, 4-byte timestamp, 2-byte cmd_id, then a CRC32 over the
 * header+payload (everything preceding the CRC). Returns the frame length. */
static uint16_t build_command_frame(uint8_t *out, uint16_t cmd_id) {
  out[0] = SYNC_BYTE;                  /* 0x56 */
  out[1] = (PACKET_TYPE_COMMAND << 4); /* type 3 in high nibble, proto v0 */
  out[2] = 2;                          /* payload length = cmd_id only */
  out[3] = 42;                         /* dev_id */
  uint32_t ts = 0x12345678u;
  memcpy(&out[4], &ts, 4);
  memcpy(&out[8], &cmd_id, 2);
  /* CRC over header (8) + payload (2); same span the deserializer checks. */
  uint32_t crc = utils_try_compute_crc32(out, NAVLINK_HEADER_SIZE + 2);
  memcpy(&out[10], &crc, 4);
  return (uint16_t)(NAVLINK_HEADER_SIZE + 2 + NAVLINK_CRC_SIZE); /* 14 */
}

/* Feed a whole frame through the deserializer; return 1 iff it yields one
 * valid packet, copied into *pkt. */
static int feed_frame(deserializer_t *d, const uint8_t *frame, uint16_t n,
                      packet_t *pkt) {
  int got = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (deserializer_feed(d, frame[i]) == 1) {
      *pkt = d->packet;
      got = 1;
    }
  }
  return got;
}

/* ----------------------------------------------------------------------------
 * GCS software-arm wire path — the exact CMD_ARM / CMD_DISARM frame the
 * Navigator emits is deserialized and dispatched, flipping g_sw_arm_request.
 * This links MainWindow::onArmClicked -> deserializer -> comm_processor.
 * --------------------------------------------------------------------------*/
static void test_arm_command_wire(void) {
  printf("  test_arm_command_wire (GCS software-arm link)\n");

  deserializer_t d;
  deserializer_init(&d);
  packet_t pkt;
  uint8_t frame[64];

  /* CMD_ARM: a clean disarmed start, then the GCS arm frame sets the latch. */
  g_sw_arm_request = 0;
  uint16_t n = build_command_frame(frame, (uint16_t)CMD_ARM);
  CHECK(feed_frame(&d, frame, n, &pkt) == 1, "CMD_ARM frame deserializes");
  CHECK(((pkt.protocol_packet_type >> 4) & 0x0F) == PACKET_TYPE_COMMAND,
        "decoded packet type is COMMAND");
  CHECK(pkt.length == 2, "decoded payload length is 2 (cmd_id only)");
  uint16_t decoded_cmd = 0;
  memcpy(&decoded_cmd, pkt.payload, 2);
  CHECK(decoded_cmd == (uint16_t)CMD_ARM, "decoded cmd_id is CMD_ARM (0x0002)");
  comm_processor_dispatch(&pkt);
  CHECK(g_sw_arm_request == 1, "CMD_ARM dispatch sets the software-arm latch");

  /* CMD_DISARM clears it again. */
  n = build_command_frame(frame, (uint16_t)CMD_DISARM);
  CHECK(feed_frame(&d, frame, n, &pkt) == 1, "CMD_DISARM frame deserializes");
  decoded_cmd = 0;
  memcpy(&decoded_cmd, pkt.payload, 2);
  CHECK(decoded_cmd == (uint16_t)CMD_DISARM,
        "decoded cmd_id is CMD_DISARM (0x0003)");
  comm_processor_dispatch(&pkt);
  CHECK(g_sw_arm_request == 0, "CMD_DISARM dispatch clears the latch");

  /* A corrupted CRC must be rejected outright (no packet, latch untouched). */
  g_sw_arm_request = 0;
  n = build_command_frame(frame, (uint16_t)CMD_ARM);
  frame[n - 1] ^= 0xFF; /* clobber the top CRC byte */
  CHECK(feed_frame(&d, frame, n, &pkt) == 0, "bad-CRC frame is dropped");
  CHECK(g_sw_arm_request == 0, "dropped frame leaves the latch clear");
}

/* ----------------------------------------------------------------------------
 * COMM-CMD-002 — reject malformed / short command payloads
 * --------------------------------------------------------------------------*/
static void test_payload_validation(void) {
  printf("  test_payload_validation (COMM-CMD-002)\n");

  uint8_t p[64];
  uint16_t len = build_set_pid(p, PID_SET_ARGC, 1.f, 0.f, 0.1f, 0.f, 0.f, 0.f);

  CHECK(pid_config_apply_command(NULL, len) == VAYU_ERR_INVALID,
        "NULL payload rejected");
  CHECK(pid_config_apply_command(p, 2) == VAYU_ERR_INVALID,
        "payload too short for argc byte rejected");

  /* argc below the required count. */
  build_set_pid(p, 5, 1.f, 0.f, 0.1f, 0.f, 0.f, 0.f);
  CHECK(pid_config_apply_command(p, len) == VAYU_ERR_INVALID,
        "argc < PID_SET_ARGC rejected");

  /* argc claims 6 args but the framing delivered fewer bytes. */
  build_set_pid(p, PID_SET_ARGC, 1.f, 0.f, 0.1f, 0.f, 0.f, 0.f);
  CHECK(pid_config_apply_command(p, 23) == VAYU_ERR_INVALID,
        "payload length < argc*4+3 rejected");
}

/* ----------------------------------------------------------------------------
 * COMM-CMD-003 — CMD_SET_PID applies live and persists
 * --------------------------------------------------------------------------*/
static void test_set_pid_apply(void) {
  printf("  test_set_pid_apply (COMM-CMD-003)\n");

  const uint8_t axis = 1; /* pitch */
  const float kp = 0.111f, ki = 0.022f, kd = 0.033f, kff = 0.044f;

  uint8_t p[64];
  uint16_t len =
      build_set_pid(p, PID_SET_ARGC, (float)PID_CTRL_RATE, (float)axis, kp, ki,
                    kd, kff);

  CHECK(pid_config_apply_command(p, len) == VAYU_OK, "valid SET_PID accepted");

  /* Applied to the live rate controller. */
  float gkp, gki, gkd, gkff;
  CHECK(angle_rate_controller_get_gains(axis, &gkp, &gki, &gkd, &gkff),
        "rate gains readable");
  CHECK(feq(gkp, kp) && feq(gki, ki) && feq(gkd, kd) && feq(gkff, kff),
        "live rate gains updated to commanded values");

  /* Stored in the persistence layer. */
  float skp, ski, skd, skff;
  CHECK(pid_config_get_rate(axis, &skp, &ski, &skd, &skff),
        "stored gains present for axis");
  CHECK(feq(skp, kp) && feq(ski, ki) && feq(skd, kd) && feq(skff, kff),
        "stored gains match commanded values");

  /* Persistence round-trip: reload from the (RAM-backed) VFS file and
   * confirm the value survives — proves save wrote and load read it. */
  pid_config_init();
  float rkp, rki, rkd, rkff;
  CHECK(pid_config_get_rate(axis, &rkp, &rki, &rkd, &rkff) && feq(rkp, kp) &&
            feq(rki, ki) && feq(rkd, kd) && feq(rkff, kff),
        "gains survive a save -> reload round-trip");

  /* Out-of-range selectors rejected. */
  build_set_pid(p, PID_SET_ARGC, (float)PID_CTRL_RATE, 9.f, kp, ki, kd, kff);
  CHECK(pid_config_apply_command(p, len) == VAYU_ERR_INVALID,
        "out-of-range axis rejected");
  build_set_pid(p, PID_SET_ARGC, 7.f, 0.f, kp, ki, kd, kff);
  CHECK(pid_config_apply_command(p, len) == VAYU_ERR_INVALID,
        "out-of-range controller rejected");
}

/* ----------------------------------------------------------------------------
 * COMM-CH-002 — a write that would exceed the 512 B TX buffer returns
 * ERROR and bumps the overflow counter.
 * --------------------------------------------------------------------------*/
static void test_tx_overflow(void) {
  printf("  test_tx_overflow (COMM-CH-002)\n");

  channel_t ch;
  memset(&ch, 0, sizeof ch);
  serial_args_t args = {.baud_rate = 115200, .uart = HAL_UART_1, .timeout = 100};
  CHECK(get_handler(CHANNEL_TYPE_SERIAL, &ch, &args, NULL) == NONE,
        "serial channel opened");

  byte buf[256];
  memset(buf, 0xAB, sizeof buf);
  uint32_t before = channel_tx_overflow_count();

  /* No flush runs in the unit test, so the active 512 B buffer just fills:
   * 2 x 256 = 512 (exactly full), then any further byte overflows. */
  CHECK(write_channel(ch, buf, 256) == NONE, "first 256 B write ok");
  CHECK(write_channel(ch, buf, 256) == NONE, "second 256 B write fills buffer");
  CHECK(write_channel(ch, buf, 1) == ERROR, "write past 512 B returns ERROR");
  CHECK(channel_tx_overflow_count() == before + 1, "overflow counted once");

  write_channel(ch, buf, 1);
  CHECK(channel_tx_overflow_count() == before + 2, "second overflow counted");
}

int main(void) {
  printf("== Phase-3 COMM SITL verification ==\n");

  test_tx_overflow();
  test_payload_validation();
  test_set_pid_apply();
  test_arm_command_wire();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
