/**
 * @file sim/host/tests/test_phase3_comm.c
 * @brief SITL verification suite for the Phase-3 COMM cluster.
 *
 * Links the real firmware sources (libvayu_sitl_core) and drives their
 * public contracts directly.
 *
 *   @verifies COMM-CH-002   TX-buffer overflow counter
 *   @verifies COMM-CMD-002  command payload length / argc validation
 *   @verifies COMM-CMD-003  CMD_SET_PID apply + SD persistence
 *   @verifies COMM-CMD-004  CMD_SET_GYRO_NOTCH apply + SD persistence
 *
 * The GCS software-arm path is exercised through the command apply engine: a
 * CMD_ARM / CMD_DISARM packet_t (as navlink_router.c reconstructs it from the
 * decoded v2 frame) is run through comm_processor_dispatch(), confirming it
 * sets/clears g_sw_arm_request. (The v1 wire deserializer is retired; v2 frame
 * decode + CRC is covered by the codec tests and live hardware validation.)
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

#include <math.h>

#include "comm/comm.h"
#include "comm/ibus.h"
#include "control/control.h"
#include "control/pid_config.h"
#include "dsp/gyro_notch.h"
#include "storage/fs_owner.h"
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

/* Build the internal command packet_t that navlink_router.c reconstructs from a
 * decoded v2 CMD_ARM / CMD_DISARM frame: a bare 2-byte cmd_id payload. */
static void make_command_packet(packet_t *pkt, uint16_t cmd_id) {
  memset(pkt, 0, sizeof(*pkt));
  pkt->sync = SYNC_BYTE;
  pkt->protocol_packet_type = (uint8_t)((PACKET_TYPE_COMMAND << 4) | 0x1);
  pkt->length = 2; /* cmd_id only */
  pkt->device_id = 42;
  memcpy(pkt->payload, &cmd_id, 2);
}

/* ----------------------------------------------------------------------------
 * GCS software-arm apply path — the CMD_ARM / CMD_DISARM packet the v2 router
 * hands to comm_processor_dispatch() flips g_sw_arm_request.
 * --------------------------------------------------------------------------*/
static void test_arm_command_wire(void) {
  printf("  test_arm_command_wire (GCS software-arm link)\n");

  packet_t pkt;

  g_sw_arm_request = 0;
  make_command_packet(&pkt, (uint16_t)CMD_ARM);
  comm_processor_dispatch(&pkt);
  CHECK(g_sw_arm_request == 1, "CMD_ARM dispatch sets the software-arm latch");

  make_command_packet(&pkt, (uint16_t)CMD_DISARM);
  comm_processor_dispatch(&pkt);
  CHECK(g_sw_arm_request == 0, "CMD_DISARM dispatch clears the latch");
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

  /* The PID save is enqueued off the comm task (the C1->C3 RX-drop fix), so the
   * save lane must be up for pid_config_save() to enqueue at all, and it needs a
   * synchronous drain to actually land on disk — a unit test has no fs_owner
   * task running it. (s_ready latches for the process, so this is idempotent.) */
  fs_owner_init();

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

  /* Persistence round-trip: drain the enqueued save so the CURRENT gains hit
   * pid.bin, then reload from the (host-mirrored) VFS file and confirm the value
   * survives — proves save wrote and load read it. Without the pump, init would
   * read a stale pid.bin from a previous run. */
  fs_owner_pump();
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
 * COMM-CH-002 — a write that would exceed the TX buffer returns ERROR and bumps
 * the overflow counter. NOTE: normal writes (telemetry/acks) are now capped at
 * CHANNEL_TX_BUF_SIZE - CHANNEL_TX_XFER_RESERVE (2048 - 768 = 1280) so a
 * saturating telemetry stream can't starve a bulk xfer; write_channel_xfer may
 * use the reserved tail up to the full 2048.
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

  /* No flush runs in the unit test, so the buffer just fills. Normal writes stop
   * at the reserved cap: 5 x 256 = 1280 (exactly the normal cap), then any
   * further normal byte overflows. */
  for (int i = 0; i < 5; i++)
    CHECK(write_channel(ch, buf, 256) == NONE,
          "256 B normal write fills toward the 1280 reserved cap");
  CHECK(write_channel(ch, buf, 1) == ERROR, "normal write past 1280 B returns ERROR");
  CHECK(channel_tx_overflow_count() == before + 1, "overflow counted once");

  write_channel(ch, buf, 1);
  CHECK(channel_tx_overflow_count() == before + 2, "second overflow counted");

  /* The reserved tail is still available to bulk xfer: with 1280 B of normal
   * data parked, write_channel_xfer can take ~768 more up to the full 2048. */
  CHECK(write_channel_xfer(ch, buf, 256) == NONE,
        "xfer write uses the reserved tail past the normal cap");
}

/* ----------------------------------------------------------------------------
 * COMM-CMD-004 — CMD_SET_GYRO_NOTCH applies live and persists. Drives the typed
 * apply contract (as navlink_router.c's handler does) rather than a byte payload.
 * --------------------------------------------------------------------------*/
static void test_set_gyro_notch_apply(void) {
  printf("  test_set_gyro_notch_apply (COMM-CMD-004)\n");

  fs_owner_init(); /* save lane must be up to enqueue (see test_set_pid_apply) */
  /* Allocate the per-axis banks so the live notch actually holds params. Returns
   * false when the notch is compiled out (VAYU_FFT_NOTCH off) — the persistence
   * store is independent of that, so only the live read-backs are gated on it. */
  bool notch_live = gyro_notch_init();

  const float q = 10.0f, fmin = 80.0f, fmax = 400.0f, ratio = 5.0f;
  CHECK(pid_config_apply_gyro_notch(true, q, fmin, fmax, ratio) == VAYU_OK,
        "valid SET_GYRO_NOTCH accepted");
  CHECK(pid_config_apply_gyro_notch(true, NAN, fmin, fmax, ratio) ==
            VAYU_ERR_INVALID,
        "NaN detection field rejected");

  if (notch_live) {
    float gq, gf, gx, gr;
    CHECK(gyro_notch_get_params(&gq, &gf, &gx, &gr) && feq(gq, q) &&
              feq(gf, fmin) && feq(gx, fmax) && feq(gr, ratio),
          "live notch detection params updated to commanded values");
    CHECK(gyro_notch_enabled(), "notch master gate enabled live");

    /* Partial command (all detection fields <=0 = unchanged): the store must
     * still capture the full EFFECTIVE tune via the read-back path. */
    CHECK(pid_config_apply_gyro_notch(true, -1.0f, -1.0f, -1.0f, -1.0f) ==
              VAYU_OK,
          "partial (unchanged) SET_GYRO_NOTCH accepted");
  }

  /* Persistence round-trip: drain the enqueued save, reload, and confirm the
   * tune (and enable) survive — proves the PID4 store field wrote and read. */
  fs_owner_pump();
  pid_config_init();
  float sq, sf, sx, sr;
  bool sen = false;
  CHECK(pid_config_get_gyro_notch(&sq, &sf, &sx, &sr, &sen),
        "stored notch tune present after reload");
  CHECK(feq(sq, q) && feq(sf, fmin) && feq(sx, fmax) && feq(sr, ratio) && sen,
        "notch tune + enable survive a save -> reload round-trip");
}

/* ----------------------------------------------------------------------------
 * Dynamic notch runtime: engaged above the throttle gate, the analyzer tracks a
 * pure tone and the retune lands the notch center on it; a disengage edge clears
 * the tuning. Exercises the hot filter path + staggered service on the host.
 * --------------------------------------------------------------------------*/
static void test_gyro_notch_runtime(void) {
  printf("  test_gyro_notch_runtime (dynamic notch tracks a tone)\n");

  if (!gyro_notch_init()) {
    printf("    (notch compiled out; skipping runtime drive)\n");
    return;
  }
  CHECK(gyro_notch_decimation() == 1u,
        "decimation factor is 1 at the 1 kHz loop");

  /* Enable with a band that brackets a 200 Hz test tone, then push throttle past
   * the engage gate. */
  pid_config_apply_gyro_notch(true, 8.0f, 60.0f, 450.0f, 4.0f);
  gyro_notch_set_throttle(0.5f);

  const float fs = 1000.0f, ftone = 200.0f;
  const float w = 6.283185307f * ftone / fs; /* -std=c11: no M_PI */
  /* One axis: apply() every tick (observes raw + filters), service() once a tick
   * runs at most one FFT retune. A few hops (N/2 = 64) in, the peak is found. */
  for (int t = 0; t < 600; t++) {
    float s = 50.0f * sinf(w * (float)t);
    (void)gyro_notch_apply(0, s);
    gyro_notch_service();
  }
  float c = gyro_notch_center_hz(0, 0);
  CHECK(c > 150.0f && c < 250.0f, "notch center tracks the 200 Hz tone");

  /* Disengage edge (throttle chopped): the banks reset so no stale notch is left
   * for the next spool-up. */
  gyro_notch_set_throttle(0.0f);
  gyro_notch_service();
  CHECK(gyro_notch_center_hz(0, 0) == 0.0f, "disengage clears the tuned center");
}

int main(void) {
  printf("== Phase-3 COMM SITL verification ==\n");

  test_tx_overflow();
  test_payload_validation();
  test_set_pid_apply();
  test_set_gyro_notch_apply();
  test_gyro_notch_runtime();
  test_arm_command_wire();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
