/* NavLink v2 C codec tests + cross-language parity emitter.
 *
 * Asserts round-trips locally, then prints a machine-parseable report that
 * run_tests.py checks against the Python codec:
 *   TABLE <msgid> <crc_extra> <wire_size>      (one per message)
 *   BYTES <NAME> <hex>                          (packed payload for fixed values)
 * Exits non-zero on any local assertion failure. */
#include "navlink_msgs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void navlink_emit_parity(void); /* generated/c/navlink_parity.c */

int main(void) {
  /* CRC-16/MCRF4XX self-check (spec §16.1). */
  uint16_t crc = 0xFFFF;
  const char *s = "123456789";
  for (const char *p = s; *p; p++)
    navlink_crc_accumulate((uint8_t)*p, &crc);
  assert(crc == 0x6F91);

  /* Local round-trips. */
  {
    navlink_attitude_euler_t a = {.roll = 0.5f,
                                  .pitch = -0.25f,
                                  .yaw = 1.0f,
                                  .rollspeed = 0.1f,
                                  .pitchspeed = 0.2f,
                                  .yawspeed = 0.3f};
    navlink_attitude_euler_wire_t w;
    navlink_attitude_euler_from_aligned(&w, &a);
    uint8_t buf[64];
    size_t n = navlink_attitude_euler_pack(buf, &w);
    assert(n == sizeof w && n == NAVLINK_WIRE_SIZE_ATTITUDE_EULER);
    navlink_attitude_euler_wire_t w2;
    navlink_attitude_euler_unpack(&w2, buf, n);
    navlink_attitude_euler_t a2;
    navlink_attitude_euler_to_aligned(&a2, &w2);
    assert(a2.roll == a.roll && a2.yawspeed == a.yawspeed);
  }
  {
    navlink_command_ack_t k = {.command = 0x123456,
                               .req_seq = 7,
                               .result = 0,
                               .progress = 100,
                               .result_param2 = -3};
    navlink_command_ack_wire_t w;
    navlink_command_ack_from_aligned(&w, &k);
    navlink_command_ack_t k2;
    navlink_command_ack_to_aligned(&k2, &w);
    assert(k2.command == 0x123456 && k2.result_param2 == -3);
  }
  /* Dispatch table is sorted + binary search works. */
  for (size_t i = 1; i < NAVLINK_MSG_COUNT; i++)
    assert(navlink_msg_table[i - 1].msgid < navlink_msg_table[i].msgid);
  assert(navlink_msg_info(1034) &&
         strcmp(navlink_msg_info(1034)->name, "PERF_GLOBAL") == 0);
  assert(navlink_msg_info(0x424242) == NULL);

  /* Short unpack zero-fills the rest (spec §5.6). */
  {
    uint8_t two[2] = {7, 0};
    navlink_param_value_wire_t w;
    navlink_param_value_unpack(&w, two, 2);
    navlink_param_value_t a;
    navlink_param_value_to_aligned(&a, &w);
    assert(a.param_id[0] == 7 && a.count == 0 && a.generation == 0 &&
           a.value[7] == 0);
  }
  /* Oversize unpack is clamped to the wire size (no overrun). */
  {
    uint8_t big[64];
    memset(big, 0xAA, sizeof big);
    navlink_attitude_euler_wire_t w;
    navlink_attitude_euler_unpack(&w, big, sizeof big);
    uint8_t out[64];
    size_t n = navlink_attitude_euler_pack(out, &w);
    assert(n == NAVLINK_WIRE_SIZE_ATTITUDE_EULER);
  }
  /* Signed i8 array round-trip (motor spin = +1/-1). */
  {
    navlink_cmd_set_motor_geometry_t g;
    memset(&g, 0, sizeof g);
    g.spin[0] = 1;
    g.spin[1] = -1;
    g.spin[2] = 1;
    g.spin[3] = -1;
    navlink_cmd_set_motor_geometry_wire_t w;
    navlink_cmd_set_motor_geometry_from_aligned(&w, &g);
    navlink_cmd_set_motor_geometry_t g2;
    navlink_cmd_set_motor_geometry_to_aligned(&g2, &w);
    assert(g2.spin[1] == -1 && g2.spin[3] == -1);
  }
  /* u64 extreme + i32 sentinel round-trip. */
  {
    navlink_time_sync_t t = {.role = 1,
                             .seq = 9,
                             .t1_gcs_tx = 0xFFFFFFFFFFFFFFFFull,
                             .t3_fc_tx = 0x0102030405060708ull,
                             .commanded_offset_ms = INT32_MIN};
    navlink_time_sync_wire_t w;
    navlink_time_sync_from_aligned(&w, &t);
    navlink_time_sync_t t2;
    navlink_time_sync_to_aligned(&t2, &w);
    assert(t2.t1_gcs_tx == 0xFFFFFFFFFFFFFFFFull &&
           t2.t3_fc_tx == 0x0102030405060708ull &&
           t2.commanded_offset_ms == INT32_MIN && t2.role == 1 && t2.seq == 9);
  }

  /* Emit parity table (msgid crc_extra wire_size). */
  for (size_t i = 0; i < NAVLINK_MSG_COUNT; i++)
    printf("TABLE %u %u %u\n", navlink_msg_table[i].msgid,
           navlink_msg_table[i].crc_extra, navlink_msg_table[i].wire_size);

  /* Emit byte vectors for every message (canonical values, must match Python). */
  navlink_emit_parity();

  fprintf(stderr, "C: %u messages, all local asserts passed\n",
          NAVLINK_MSG_COUNT);
  return 0;
}
