/**
 * @file sim/host/tests/test_fs_query.c
 * @brief SITL test of the filesystem-navigation service (FS_LIST / FS_INFO /
 *        FS_DELETE).
 *
 * Drives the real fs_query service (SM + fs_owner + host VFS) through a capturing
 * emitter: list a directory (entries + terminal), stat a file, and confirm a
 * missing path is reported DENIED ("does not exist") in both modes.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm/xfer/fs_query.h"
#include "storage/fs_owner.h"
#include "storage/imu_hs_log.h"
#include "sys/state.h"
#include "variables.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond)                                                                  \
      printf("    ok   %s\n", (msg));                                          \
    else {                                                                     \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

static struct {
  struct {
    uint32_t msgid;
    uint8_t req_seq, result;
  } ack[8];
  int n_ack;
  struct {
    uint8_t result, type;
    uint16_t index, count;
    uint32_t size;
    char name[16];
  } ent[64];
  int n_ent;
  struct {
    uint8_t result, type;
    uint32_t size, mtime;
  } info[8];
  int n_info;
} CAP;
static void cap_reset(void) { memset(&CAP, 0, sizeof CAP); }

static void e_ack(uint32_t msgid, uint8_t req, uint8_t result) {
  int i = CAP.n_ack++;
  CAP.ack[i].msgid = msgid;
  CAP.ack[i].req_seq = req;
  CAP.ack[i].result = result;
}
static void e_entry(uint8_t req, uint8_t result, uint16_t index, uint16_t count,
                    uint8_t type, uint32_t size, const char *name) {
  (void)req;
  int i = CAP.n_ent++;
  CAP.ent[i].result = result;
  CAP.ent[i].index = index;
  CAP.ent[i].count = count;
  CAP.ent[i].type = type;
  CAP.ent[i].size = size;
  snprintf(CAP.ent[i].name, sizeof CAP.ent[i].name, "%s", name);
}
static void e_info(uint8_t req, uint8_t result, uint8_t type, uint32_t size,
                   uint32_t mtime) {
  (void)req;
  int i = CAP.n_info++;
  CAP.info[i].result = result;
  CAP.info[i].type = type;
  CAP.info[i].size = size;
  CAP.info[i].mtime = mtime;
}
static const fs_query_tx_ops_t TX = {e_ack, e_entry, e_info};

static void drain(int max_ticks) {
  for (int t = 0; t < max_ticks && fs_query_busy(); t++)
    fs_query_tick(4);
}

static void test_list(void) {
  printf("  test_list\n");
  cap_reset();
  CHECK(fs_query_on_list(0x10, 1, 1, "0:", 0) == FS_QUERY_DEFERRED,
        "FS_LIST accepted + deferred");
  drain(64);
  CHECK(CAP.n_ack == 1 && CAP.ack[0].msgid == FS_WIRE_MSGID_LIST &&
            CAP.ack[0].result == FSQ_RES_OK,
        "FS_LIST COMMAND_ACK ACCEPTED");
  bool saw_a = false, saw_b = false, saw_terminal = false;
  uint16_t total = 0;
  for (int i = 0; i < CAP.n_ent; i++) {
    if (strcmp(CAP.ent[i].name, "q_a.bin") == 0)
      saw_a = true;
    if (strcmp(CAP.ent[i].name, "q_b.bin") == 0)
      saw_b = true;
    if (CAP.ent[i].name[0] == '\0') {
      saw_terminal = true;
      total = CAP.ent[i].count;
    }
  }
  CHECK(saw_a && saw_b, "FS_LIST streamed both files as entries");
  CHECK(saw_terminal && total >= 2,
        "FS_LIST emitted a terminal entry with the total count");
}

static void test_list_missing(void) {
  printf("  test_list_missing\n");
  cap_reset();
  fs_query_on_list(0x11, 1, 1, "0:no_such_dir/", 0);
  drain(8);
  CHECK(CAP.n_ent == 1 && CAP.ent[0].result == FSQ_RES_DENIED,
        "FS_LIST of a missing path -> single DENIED entry");
}

static void test_info(void) {
  printf("  test_info\n");
  cap_reset();
  CHECK(fs_query_on_info(0x20, 1, 1, "0:q_a.bin") == FS_QUERY_DEFERRED,
        "FS_INFO accepted + deferred");
  drain(8);
  CHECK(CAP.n_ack == 1 && CAP.ack[0].msgid == FS_WIRE_MSGID_INFO,
        "FS_INFO COMMAND_ACK for the right command");
  CHECK(CAP.n_info == 1 && CAP.info[0].result == FSQ_RES_OK &&
            CAP.info[0].type == 0 && CAP.info[0].size == 200u,
        "FS_INFO reports file size + type");

  cap_reset();
  fs_query_on_info(0x21, 1, 1, "0:does_not_exist.bin");
  drain(8);
  CHECK(CAP.n_info == 1 && CAP.info[0].result == FSQ_RES_DENIED,
        "FS_INFO of a missing path -> DENIED (does not exist)");
}

/* The delete guards are the reason this service can be exposed at all, so they
 * are tested from the refusing side first: a guard that has only ever been seen
 * to allow is not known to refuse. */
static uint8_t delete_result(uint8_t seq, const char *path) {
  cap_reset();
  int d = fs_query_on_delete(seq, 1, 1, path);
  if (d != FS_QUERY_DEFERRED) {
    return (uint8_t)d; /* refused inline, on the comm task */
  }
  drain(8);
  return (CAP.n_ack == 1 && CAP.ack[0].msgid == FS_WIRE_MSGID_DELETE)
             ? CAP.ack[0].result
             : 0xFFu;
}

static bool exists(const char *path) {
  vfs_stat_t st;
  return fs_owner_stat(path, &st) == 0 && st.exists;
}

static void test_delete_ok(void) {
  printf("  test_delete_ok\n");
  uint8_t blob[16];
  memset(blob, 0x5A, sizeof blob);
  fs_owner_enqueue_write_at(0, "0:q_del.bin", 0, blob, sizeof blob);
  fs_owner_pump();
  CHECK(exists("0:q_del.bin"), "seeded the file to delete");
  CHECK(delete_result(0x30, "0:q_del.bin") == FSQ_RES_OK,
        "FS_DELETE of an ordinary file -> ACCEPTED");
  CHECK(!exists("0:q_del.bin"), "the file is actually gone");
}

static void test_delete_missing(void) {
  printf("  test_delete_missing\n");
  CHECK(delete_result(0x31, "0:never_existed.bin") == FSQ_RES_DENIED,
        "FS_DELETE of a missing path -> DENIED");
}

static void test_delete_protected(void) {
  printf("  test_delete_protected\n");
  uint8_t blob[8];
  memset(blob, 0x11, sizeof blob);
  fs_owner_enqueue_write_at(0, CALIBRATION_FILE_PATH, 0, blob, sizeof blob);
  fs_owner_pump();
  fs_owner_enqueue_write_at(0, PID_CONFIG_FILE_PATH, 0, blob, sizeof blob);
  fs_owner_pump();

  CHECK(delete_result(0x32, CALIBRATION_FILE_PATH) == FSQ_RES_DENIED,
        "FS_DELETE of cal.bin -> DENIED");
  CHECK(exists(CALIBRATION_FILE_PATH), "cal.bin survived");
  CHECK(delete_result(0x33, PID_CONFIG_FILE_PATH) == FSQ_RES_DENIED,
        "FS_DELETE of pid.bin -> DENIED");
  CHECK(exists(PID_CONFIG_FILE_PATH), "pid.bin survived");
  /* FatFS is case-insensitive, so a guard that only matched one spelling of
   * the name would be trivially bypassed. */
  CHECK(delete_result(0x34, "0:PID.BIN") == FSQ_RES_DENIED,
        "FS_DELETE of 0:PID.BIN (upper case) -> DENIED");
  CHECK(exists(PID_CONFIG_FILE_PATH),
        "pid.bin survived the upper-case spelling");
}

static void test_delete_hsl_state_gated(void) {
  printf("  test_delete_hsl_state_gated\n");
  uint8_t blob[8];
  memset(blob, 0x22, sizeof blob);
  fs_owner_enqueue_write_at(0, HSL_FILENAME, 0, blob, sizeof blob);
  fs_owner_pump();

  _system_current_status = SYSTEM_STATE_ARMED;
  imu_hs_log_drain(); /* opens a session: the recorder now holds the file */
  CHECK(imu_hs_log_active(), "recorder is active while armed");
  CHECK(delete_result(0x35, HSL_FILENAME) == FSQ_RES_BUSY,
        "FS_DELETE of imuhs.bin while recording -> TEMPORARILY_REJECTED");
  CHECK(exists(HSL_FILENAME), "imuhs.bin survived while recording");

  _system_current_status = SYSTEM_STATE_STANDBY;
  imu_hs_log_drain(); /* closes the session */
  CHECK(!imu_hs_log_active(), "recorder stopped on disarm");
  CHECK(delete_result(0x36, HSL_FILENAME) == FSQ_RES_OK,
        "FS_DELETE of imuhs.bin once disarmed -> ACCEPTED");
  CHECK(!exists(HSL_FILENAME), "imuhs.bin deleted once disarmed");
}

int main(void) {
  printf("== fs_query (filesystem navigation) SITL ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_fs_query_test", 1);
  fs_owner_init();
  fs_query_init(&TX);

  /* Seed two files (200 B + 80 B). */
  uint8_t blob[200];
  memset(blob, 0x3C, sizeof blob);
  fs_owner_enqueue_write_at(0, "0:q_a.bin", 0, blob, 200);
  fs_owner_pump();
  fs_owner_enqueue_write_at(0, "0:q_b.bin", 0, blob, 80);
  fs_owner_pump();

  test_list();
  test_list_missing();
  test_info();
  test_delete_ok();
  test_delete_missing();
  test_delete_protected();
  test_delete_hsl_state_gated();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
