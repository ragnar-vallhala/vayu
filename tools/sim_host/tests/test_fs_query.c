/**
 * @file tools/sim_host/tests/test_fs_query.c
 * @brief SITL test of the filesystem-navigation service (FS_LIST / FS_INFO).
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

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    g_checks++;                                                                 \
    if (cond)                                                                   \
      printf("    ok   %s\n", (msg));                                           \
    else {                                                                      \
      g_fails++;                                                                \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);             \
    }                                                                           \
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

int main(void) {
  printf("== fs_query (filesystem navigation) SITL ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_fs_query_test", 1);
  fs_owner_init();
  fs_query_init(&TX);

  /* Seed two files (200 B + 80 B). */
  uint8_t blob[200];
  memset(blob, 0x3C, sizeof blob);
  fs_owner_enqueue_write_at("0:q_a.bin", 0, blob, 200);
  fs_owner_pump();
  fs_owner_enqueue_write_at("0:q_b.bin", 0, blob, 80);
  fs_owner_pump();

  test_list();
  test_list_missing();
  test_info();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
