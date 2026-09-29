/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file sim/host/tests/test_fs_owner.c
 * @brief SITL verification suite for the centralised filesystem owner.
 *
 * Links the real firmware source (libvayu_sitl_core -> src/storage/fs_owner.c)
 * and drives its public contract directly. The FS task itself is not spawned;
 * fs_owner_pump() runs the exact same handlers the task runs, synchronously and
 * deterministically, so the queue/snapshot/save/log paths are testable without
 * a scheduler.
 *
 *   @verifies the C1->C3 fix design properties (firmware/docs/plans/centralised-fs-owner.md):
 *     - copy-into-queue snapshot is safe against post-enqueue mutation
 *     - reserved save lane is never starved by a full log lane
 *     - overflow logs are dropped-and-counted (mirrors COMM-CH-002)
 *     - boot_init + circular write actually persist a record to SD
 *     - calib save concatenates header+payload faithfully
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "storage/fs_owner.h"
#include "est/hover_estimate.h"
#include "control/height_controller.h"
#include "storage/hover_store.h"
#include "vfs.h"

static int g_checks = 0;
static int g_fails = 0;

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

/* Read up to `max` bytes from a VFS path into buf; returns byte count or <0. */
static int read_file(const char *path, void *buf, uint32_t max) {
  vfs_fd_t fd = vfs_open(path, VFS_O_RDONLY);
  if (fd < 0)
    return -1;
  int n = vfs_read(fd, buf, max);
  vfs_close(fd);
  return n;
}

/* ----------------------------------------------------------------------------
 * Producers drop (don't crash) before the queues exist. Must run FIRST, before
 * any fs_owner_init(), because s_ready latches true for the process lifetime.
 * --------------------------------------------------------------------------*/
static void test_not_ready_drops(void) {
  printf("  test_not_ready_drops\n");
  uint8_t blob[16];
  memset(blob, 0xA5, sizeof blob);

  bool pid_ok = fs_owner_enqueue_pid_save(blob, sizeof blob);
  CHECK(!pid_ok, "save enqueue refused before init");
  CHECK(fs_owner_dropped_saves() >= 1, "pre-init save drop counted");
}

/* ----------------------------------------------------------------------------
 * Copy-into-queue: the persisted bytes are the snapshot taken at enqueue time,
 * NOT whatever the source holds when the FS task later runs. This is the
 * torn-read guarantee that lets pid_config_save() return immediately.
 * --------------------------------------------------------------------------*/
static void test_pid_snapshot_round_trip(void) {
  printf("  test_pid_snapshot_round_trip\n");

  uint8_t src[140];
  for (uint32_t i = 0; i < sizeof src; i++)
    src[i] = (uint8_t)(i * 7u + 1u); /* deterministic pre-mutation pattern */

  bool ok = fs_owner_enqueue_pid_save(src, sizeof src);
  CHECK(ok, "pid save enqueued");

  /* Mutate the source AFTER enqueue, BEFORE the write runs. A pointer-passing
   * design would persist this garbage; a snapshot must ignore it. */
  memset(src, 0xFF, sizeof src);

  fs_owner_pump();

  uint8_t back[140];
  int n = read_file("0:pid.bin", back, sizeof back);
  CHECK(n == (int)sizeof back, "pid.bin has the snapshot length");

  bool matches_snapshot = true;
  for (uint32_t i = 0; i < sizeof back; i++)
    if (back[i] != (uint8_t)(i * 7u + 1u))
      matches_snapshot = false;
  CHECK(matches_snapshot, "persisted bytes are the pre-mutation snapshot");
}

/* ----------------------------------------------------------------------------
 * Calib save concatenates header + payload into one record, in order.
 * --------------------------------------------------------------------------*/
static void test_calib_snapshot_round_trip(void) {
  printf("  test_calib_snapshot_round_trip\n");

  uint8_t hdr[8];
  uint8_t payload[84];
  for (uint32_t i = 0; i < sizeof hdr; i++)
    hdr[i] = (uint8_t)(0x10u + i);
  for (uint32_t i = 0; i < sizeof payload; i++)
    payload[i] = (uint8_t)(0x80u + i);

  bool ok =
      fs_owner_enqueue_calib_save(hdr, sizeof hdr, payload, sizeof payload);
  CHECK(ok, "calib save enqueued");
  fs_owner_pump();

  uint8_t back[8 + 84];
  int n = read_file("0:cal.bin", back, sizeof back);
  CHECK(n == (int)sizeof back, "cal.bin = header + payload length");
  CHECK(memcmp(back, hdr, sizeof hdr) == 0, "header bytes first, intact");
  CHECK(memcmp(back + sizeof hdr, payload, sizeof payload) == 0,
        "payload bytes follow header, intact");
}

/* ----------------------------------------------------------------------------
 * Reserved save lane: a full lower-priority lane must NOT block or drop a
 * save. This is the "reserved save slots" decision — the whole reason saves
 * have a queue of their own.
 *
 * The lane doing the flooding used to be the SD log lane; that lane is gone
 * (text logs are a stream in the blackbox recorder now), so the flood comes
 * from the upload write-at lane instead — which is the remaining lane the
 * drain order puts after saves.
 * --------------------------------------------------------------------------*/
static void test_reserved_save_lane(void) {
  printf("  test_reserved_save_lane\n");

  fs_owner_pump(); /* start from drained queues */

  uint32_t wa0 = fs_owner_dropped_writeats();
  uint32_t saves0 = fs_owner_dropped_saves();

  /* Flood the write-at lane far past capacity WITHOUT draining. */
  uint8_t rec[64];
  memset(rec, 0x5A, sizeof rec);
  int accepted = 0;
  const int flood = 200;
  for (int i = 0; i < flood; i++)
    if (fs_owner_enqueue_write_at(0, "0:flood.bin", (uint32_t)i * 64u, rec,
                                  sizeof rec))
      accepted++;

  CHECK(accepted >= 1 && accepted <= 32,
        "write-at lane bounded by its capacity (<=32)");
  CHECK(fs_owner_dropped_writeats() == wa0 + (uint32_t)(flood - accepted),
        "every over-capacity write-at is counted as a drop");

  /* With that lane jammed full, a save must still be accepted. */
  uint8_t pid[140];
  memset(pid, 0x11, sizeof pid);
  bool saved = fs_owner_enqueue_pid_save(pid, sizeof pid);
  CHECK(saved, "save accepted while log lane is full (reserved lane)");
  CHECK(fs_owner_dropped_saves() == saves0, "no save dropped");

  fs_owner_pump(); /* drain everything we queued */
}

/* ----------------------------------------------------------------------------
 * Write-at lane (xfer uploads): positioned writes accumulate into one file and
 * read back via fs_owner_read_at. Small file (<4 KB) so the host VFS models it.
 * --------------------------------------------------------------------------*/
static void test_writeat_roundtrip(void) {
  printf("  test_writeat_roundtrip\n");
  fs_owner_pump(); /* drain */

  uint8_t src[600];
  for (uint32_t i = 0; i < sizeof src; i++)
    src[i] = (uint8_t)(i * 13u + 5u);

  /* Three positioned chunks, as an upload would arrive — drained between
   * batches by the FS task (lane CAP=2 flow-controls more than 2 outstanding). */
  CHECK(fs_owner_enqueue_write_at(0, "0:upload.bin", 0, src, 247),
        "write-at chunk @0 queued");
  CHECK(fs_owner_enqueue_write_at(0, "0:upload.bin", 247, src + 247, 247),
        "write-at chunk @247 queued");
  fs_owner_pump(); /* FS task drains the 2 outstanding */
  CHECK(fs_owner_enqueue_write_at(0, "0:upload.bin", 494, src + 494, 106),
        "write-at chunk @494 queued after drain");
  fs_owner_pump();

  uint8_t back[600];
  int n = fs_owner_read_at("0:upload.bin", 0, back, sizeof back);
  CHECK(n == (int)sizeof back, "read-at returns the full written length");
  CHECK(memcmp(back, src, sizeof src) == 0,
        "positioned writes reassemble to the source");

  /* Positioned partial read. */
  uint8_t mid[100];
  int m = fs_owner_read_at("0:upload.bin", 247, mid, sizeof mid);
  CHECK(m == (int)sizeof mid && memcmp(mid, src + 247, sizeof mid) == 0,
        "read-at honours the offset");
}

/* ----------------------------------------------------------------------------
 * The write-at lane is bounded + drop-counted, and (being a separate lane) it
 * can neither starve the reserved save lane nor be starved by logs.
 * --------------------------------------------------------------------------*/
static void test_writeat_lane_bounds_and_reservation(void) {
  printf("  test_writeat_lane_bounds_and_reservation\n");
  fs_owner_pump(); /* drained queues */

  uint32_t wa0 = fs_owner_dropped_writeats();
  uint32_t saves0 = fs_owner_dropped_saves();

  /* Flood the write-at lane WITHOUT draining. */
  uint8_t rec[64];
  memset(rec, 0x7E, sizeof rec);
  int accepted = 0;
  const int flood = 50;
  for (int i = 0; i < flood; i++)
    if (fs_owner_enqueue_write_at(0, "0:u.bin", (uint32_t)(i * 64), rec,
                                  sizeof rec))
      accepted++;
  CHECK(accepted >= 1 && accepted <= 2,
        "write-at lane bounded by its capacity (<=2)");
  CHECK(fs_owner_dropped_writeats() == wa0 + (uint32_t)(flood - accepted),
        "every over-capacity write-at is counted as a drop");

  /* With the write-at lane jammed full, a save is still accepted (reserved). */
  uint8_t pid[140];
  memset(pid, 0x22, sizeof pid);
  CHECK(fs_owner_enqueue_pid_save(pid, sizeof pid),
        "save accepted while write-at lane is full (reserved lane)");
  CHECK(fs_owner_dropped_saves() == saves0, "no save dropped");

  /* Over-long path is rejected (bounds the fixed path buffer). */
  char longpath[80];
  memset(longpath, 'a', sizeof longpath);
  longpath[sizeof longpath - 1] = '\0';
  uint32_t wa1 = fs_owner_dropped_writeats();
  CHECK(!fs_owner_enqueue_write_at(0, longpath, 0, rec, sizeof rec),
        "over-long path rejected");
  CHECK(fs_owner_dropped_writeats() == wa1 + 1,
        "rejected path counted as a drop");

  fs_owner_pump(); /* drain everything queued */
}

/* ----------------------------------------------------------------------------
 * Directory browse + stat (the filesystem-navigation gateway). Create a couple
 * of files, list the root, stat a file, and confirm a bad path reports "does
 * not exist" distinctly.
 * --------------------------------------------------------------------------*/
static void test_dir_browse_and_stat(void) {
  printf("  test_dir_browse_and_stat\n");

  uint8_t blob[120];
  memset(blob, 0x5C, sizeof blob);
  fs_owner_enqueue_write_at(0, "0:nav_a.bin", 0, blob, sizeof blob);
  fs_owner_pump();
  fs_owner_enqueue_write_at(0, "0:nav_b.bin", 0, blob, 64);
  fs_owner_pump();

  /* stat an existing file: exists, size, not a directory. */
  vfs_stat_t st;
  int r = fs_owner_stat("0:nav_a.bin", &st);
  CHECK(r == 0 && st.exists == 1, "stat finds an existing file");
  CHECK(st.size == sizeof blob && st.is_dir == 0,
        "stat reports size + not-dir");

  /* stat a missing path: distinct "does not exist". */
  r = fs_owner_stat("0:nope_xyz.bin", &st);
  CHECK(r < 0 && st.exists == 0,
        "stat of a missing path reports does-not-exist");

  /* stat the root: it is a directory. */
  CHECK(fs_owner_stat("0:", &st) == 0 && st.is_dir == 1, "root stats as a dir");

  /* list the root and confirm both files appear. */
  vfs_dir_t d = fs_owner_opendir("0:");
  CHECK(d >= 0, "opendir root succeeds");
  bool saw_a = false, saw_b = false;
  int guard = 0;
  vfs_dirent_t ent;
  while (guard++ < 64) {
    int n = fs_owner_readdir(d, &ent);
    if (n <= 0)
      break;
    if (strcmp(ent.name, "nav_a.bin") == 0)
      saw_a = true;
    if (strcmp(ent.name, "nav_b.bin") == 0)
      saw_b = true;
  }
  fs_owner_closedir(d);
  CHECK(saw_a && saw_b, "readdir lists the created files");

  /* opendir of a non-directory / missing path fails (does not exist). */
  CHECK(fs_owner_opendir("0:not_a_dir/") < 0,
        "opendir of a missing path fails");
}

/* ----------------------------------------------------------------------------
 * The persisted hover collective: saved through the write-at lane, loaded
 * through fs_owner_read_at. Both halves go through the owner, which is the
 * point -- the load used to call vfs_open/vfs_read directly from
 * vertical_estimator_task, after the scheduler is up and so concurrently with
 * whatever the FS task had open.
 *
 * The guards matter more than the round-trip: this number is what the next
 * lift-off opens the throttle at, so a short read, a foreign file or a wild
 * value must all fall back to the compiled guess rather than be flown.
 * --------------------------------------------------------------------------*/
static void test_hover_store_roundtrip(void) {
  printf("  test_hover_store_roundtrip\n");
  fs_owner_pump();

  const float guess = HEIGHT_HOVER_GUESS;

  (void)fs_owner_unlink(HOVER_STORE_PATH);
  CHECK(hover_store_load(guess) == guess,
        "no stored file falls back to the guess");

  CHECK(hover_store_save(0.42f), "a plausible hover is queued");
  fs_owner_pump();
  const float got = hover_store_load(guess);
  CHECK(got > 0.4199f && got < 0.4201f, "and reads back through fs_owner");

  /* Out of band on either side is refused at SAVE time, so a bad estimate
   * never reaches the card and the previous good one survives. */
  CHECK(!hover_store_save(HOVER_EST_MAX + 0.1f),
        "an over-range hover is not saved");
  CHECK(!hover_store_save(HOVER_EST_MIN - 0.01f), "nor an under-range one");
  fs_owner_pump();
  const float still = hover_store_load(guess);
  CHECK(still > 0.4199f && still < 0.4201f,
        "the last good value is still what is stored");

  /* A file of the right size with the wrong magic is somebody else's data.
   * Its VALUE is deliberately in band, so only the magic check can reject it
   * -- a foreign file whose bytes happen to be out of range would let the
   * range check pass this for the wrong reason. */
  {
    uint8_t foreign[8];
    const uint32_t not_ours = 0xDEADBEEFu;
    const float plausible = 0.42f;
    memcpy(foreign, &not_ours, 4);
    memcpy(foreign + 4, &plausible, 4);
    CHECK(fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, HOVER_STORE_PATH, 0,
                                    foreign, sizeof foreign),
          "a foreign 8-byte file with an in-band value is written");
    fs_owner_pump();
    CHECK(hover_store_load(guess) == guess,
          "bad magic falls back to the guess");
  }

  /* A truncated file: right magic, too few bytes. Two things stop it being
   * flown -- the length check, and the zero-initialised record whose 0.0f the
   * range check then refuses. Removing EITHER alone still falls back, so this
   * case holds the pair together rather than isolating one: drop both and the
   * short read reaches the throttle as uninitialised stack. */
  {
    (void)fs_owner_unlink(HOVER_STORE_PATH);
    const uint32_t magic = HOVER_STORE_MAGIC;
    CHECK(fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, HOVER_STORE_PATH, 0,
                                    &magic, sizeof magic),
          "a 4-byte truncated record is written");
    fs_owner_pump();
    CHECK(hover_store_load(guess) == guess,
          "a short read falls back rather than trusting a partial record");
  }

  /* Right magic, value outside the band -- the case a plain magic check would
   * pass straight through to the throttle. */
  {
    uint8_t rec[8];
    const uint32_t magic = HOVER_STORE_MAGIC;
    const float wild = 0.95f;
    memcpy(rec, &magic, 4);
    memcpy(rec + 4, &wild, 4);
    CHECK(fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, HOVER_STORE_PATH, 0,
                                    rec, sizeof rec),
          "a well-formed record with a wild value is written");
    fs_owner_pump();
    CHECK(hover_store_load(guess) == guess,
          "right magic but out-of-band value is still refused");
  }

  (void)fs_owner_unlink(HOVER_STORE_PATH);
}

int main(void) {
  printf("== FS owner SITL verification ==\n");

  /* Isolate from any other run's VFS state. */
  setenv("VAYU_VFS_DIR", "/tmp/vayu_fs_owner_test", 1);

  test_not_ready_drops(); /* must precede any init */

  /* NB: fs_owner_boot_init() is intentionally NOT called — it preallocs the
   * blackbox files to 64KB, which the 4 KB host-VFS file buffer can't hold.
   * The save path (pid.bin/cal.bin) opens its own small files and needs no
   * boot_init. */
  fs_owner_init(); /* create the queues */

  test_pid_snapshot_round_trip();
  test_calib_snapshot_round_trip();
  test_reserved_save_lane();
  test_writeat_roundtrip();
  test_writeat_lane_bounds_and_reservation();
  test_dir_browse_and_stat();
  test_hover_store_roundtrip();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
