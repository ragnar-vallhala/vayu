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
 * @file sim/host/tests/test_hslog_drops.c
 * @brief What the recorder says it lost, and whether that can be believed.
 *
 * A card pull from the 2026-09-29 flights read "0 dropped" for a session that
 * had in fact dropped 19 sectors. Three separate things made that number
 * useless, and each is checked here:
 *
 *   1. The close line was emitted AFTER vfs_close(), so it could only ever be
 *      written into the NEXT session. Every close line in that pull describes
 *      the session before the one carrying it, and the flight that ended in a
 *      power cut had no line at all.
 *   2. The counter was cumulative since boot, so even in the right place it
 *      did not say what one flight lost.
 *   3. It was a single number across eight streams, so it could not say that
 *      the loss was ctl -- the one stream with two buffers -- which is the
 *      whole diagnosis.
 *
 * The counts now live in the file header, which is re-flushed every
 * HSL_HDR_SYNC_FRAMES, so they survive the power cut that loses the text.
 *
 * Separately: every path that ends a session returned before the drain loop at
 * the end of imu_hs_log_drain, so published-but-unwritten sectors were
 * abandoned at each disarm without being counted as anything. That is checked
 * last.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "storage/fs_owner.h"
#include "storage/imu_hs_log.h"
#include "storage/paths.h"
#include "sys/state.h"
#include "vfs.h"

static int g_checks = 0, g_fails = 0;
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

extern volatile sys_state_t _system_current_status;

/* Stream indices, in the order the header stores their counters. Mirrors the
 * private enum in imu_hs_log.c; if that order changed without this following,
 * the attribution checks below would fail rather than quietly mislabel. */
enum { D_IMU = 0, D_ACT, D_VRT, D_CTL, D_RX, D_TXT, D_ATT, D_RC, D_N };

static const float RATES[3] = {10.0f, -20.0f, 30.0f};
static const float U[3] = {0.25f, -0.5f, 0.125f};
static const uint16_t MOTORS[4] = {1000, 2000, 3000, 4000};

static uint16_t rd16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}

/* Read the per-stream drop counters straight out of the file header. */
static bool read_drops(uint16_t out[D_N]) {
  int fd = vfs_open(HSL_FILENAME, VFS_O_RDONLY);
  if (fd < 0) {
    return false;
  }
  uint8_t h[HSL_FILE_HDR_BYTES];
  bool ok = vfs_read(fd, h, sizeof h) == (int)sizeof h;
  vfs_close(fd);
  if (!ok) {
    return false;
  }
  for (int i = 0; i < D_N; i++) {
    out[i] = rd16(&h[HSL_HDR_DROPS_OFF + i * 2]);
  }
  return true;
}

/* Find the txt sector carrying `needle` and report the SESSION TAG of the
 * sector it landed in. -1 if the text is nowhere in the file.
 *
 * "is the text present" is not the question -- with the line emitted after
 * vfs_close it is still present, just filed under the following session. The
 * question is which session's sector carries it. */
static int session_tag_carrying(const char *needle) {
  int fd = vfs_open(HSL_FILENAME, VFS_O_RDONLY);
  if (fd < 0) {
    return -1;
  }
  static uint8_t buf[HSL_FILE_SIZE];
  int n = vfs_read(fd, buf, sizeof buf);
  vfs_close(fd);
  if (n <= 0) {
    return -1;
  }
  const size_t len = strlen(needle);
  /* The SIM ring is 63 slots and wraps repeatedly, so earlier sessions'
   * copies of the same line are still on the card. Slot order is not time
   * order either -- take the match with the highest seq, which is the one
   * just written. */
  int best_tag = -1;
  uint32_t best_seq = 0;
  for (int off = HSL_PREAMBLE_BYTES; off + (int)HSL_SECTOR_BYTES <= n;
       off += (int)HSL_SECTOR_BYTES) {
    const uint8_t *sec = &buf[off];
    if (sec[0] != HSL_TYPE_BLOCK || sec[4] != HSL_STREAM_TXT) {
      continue;
    }
    const uint16_t cnt = rd16(&sec[6]);
    const uint8_t *pay = sec + HSL_FRAME_HDR_BYTES + HSL_BLOCK_HDR_BYTES;
    if (cnt < len) {
      continue;
    }
    const uint32_t seq = (uint32_t)sec[8] | ((uint32_t)sec[9] << 8) |
                         ((uint32_t)sec[10] << 16) | ((uint32_t)sec[11] << 24);
    for (uint16_t i = 0; i + len <= cnt; i++) {
      if (memcmp(&pay[i], needle, len) == 0) {
        if (best_tag < 0 || seq > best_seq) {
          best_tag = (int)sec[5];
          best_seq = seq;
        }
        break;
      }
    }
  }
  return best_tag;
}

/* One armed period. `drain_every` records how often the FS task gets to run:
 * a large value starves the rings and is what makes a sector drop. 0 means
 * never drain mid-session. */
static void run_session(int n_samples, int drain_every) {
  _system_current_status = SYSTEM_STATE_ARMED;
  imu_hs_log_drain();
  for (int i = 0; i < n_samples; i++) {
    const uint32_t t = (uint32_t)i * (84000000u / 2000u);
    imu_hs_log_sample((int16_t[3]){(int16_t)i, 0, 0}, (int16_t[3]){0, 0, 0}, t);
    imu_hs_log_act(MOTORS, 1234u, HSL_ACT_F_ARMED, t);
    if ((i % 2) == 0) {
      imu_hs_log_ctl(RATES, U, t);
    }
    if (drain_every > 0 && (i % drain_every) == (drain_every - 1)) {
      imu_hs_log_drain();
    }
  }
  _system_current_status = SYSTEM_STATE_STANDBY;
  imu_hs_log_drain();
}

int main(void) {
  printf("== HSL drop accounting ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_hslog_drops", 1);
  remove("/tmp/vayu_hslog_drops/0_blackbox.bin");
  imu_hs_log_set_scale(0.0610351562f, 0.0047884034f);
  imu_hs_log_boot_init();

  uint16_t d[D_N];

  printf("  [1] a promptly-drained session loses nothing\n");
  {
    run_session(400, 20);
    CHECK(read_drops(d), "the header carries per-stream counters");
    uint32_t sum = 0;
    for (int i = 0; i < D_N; i++) {
      sum += d[i];
    }
    CHECK(sum == 0, "nothing dropped when the FS task keeps up");
    CHECK(imu_hs_log_dropped() == 0, "and the lifetime total agrees");
  }

  printf("  [2] a starved drain attributes each loss to its own stream\n");
  {
    /* Note what this does NOT show: ctl losing first because it has two
     * buffers. The depths are balanced in TIME, not in count -- ctl holds
     * 2 x 41 ms and imu 4 x 20.5 ms, both 82 ms -- so under equal starvation
     * they lose in proportion to their rates, 2:1, which is what comes out
     * below. The flight logs' ctl-heavy gap count therefore is NOT a
     * shallow-buffer effect, and the premise that it was is what a single
     * global counter let stand unexamined. */
    const uint32_t life_before = imu_hs_log_dropped();
    run_session(4000, 0);
    CHECK(read_drops(d), "counters readable after a starved session");
    printf("      imu=%u act=%u vrt=%u ctl=%u rx=%u txt=%u att=%u rc=%u\n",
           d[D_IMU], d[D_ACT], d[D_VRT], d[D_CTL], d[D_RX], d[D_TXT], d[D_ATT],
           d[D_RC]);
    CHECK(d[D_CTL] > 0 && d[D_IMU] > 0, "the starved streams lost sectors");
    CHECK(d[D_VRT] == 0 && d[D_RC] == 0,
          "streams that were never fed lost nothing");
    /* imu runs at twice ctl's rate into an equal time-depth of buffering, so
     * it loses about twice as much. Bounded rather than exact: the session
     * boundary lands wherever it lands. */
    CHECK(d[D_IMU] > d[D_CTL], "the faster stream lost more");
    CHECK(d[D_IMU] < 3u * d[D_CTL], "and in proportion to its rate, not wildly");
    CHECK(imu_hs_log_dropped() > life_before,
          "the lifetime total moved as well");
    uint32_t sum = 0;
    for (int i = 0; i < D_N; i++) {
      sum += d[i];
    }
    CHECK(imu_hs_log_dropped() - life_before == sum,
          "per-stream counts account for every drop, none double-counted");
  }

  printf("  [3] the counts are PER SESSION, the lifetime total is not\n");
  {
    /* The old counter was cumulative since boot, so a clean flight following
     * a bad one inherited its number and looked broken -- or, as happened, a
     * bad flight was read through the previous session's line and looked
     * clean. */
    const uint32_t life = imu_hs_log_dropped();
    CHECK(life > 0, "the previous session did lose sectors");
    run_session(400, 20);
    CHECK(read_drops(d), "counters readable");
    uint32_t sum = 0;
    for (int i = 0; i < D_N; i++) {
      sum += d[i];
    }
    CHECK(sum == 0, "a clean session reports zero, not the last one's total");
    CHECK(imu_hs_log_dropped() == life,
          "while the lifetime total is unchanged by a clean session");
  }

  printf("  [4] the close line lands in the session it describes\n");
  {
    /* It used to be emitted after vfs_close(), so it could only reach the
     * card as part of the NEXT session -- or never, if the vehicle lost
     * power. Both lines must be in the file the moment the session ends. */
    /* The needle names the session, so an older session's line cannot
     * satisfy this by accident -- which a bare "hsl: session " can, and did.
     * If the line is emitted after vfs_close it is still sitting in the text
     * stream's RAM buffer at this point and is on the card nowhere, so the
     * search misses entirely rather than finding the wrong tag. */
    const unsigned sess = imu_hs_log_session();
    char needle[64];
    snprintf(needle, sizeof needle, "hsl: session %u closed", sess);
    const int tag_cur = session_tag_carrying(needle);
    const int tag_drop = session_tag_carrying("hsl: dropped imu=");
    printf("      session=%u, its close line carried by tag %d (drops %d)\n",
           sess, tag_cur, tag_drop);
    CHECK(tag_cur >= 0,
          "this session's own close line is on the card when it closes");
    CHECK(tag_cur == (int)(sess & 0xFFu),
          "and is filed under the session it describes, not the next one");
    CHECK(tag_drop == (int)(sess & 0xFFu),
          "the per-stream drop line goes with it");
  }

  printf("  [5] ending a session writes what was already published\n");
  {
    /* Every path that ends a session returns before the drain loop at the end
     * of imu_hs_log_drain, so sectors sitting between tail and head were
     * abandoned at each disarm -- counted as nothing, and invisible except as
     * a hole at the end of every flight. */
    /* seq, not head_slot: under VAYU_SIM the ring is 63 slots and wraps
     * constantly, so the slot cursor is not monotonic. seq is. */
    _system_current_status = SYSTEM_STATE_ARMED;
    imu_hs_log_drain(); /* opens the session; costs a SESSION + EVENT sector */
    const uint32_t seq_open = imu_hs_log_seq();

    /* Fill several ctl sectors without ever letting the FS task run. */
    for (int i = 0; i < 41 * 2; i++) {
      imu_hs_log_ctl(RATES, U, (uint32_t)i * (84000000u / 1000u));
    }
    CHECK(imu_hs_log_seq() == seq_open,
          "nothing reached the card while the FS task was starved");

    _system_current_status = SYSTEM_STATE_STANDBY;
    imu_hs_log_drain(); /* the disarm path */
    /* One EVENT (the state change) plus the published ctl sector plus the
     * partial one: abandoning the published sector would cost exactly one. */
    CHECK(imu_hs_log_seq() >= seq_open + 3u,
          "the disarm wrote the published sectors instead of abandoning them");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
