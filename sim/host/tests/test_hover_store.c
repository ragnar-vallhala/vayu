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
 * @file sim/host/tests/test_hover_store.c
 * @brief The persisted hover collective (src/storage/hover_store.c).
 *
 * This file holds one float, and that float is what the next lift-off after a
 * reboot opens the throttle at. Everything below is about refusing a bad one:
 * a missing file, somebody else's file, a truncated record, a value outside
 * the band, or a NaN -- each must degrade to the compiled guess, because the
 * alternative is a wild collective on a vehicle that is about to leave the
 * ground. The happy path is one check; the rest is the guards.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/height_controller.h" /* HEIGHT_HOVER_GUESS */
#include "est/hover_estimate.h"        /* HOVER_EST_MIN / MAX */
#include "storage/fs_owner.h"
#include "storage/hover_store.h"

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

static const float GUESS = HEIGHT_HOVER_GUESS;

/* Put arbitrary bytes in the store, bypassing hover_store_save's validation --
 * that is the point, these are the records save would never write but a
 * corrupt card, an interrupted write or another program can leave behind. */
static bool put_raw(const void *bytes, uint32_t len) {
  if (!fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, HOVER_STORE_PATH, 0,
                                 bytes, len)) {
    return false;
  }
  fs_owner_pump();
  return true;
}

static bool put_record(uint32_t magic, float value) {
  uint8_t rec[8];
  memcpy(rec, &magic, 4);
  memcpy(rec + 4, &value, 4);
  return put_raw(rec, sizeof rec);
}

static void clear_store(void) {
  (void)fs_owner_unlink(HOVER_STORE_PATH);
  fs_owner_pump();
}

int main(void) {
  printf("== hover store ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_hover_test", 1);
  fs_owner_init();
  fs_owner_pump();

  printf("  [1] a value the aircraft measured survives a round trip\n");
  {
    clear_store();
    CHECK(hover_store_load(GUESS) == GUESS,
          "no file at all falls back to the compiled guess");
    CHECK(hover_store_save(0.42f), "a plausible hover is queued");
    fs_owner_pump();
    const float got = hover_store_load(GUESS);
    CHECK(got > 0.4199f && got < 0.4201f, "and comes back unchanged");
  }

  printf("  [2] the band is exclusive at both ends\n");
  {
    /* The guards are `!(v > MIN) || !(v < MAX)`, so the endpoints themselves
     * are OUT. Pinned because relaxing either to >= / <= is a one-character
     * edit that widens what the vehicle will fly. */
    CHECK(!hover_store_save(HOVER_EST_MIN), "MIN exactly is refused");
    CHECK(!hover_store_save(HOVER_EST_MAX), "MAX exactly is refused");
    CHECK(hover_store_save(HOVER_EST_MIN + 0.01f), "just inside MIN is kept");
    fs_owner_pump();
    CHECK(hover_store_save(HOVER_EST_MAX - 0.01f), "just inside MAX is kept");
    fs_owner_pump();

    /* And on the LOAD side, which is the one that matters -- a file can hold
     * an endpoint even though save would not have written it. */
    CHECK(put_record(HOVER_STORE_MAGIC, HOVER_EST_MIN),
          "a stored record at MIN is written");
    CHECK(hover_store_load(GUESS) == GUESS, "and refused on load");
    CHECK(put_record(HOVER_STORE_MAGIC, HOVER_EST_MAX),
          "a stored record at MAX is written");
    CHECK(hover_store_load(GUESS) == GUESS, "and refused on load too");
  }

  printf("  [3] NaN and infinity are refused, not compared into the band\n");
  {
    /* Written as `!(v > MIN)` rather than `v <= MIN` precisely so NaN fails:
     * every comparison against NaN is false, so the negated form rejects it
     * while the natural-looking form would let it straight through. A NaN
     * collective propagates through the whole vertical loop. */
    const float nan_v = NAN;
    const float inf_v = INFINITY;
    CHECK(!hover_store_save(nan_v), "NaN is not saved");
    CHECK(!hover_store_save(inf_v), "+inf is not saved");
    CHECK(!hover_store_save(-inf_v), "-inf is not saved");

    CHECK(put_record(HOVER_STORE_MAGIC, nan_v), "a NaN record is written");
    const float got = hover_store_load(GUESS);
    CHECK(got == GUESS, "a stored NaN falls back to the guess");
    CHECK(!isnan(got), "and nothing NaN reaches the caller");

    CHECK(put_record(HOVER_STORE_MAGIC, inf_v), "an inf record is written");
    CHECK(hover_store_load(GUESS) == GUESS, "a stored inf falls back too");
  }

  printf("  [4] a wild but finite value is refused\n");
  {
    /* The case a bare magic check would pass straight to the throttle. */
    CHECK(put_record(HOVER_STORE_MAGIC, 0.95f), "an over-range record written");
    CHECK(hover_store_load(GUESS) == GUESS, "over-range refused on load");
    CHECK(put_record(HOVER_STORE_MAGIC, 0.0f), "a zero record written");
    CHECK(hover_store_load(GUESS) == GUESS, "zero refused on load");
    CHECK(put_record(HOVER_STORE_MAGIC, -0.5f), "a negative record written");
    CHECK(hover_store_load(GUESS) == GUESS, "negative refused on load");
  }

  printf("  [5] somebody else's file is not read as ours\n");
  {
    /* In-band VALUE with the wrong magic, so only the magic check can reject
     * it -- a foreign file whose bytes happened to be out of range would let
     * the range check pass this for the wrong reason. */
    CHECK(put_record(0xDEADBEEFu, 0.42f),
          "a foreign record with an in-band value is written");
    CHECK(hover_store_load(GUESS) == GUESS, "wrong magic falls back");
  }

  printf("  [6] a truncated record is not trusted\n");
  {
    /* Two things stop this being flown: the length check, and the
     * zero-initialised record whose 0.0f the range check then refuses.
     * Removing either alone still falls back; removing both puts
     * uninitialised stack on the throttle. */
    clear_store();
    const uint32_t magic = HOVER_STORE_MAGIC;
    CHECK(put_raw(&magic, sizeof magic), "a 4-byte record is written");
    CHECK(hover_store_load(GUESS) == GUESS, "a short read falls back");
  }

  printf("  [7] a full queue loses the save, not the flight\n");
  {
    /* Documented contract: persistence is an optimisation, so a full lane
     * returns false rather than blocking the caller -- which is the vertical
     * task on a disarm. The previously stored value must survive. */
    clear_store();
    CHECK(hover_store_save(0.42f), "a good value is stored first");
    fs_owner_pump();

    int queued = 0;
    for (int i = 0; i < 64; i++) {
      if (!hover_store_save(0.30f + (float)i * 0.001f)) {
        break;
      }
      queued++;
    }
    CHECK(queued < 64, "the lane fills and save starts reporting false");
    printf("      lane accepted %d queued saves before refusing\n", queued);
    fs_owner_pump();
    const float got = hover_store_load(GUESS);
    CHECK(got > HOVER_EST_MIN && got < HOVER_EST_MAX,
          "whatever landed is still a sane value");
  }

  clear_store();
  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
