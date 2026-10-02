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
/* Battery divider calibration on the card (storage/battery_calib.c).
 *
 * The failure this guards is quiet. A wild hover estimate makes the aircraft
 * behave oddly; a wild divider scale just makes the FC report a voltage that is
 * wrong by a constant factor, which looks entirely plausible and is believed --
 * and the whole point of the battery instrument is to be believed. So every way
 * a bad record can reach the loader has to leave the compiled defaults standing.
 *
 * The file is also a wire format between two programs: tools/calib/battery_calib.py
 * writes it and the firmware reads it. Its size and layout are asserted here
 * because a silent struct change on either side reads garbage as a calibration.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/battery.h" /* BOARD_VBAT_* defaults via vayu_board.h */
#include "storage/battery_calib.h"
#include "storage/fs_owner.h"

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

/* Arbitrary bytes into the store, bypassing battery_calib_save's validation --
 * that is the point: these are records save would never write, but a corrupt
 * card, an interrupted write or a mis-built host tool can leave them. */
static bool put_raw(const void *bytes, uint32_t len) {
  if (!fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, BATTERY_CALIB_PATH, 0,
                                 bytes, len)) {
    return false;
  }
  fs_owner_pump();
  return true;
}

static bool put_record(uint32_t magic, uint16_t version, float vpc,
                       float offset) {
  uint8_t rec[16];
  const uint16_t pad = 0u;
  memcpy(rec, &magic, 4);
  memcpy(rec + 4, &version, 2);
  memcpy(rec + 6, &pad, 2);
  memcpy(rec + 8, &vpc, 4);
  memcpy(rec + 12, &offset, 4);
  return put_raw(rec, sizeof rec);
}

static void clear_store(void) {
  (void)fs_owner_unlink(BATTERY_CALIB_PATH);
  fs_owner_pump();
}

/* Load into values seeded with the compiled defaults, as battery_task does. */
static bool load(float *vpc, float *off) {
  *vpc = BOARD_VBAT_VOLTS_PER_COUNT;
  *off = BOARD_VBAT_OFFSET_COUNTS;
  return battery_calib_load(vpc, off);
}

#define IS_COMPILED(vpc, off)                                                  \
  ((vpc) == BOARD_VBAT_VOLTS_PER_COUNT && (off) == BOARD_VBAT_OFFSET_COUNTS)

int main(void) {
  printf("== battery divider calibration ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_batcal_test", 1);
  fs_owner_init();
  clear_store();

  float vpc = 0.0f, off = 0.0f;

  printf("  [1] no file on the card -> the compiled defaults stand\n");
  CHECK(!load(&vpc, &off), "load reports it applied nothing");
  CHECK(IS_COMPILED(vpc, off), "and left both terms untouched");

  printf("  [2] a good record is applied\n");
  {
    CHECK(battery_calib_save(0.011476f, 5.24f), "save accepts a sane pair");
    fs_owner_pump();
    CHECK(load(&vpc, &off), "load reports it applied the file");
    CHECK(fabsf(vpc - 0.011476f) < 1e-7f, "scale came from the card");
    CHECK(fabsf(off - 5.24f) < 1e-4f, "offset came from the card");
    /* The point of the exercise: the card's numbers, not the header's. */
    CHECK(!IS_COMPILED(vpc, off), "the compiled values were NOT used");
  }

  printf("  [3] every malformed record leaves the defaults alone\n");
  {
    clear_store();
    CHECK(put_record(0xDEADBEEFu, BATTERY_CALIB_VERSION, 0.0114f, 0.0f),
          "wrote a bad-magic record");
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off), "bad magic refused");

    clear_store();
    CHECK(put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION + 1u, 0.0114f,
                     0.0f),
          "wrote a future-version record");
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
          "an unknown version is refused, not guessed at");

    clear_store();
    { /* short record: an interrupted write that got nowhere */
      const uint8_t half[8] = {0x42, 0x43, 0x41, 0x4C, 1, 0, 0, 0};
      CHECK(put_raw(half, sizeof half), "wrote a truncated record");
      CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off), "short read refused");
    }

    clear_store();
    { /* The short record that MATTERS: a write interrupted after the scale but
       * before the offset. Everything present is valid and in band, so only the
       * length tells you the offset was never written -- without that check this
       * is accepted with a silent offset of zero, which is a plausible-looking
       * calibration built from half a file. */
      uint8_t partial[12];
      const uint32_t magic = BATTERY_CALIB_MAGIC;
      const uint16_t ver = BATTERY_CALIB_VERSION, pad = 0u;
      const float good_vpc = 0.011476f;
      memcpy(partial, &magic, 4);
      memcpy(partial + 4, &ver, 2);
      memcpy(partial + 6, &pad, 2);
      memcpy(partial + 8, &good_vpc, 4);
      CHECK(put_raw(partial, sizeof partial),
            "wrote a record cut off after the scale");
      CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
            "a record missing only the offset is refused, not defaulted to 0");
    }

    clear_store();
    { /* A LONGER file is accepted on its first 16 bytes, and that is deliberate
       * rather than an oversight. The loader asks for exactly sizeof(record), so
       * a 16-byte read cannot tell a 24-byte file from a 16-byte one -- catching
       * it would mean reading a byte past the record purely to reject.
       *
       * The version field is the forward-compatibility mechanism instead: a
       * future longer record carries version 2 and is refused by the check
       * above. A longer record still claiming version 1 is corruption whose
       * prefix is a valid in-band calibration, and using it is no worse than
       * refusing. This asserts the real behaviour so the limit is on the record
       * rather than discovered later. */
      uint8_t over[24] = {0};
      const uint32_t magic = BATTERY_CALIB_MAGIC;
      const uint16_t ver = BATTERY_CALIB_VERSION;
      const float good_vpc = 0.011476f, good_off = 5.24f;
      memcpy(over, &magic, 4);
      memcpy(over + 4, &ver, 2);
      memcpy(over + 8, &good_vpc, 4);
      memcpy(over + 12, &good_off, 4);
      CHECK(put_raw(over, sizeof over), "wrote an oversized record");
      CHECK(load(&vpc, &off) && fabsf(vpc - good_vpc) < 1e-7f,
            "a longer record is read on its prefix; version gates the format");

      clear_store();
      memcpy(over + 4, (const uint16_t[]){BATTERY_CALIB_VERSION + 1u}, 2);
      CHECK(put_raw(over, sizeof over), "wrote a longer FUTURE-version record");
      CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
            "and that is what the version field is for");
    }
  }

  printf("  [4] out-of-band values are refused, not clamped\n");
  {
    /* Clamping would be worse than refusing: it would produce a reading that is
     * wrong but in range, which is indistinguishable from a good one. */
    const float bad_vpc[] = {0.0f, -0.0114f, BATTERY_CALIB_VPC_MIN,
                             BATTERY_CALIB_VPC_MAX, 1.0f};
    for (unsigned i = 0; i < sizeof bad_vpc / sizeof bad_vpc[0]; i++) {
      clear_store();
      (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, bad_vpc[i],
                       0.0f);
      CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
            "a scale outside the band is refused");
    }
    clear_store();
    (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, 0.0114f,
                     BATTERY_CALIB_OFFSET_ABS_MAX);
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
          "an offset at the band edge is refused");
    clear_store();
    (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, 0.0114f,
                     -4000.0f);
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
          "a wildly negative offset is refused");

    /* NaN fails every comparison, so a band written as "is not outside" would
     * let it through and make every subsequent reading NaN. */
    clear_store();
    (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, NAN, 0.0f);
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off), "a NaN scale is refused");
    clear_store();
    (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, 0.0114f, NAN);
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
          "a NaN offset is refused");
    clear_store();
    (void)put_record(BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION, INFINITY,
                     0.0f);
    CHECK(!load(&vpc, &off) && IS_COMPILED(vpc, off),
          "an infinite scale is refused");
  }

  printf("  [5] save refuses what load would refuse\n");
  {
    /* Otherwise the FC can persist a file it will then reject at every boot,
     * reporting a calibration failure forever with no way to see why. */
    CHECK(!battery_calib_save(0.0f, 0.0f), "save rejects a zero scale");
    CHECK(!battery_calib_save(NAN, 0.0f), "save rejects NaN");
    CHECK(!battery_calib_save(0.0114f, 9999.0f), "save rejects a wild offset");
    CHECK(battery_calib_save(0.0114f, -5.0f), "but accepts a negative offset");
  }

  printf("  [6] the file layout is a contract with the host tool\n");
  {
    /* tools/calib/battery_calib.py packs '<IHHff'. If either side changes shape
     * the other reads garbage as a calibration, so the size is pinned here. */
    clear_store();
    CHECK(battery_calib_save(0.012345f, -7.5f), "saved a known pair");
    fs_owner_pump();
    uint8_t raw[32] = {0};
    const int n = fs_owner_read_at(BATTERY_CALIB_PATH, 0, raw, sizeof raw);
    CHECK(n == 16, "the record is exactly 16 bytes");
    uint32_t magic = 0;
    uint16_t ver = 0;
    float vv = 0.0f, oo = 0.0f;
    memcpy(&magic, raw, 4);
    memcpy(&ver, raw + 4, 2);
    memcpy(&vv, raw + 8, 4);
    memcpy(&oo, raw + 12, 4);
    CHECK(magic == BATTERY_CALIB_MAGIC, "magic at offset 0");
    CHECK(ver == BATTERY_CALIB_VERSION, "version at offset 4");
    CHECK(fabsf(vv - 0.012345f) < 1e-7f, "volts_per_count at offset 8");
    CHECK(fabsf(oo + 7.5f) < 1e-4f, "offset_counts at offset 12");
  }

  printf("  [7] the offset is what a one-term scale could not express\n");
  {
    /* volts = (counts - offset) * vpc. With a real zero error, a pure scale is
     * wrong at BOTH ends and worst where it matters least to the arithmetic but
     * most to a pilot -- near empty. These are the numbers the fit produced from
     * the bench points. */
    const float k = 0.011476f, z = 5.24f;
    const float at_full = (1091.0f - z) * k;
    const float at_low = (698.0f - z) * k;
    CHECK(fabsf(at_full - 12.460f) < 0.01f, "fits the charged point");
    CHECK(fabsf(at_low - 7.950f) < 0.01f, "fits the low point");
    /* Same points with offset forced to zero: the error a single term leaves. */
    const float naive_low = 698.0f * k;
    CHECK(fabsf(naive_low - 7.950f) > 0.04f,
          "a scale-only fit is visibly wrong at the low end");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
