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
/* Pack-presence floor (driver/battery.h).
 *
 * The reading this exists to reject is not a fault: with no pack the divider
 * goes on measuring the battery-side rail, which sits on residual charge in the
 * ESC capacitors and converts perfectly. Measured on this board at 2.65 V.
 * Published as a pack voltage it is a phantom number, and a flight log carrying
 * `battery valid` with no battery attached is a log that asserts something
 * nobody checked.
 *
 * So what matters here is the MARGIN on both sides of the floor, not the floor
 * itself. The numbers below are measurements and pack chemistry, not taste.
 */
#include "driver/battery.h"

#include <stdio.h>
#include "vayu_status.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

int main(void) {
  printf("== battery pack presence ==\n");

  printf("  [1] the measured de-energised rail is rejected\n");
  /* 232 counts * BOARD_VBAT_VOLTS_PER_COUNT, read live over telemetry with
   * nothing driving the rail. This is THE number the floor exists to exclude:
   * residual ESC capacitor charge, which converts perfectly and is not a pack
   * voltage. */
  CHECK(!battery_pack_present(2.655f),
        "2.655 V (232 counts, de-energised rail) is not usable");
  CHECK(!battery_pack_present(0.0f), "a dead zero is absent");
  /* Margin above the residual: if the rail held nearly twice the charge it
   * still must not read as usable. The residual is not a constant -- it depends
   * on what is on the rail, not on the resistors -- so the floor is sized for
   * room, not fitted to one measurement. */
  CHECK(!battery_pack_present(4.9f),
        "nearly double the residual is still not usable");

  printf("  [2] every pack this airframe can fly on is accepted\n");
  /* Bench-measured, both against a multimeter. NOTE these assert the floor lets
   * a real pack through -- NOT that the reading proves a pack. The divider sees
   * the main rail, so a bench supply at the same voltage is indistinguishable;
   * the board read 10.9 V with no battery connected at all. */
  CHECK(battery_pack_present(12.46f), "3S at 12.46 V present");
  CHECK(battery_pack_present(10.83f), "3S at 10.83 V (3.61 V/cell) present");
  /* 3S scrap threshold, 2.5 V/cell -- below any flight, above the floor, so a
   * dangerously flat pack still reports as PRESENT and its voltage is believed.
   * If the floor ever rose past this, a pack would vanish from the log exactly
   * when it mattered most. */
  CHECK(battery_pack_present(7.5f), "a ruined 3S is present, not absent");
  /* No cell count is sensed, so the floor must not quietly encode one. */
  CHECK(battery_pack_present(7.4f),
        "a 2S passes -- this is presence, not health");

  printf("  [3] the floor sits in a dead band, not against either side\n");
  CHECK(battery_pack_present(BOARD_VBAT_PRESENT_MIN_V),
        "the floor itself counts as present");
  CHECK(BOARD_VBAT_PRESENT_MIN_V > 2.0f * 2.655f * 0.9f,
        "floor is ~2x the measured residual or better");
  CHECK(BOARD_VBAT_PRESENT_MIN_V < 7.5f,
        "floor is below the lowest voltage a real pack can show");

  printf("  [4] the mask is disjoint, so causes cannot mask each other\n");
  {
    const uint8_t bits[] = {BATTERY_F_PRESENT, BATTERY_F_CONVERTED,
                            BATTERY_F_TIMEOUT, BATTERY_F_INIT_FAIL,
                            BATTERY_F_RAILED};
    uint8_t seen = 0u;
    for (unsigned i = 0; i < sizeof bits / sizeof bits[0]; i++) {
      CHECK((bits[i] & (uint8_t)(bits[i] - 1u)) == 0u, "each flag is one bit");
      CHECK((seen & bits[i]) == 0u, "no two flags share a bit");
      seen |= bits[i];
    }
  }

  printf("  [5] a reading is believable only when measured AND present\n");
  CHECK(battery_flags_believable(BATTERY_F_PRESENT | BATTERY_F_CONVERTED),
        "measured a real pack");
  CHECK(!battery_flags_believable(BATTERY_F_CONVERTED),
        "converted but nothing there is NOT believable");
  CHECK(!battery_flags_believable(BATTERY_F_PRESENT),
        "present without a conversion is NOT believable");
  CHECK(!battery_flags_believable(0u), "an empty mask is not believable");
  /* RAILED is the case that matters, and the reason believable() cannot just
   * test the good pair: a railed count converts cleanly and lands far ABOVE the
   * presence floor, so it arrives WITH both good bits set. Trusting the pair
   * alone would publish a collapsed reference as a ~47 V pack. */
  CHECK(!battery_flags_believable(BATTERY_F_PRESENT | BATTERY_F_CONVERTED |
                                  BATTERY_F_RAILED),
        "a railed reading is NOT believable despite both good bits");
  CHECK(!battery_flags_believable(BATTERY_F_PRESENT | BATTERY_F_CONVERTED |
                                  BATTERY_F_TIMEOUT),
        "nor is any other fault bit alongside them");

  printf("  [6] the mask built for each acquisition outcome\n");
  {
    /* The bench case this whole change exists for: a clean conversion of a
     * de-energised rail. CONVERTED set, PRESENT clear -- "the measurement is
     * fine, there is no battery". The old boolean said valid=1 here. */
    CHECK(battery_flags_from(VAYU_OK, 2.655f, 232u) == BATTERY_F_CONVERTED,
          "no pack: CONVERTED alone");
    CHECK(battery_flags_from(VAYU_OK, 12.46f, 1091u) ==
              (BATTERY_F_PRESENT | BATTERY_F_CONVERTED),
          "a real pack: PRESENT|CONVERTED and no fault");
    /* A collapsed reference rails the count. It converts cleanly and lands far
     * above the presence floor, so PRESENT and CONVERTED are both set and only
     * RAILED says the number is nonsense. Without that bit this is published as
     * a ~47 V pack and believed. */
    CHECK(battery_flags_from(VAYU_OK, 46.8f, 4095u) ==
              (BATTERY_F_PRESENT | BATTERY_F_CONVERTED | BATTERY_F_RAILED),
          "railed: flagged, not silently believed");
    /* Faults report the cause and claim nothing about the pack -- the volts are
     * meaningless, so PRESENT must not be inferred from them. */
    CHECK(battery_flags_from(VAYU_ERR_TIMEOUT, 12.46f, 1091u) ==
              BATTERY_F_TIMEOUT,
          "timeout: cause only, no PRESENT from a stale reading");
    CHECK(battery_flags_from(VAYU_ERR_INVALID, 12.46f, 1091u) ==
              BATTERY_F_INIT_FAIL,
          "dead ADC: cause only");
    CHECK(!battery_flags_believable(
              battery_flags_from(VAYU_ERR_TIMEOUT, 12.46f, 1091u)),
          "and a fault is never believable");
    CHECK(!battery_flags_believable(battery_flags_from(VAYU_OK, 46.8f, 4095u)),
          "a railed acquisition is not believable end to end");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
