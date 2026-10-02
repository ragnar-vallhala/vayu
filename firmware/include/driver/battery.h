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
 * @file driver/battery.h
 * @brief Pack voltage from a resistor divider on an ADC pin.
 *
 * The FC had no idea what its battery was doing. That is not a comfort
 * question: a hover collective of 0.75 where 0.38 was once measured is either
 * a dead pack, a different cell count, or lost thrust for some other reason,
 * and NOTHING in a flight recording could tell those apart. This is the
 * instrument that makes that answerable.
 *
 * WHY THE REGISTERS ARE HERE. NavHAL has no ADC driver -- no hal_adc, no
 * adc_reg.h, nothing in its Kconfig -- so this file pokes ADC1 directly. That
 * is legal where it sits (silicon is what driver/ is for, and the layering gate
 * exempts it) but it is the wrong long-term home: an ADC belongs beside
 * hal_i2c and hal_uart, not in one vehicle's battery monitor. Filed as such;
 * when hal_adc lands, everything below the divider maths should move to it and
 * this file should shrink to a scale factor and a getter.
 *
 * Deliberately polled and slow. A pack voltage that moves meaningfully inside
 * 100 ms is a pack that is already failing, and the whole reading -- sixteen
 * conversions averaged -- costs under a millisecond with nothing to schedule
 * around it.
 */
#ifndef VAYU_DRIVER_BATTERY_H
#define VAYU_DRIVER_BATTERY_H

#include "hub/sample.h" /* BATTERY_F_* live with the sample they describe */
#include "vayu_board.h" /* BOARD_VBAT_PRESENT_MIN_V */
#include "vayu_status.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Bring up the ADC and the sense pin. Safe to call before the scheduler.
 *
 * @return VAYU_OK, or VAYU_ERR_FAULT if the ADC did not come out of reset.
 */
vayu_status_t battery_init(void);

/**
 * One conversion, converted to pack volts through the board's divider.
 *
 * @param volts_out written only on success.
 * @return VAYU_OK, or VAYU_ERR_TIMEOUT if the conversion did not complete.
 */
vayu_status_t battery_read_volts(float *volts_out);

/** Last value battery_read_volts() succeeded with, or 0.0f before the first.
 *  Lock-free single float read -- safe from any context. */
float battery_last_volts(void);

/**
 * The divider calibration currently in force: volts per ADC count, and the
 * zero-error offset in counts. Reflects 0:batcal.bin once battery_task has
 * loaded it, the compiled board defaults before that.
 *
 * Worth exposing because a voltage is only interpretable alongside the scale it
 * was produced with -- a log that records one without the other cannot be
 * re-derived after a recalibration.
 */
void battery_calibration(float *volts_per_count, float *offset_counts);

/** Raw last ADC count, for divider bring-up and for telling "reads zero"
 *  apart from "reads nothing". */
uint16_t battery_last_counts(void);

/**
 * Is the sensed rail carrying a usable pack voltage?
 *
 * NOT a battery-detect, and the name is the closest short word rather than a
 * promise. The divider sees the main power RAIL, so a bench supply or a BEC on
 * that rail passes this exactly as a pack does -- measured at 10.69 V on this
 * board with no battery connected. One ADC pin cannot distinguish them.
 *
 * What it does exclude is the de-energised case, which is the defect this
 * exists for: with nothing driving the rail it holds residual charge from the
 * ESC capacitors (2.65 V measured) and converts perfectly, so a reading alone
 * would be published as a pack voltage and believed.
 *
 * Nor is it a health test: nothing senses cell count, so a dangerously flat
 * pack passes and its voltage is to be trusted.
 */
static inline bool battery_pack_present(float volts) {
  return volts >= BOARD_VBAT_PRESENT_MIN_V;
}

/* Acquisition cadence. In the header because BATTERY_PRESENT_SETTLE_SAMPLES is
 * a count of these and means nothing without it -- a reviewer checking the
 * settle window is ~2 s needs both numbers in one place. A pack voltage that
 * moves meaningfully faster than this is already failing. */
#define BATTERY_SAMPLE_PERIOD_MS 250u

/* Consecutive samples the rail must read above the floor before PRESENT is
 * asserted. ~2 s at BATTERY_SAMPLE_PERIOD_MS, which is what PX4 waits before it
 * trusts a battery: LITHIUM_BATTERY_RECOGNITION_VOLTAGE gates its _connected,
 * but the rest of PX4 consumes _battery_initialized, which is _connected held
 * for 2 s. Its comment gives the reason and it applies here identically --
 * "avoid relying on a voltage sample from the rising edge".
 *
 * Plugging a pack in does not step the rail; it charges the ESC capacitors
 * through whatever is in the path. Without this, one 250 ms sample taken
 * part-way up that curve reads above the floor and is published as a believable
 * pack voltage -- a phantom number of exactly the kind the floor exists to
 * stop, just a brief one. */
#define BATTERY_PRESENT_SETTLE_SAMPLES 8u

/**
 * Hold PRESENT back until the rail has been plausible long enough to believe,
 * given how many consecutive samples -- including this one -- have read above
 * the floor.
 *
 * Only PRESENT is withheld. CONVERTED and the fault bits describe THIS
 * acquisition and are true the moment they are seen, so debouncing them would
 * misreport the converter; it is only "a pack is on the rail" that a rising edge
 * can fake.
 *
 * Pure, so the settling rule is testable without waiting two seconds of real
 * time or owning an ADC.
 */
static inline uint8_t battery_flags_settled(uint8_t flags,
                                            uint32_t samples_above) {
  if ((flags & BATTERY_F_PRESENT) != 0u &&
      samples_above < BATTERY_PRESENT_SETTLE_SAMPLES) {
    return (uint8_t)(flags & (uint8_t)~BATTERY_F_PRESENT);
  }
  return flags;
}

/**
 * Build the BATTERY_F_* mask for one acquisition.
 *
 * Pure, and separate from battery_task() so it can be tested at all -- the task
 * is a while(1) and the ADC is not there on a host. @see hub/sample.h for what
 * each bit means and why PRESENT and CONVERTED are not the same question.
 *
 * @param rc     what battery_read_volts() returned.
 * @param volts  its reading; ignored unless rc is VAYU_OK.
 * @param counts the raw count behind it.
 */
static inline uint8_t battery_flags_from(vayu_status_t rc, float volts,
                                         uint16_t counts) {
  if (rc == VAYU_ERR_TIMEOUT) {
    return BATTERY_F_TIMEOUT;
  }
  if (rc != VAYU_OK) {
    /* VAYU_ERR_INVALID from a read is only reachable when init failed: that is
     * the one thing that leaves the driver not ready. */
    return BATTERY_F_INIT_FAIL;
  }
  uint8_t flags = BATTERY_F_CONVERTED;
  if (battery_pack_present(volts)) {
    flags |= BATTERY_F_PRESENT;
  }
  if (counts >= BATTERY_COUNTS_FULL_SCALE) {
    flags |= BATTERY_F_RAILED;
  }
  return flags;
}

/** Acquisition task: one conversion every 250 ms, published to the hub.
 *  Started by the composition root; nothing else should sample the ADC. */
void battery_task(void *args);

/** Bring-up detail: how init and the last read ended, plus the raw ADC status
 *  and control words and a timeout count. Any pointer may be NULL. */
void battery_debug(int32_t *init_rc, int32_t *read_rc, uint32_t *sr,
                   uint32_t *cr2, uint32_t *timeouts);

#endif /* VAYU_DRIVER_BATTERY_H */
