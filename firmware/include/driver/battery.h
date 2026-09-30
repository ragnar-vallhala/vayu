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

/** Raw last ADC count, for divider bring-up and for telling "reads zero"
 *  apart from "reads nothing". */
uint16_t battery_last_counts(void);

/** Acquisition task: one conversion every 250 ms, published to the hub.
 *  Started by the composition root; nothing else should sample the ADC. */
void battery_task(void *args);

/** Bring-up detail: how init and the last read ended, plus the raw ADC status
 *  and control words and a timeout count. Any pointer may be NULL. */
void battery_debug(int32_t *init_rc, int32_t *read_rc, uint32_t *sr,
                   uint32_t *cr2, uint32_t *timeouts);

#endif /* VAYU_DRIVER_BATTERY_H */
