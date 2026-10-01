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
 * @file vayu_board.h
 * @brief Board facts for navixdev -- the F401 development flight controller.
 *
 * @details
 * Every macro here would be wrong on a different PCB and is right nowhere
 * else. This is the only header vayu ships that is allowed to name silicon on
 * a consumer's behalf; see board/README.md for the contract every board must
 * satisfy, and docs/analysis/navhal-hardware-logic-separation.md for why these
 * left variables.h.
 *
 * Only the narrow port TYPE headers are included, never navhal.h: naming a pin
 * must not cost the caller sixteen subsystem headers and their function
 * declarations. This is the same include set NavHAL's own board files use.
 */
#ifndef VAYU_BOARD_NAVIXDEV_H
#define VAYU_BOARD_NAVIXDEV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "utils/gpio_types.h"
#include "utils/i2c_types.h"
#include "utils/timer_types.h"
#include "utils/uart_types.h"
/* The IRQ vector numbers below (USART6_IRQn, DMA2_Stream7_IRQn...) are the
 * part's, not the core's, so they come from the family header rather than a
 * utils/ type header. Naming a vector is exactly what a board file is for. */
#include "family/interrupt_reg.h"

/** Identity, and the NavHAL board this one must be built against.
 *  firmware/board/boards.cmake checks the second against navhal.config. */
#define VAYU_BOARD_NAME "navixdev"
#define VAYU_BOARD_NAVHAL_BOARD "nucleo_f401re"

/* ---- Status indicators --------------------------------------------------
 * Three discrete LEDs and a buzzer. NOT the NavHAL LED_BUILTIN: on the
 * nucleo_f401re that name is PA05, which on this board is the BUZZER. Do not
 * substitute one for the other. */
#define BOARD_LED_BLUE GPIO_PB12
#define BOARD_LED_GREEN GPIO_PB13
#define BOARD_LED_RED GPIO_PB14
#define BOARD_BUZZER GPIO_PA05

/* ---- Sensor bus ---------------------------------------------------------
 * I2C1 carries the IMU, the barometer and the rangefinder. Every device on it
 * rides the IMU's single-owner DMA loop -- nothing drives this bus
 * independently (driver/i2c_manager.h). */
#define BOARD_I2C_BUS HAL_I2C_1
#define BOARD_I2C_SCL GPIO_PB08
#define BOARD_I2C_SDA GPIO_PB09
/** I2C1->DR, the DMA peripheral address. F4 memory map; an H7 part moves it. */
#define BOARD_I2C_DR_ADDR (uint32_t)(0x40005400 + 0x10)

/** 7-bit address of the BMX160 on BOARD_I2C_BUS (SDO strap). */
#define BOARD_IMU_I2C_ADDR 0x68

/* ---- Motors -------------------------------------------------------------
 * Four ESC outputs on one advanced-control timer, M1..M4 in mixer order.
 * The timer is shared: it is initialised once for the group, not per channel
 * (fault line F7). */
#define BOARD_ESC_TIMER TIM1
#define BOARD_ESC_M1_PIN GPIO_PA08 /* TIM1_CH1 */
#define BOARD_ESC_M2_PIN GPIO_PA09 /* TIM1_CH2 */
#define BOARD_ESC_M3_PIN GPIO_PA10 /* TIM1_CH3 */
#define BOARD_ESC_M4_PIN GPIO_PA11 /* TIM1_CH4 */
/* Which alternate function routes the timer to those pins. A package fact:
 * on STM32F4 TIM1/TIM2 are AF1 and TIM3/4/5 are AF2, and another part may
 * map it differently. The ESC driver used to guess this from the timer
 * instance (F8). */
#define BOARD_ESC_AF HAL_GPIO_AF1

/* ---------------------------------------------------------------------------
 * Battery sense
 *
 * A resistor divider from the pack to an ADC pin. 37k on the high side, 4k on
 * the low side, tap to PA0 (ADC1_IN0): ratio 4/41, so a full 3S at 12.6 V
 * reads 1.23 V and the 3.3 V full scale corresponds to 33.8 V -- headroom to
 * 8S, at the cost of using only 37% of the range on 3S.
 *
 * PA0 is also SYS_WKUP1. Nothing here drives it, but a pull-down fitted for
 * wake-up would sit in parallel with the low-side resistor and read the pack
 * low, which is why BOARD_VBAT_SCALE is a board fact and not a constant in the
 * driver: measure the real ratio and correct it here.
 * -------------------------------------------------------------------------*/
#define BOARD_VBAT_PIN GPIO_PA00
#define BOARD_VBAT_ADC_CHANNEL 0u /* PA0 is ADC1_IN0 */

/* Pack volts per ADC count, MEASURED -- not derived from the resistor values.
 *
 * The nominal 37k/4k ratio predicts 0.0976 and the bench measures 0.071: a 3S
 * at 10.83 V lands on 948 counts, not the 1310 the arithmetic wants. So one of
 * the resistors is not what it says, or something loads the low side -- PA0 is
 * SYS_WKUP1 and a ~9.4k pull-down would do exactly this.
 *
 * Rather than model that, calibrate it. One constant absorbs the divider, the
 * resistor tolerances, the real VREF (the 3.3 V rail is a regulator, not a
 * reference) and any stray load, and unlike a computed ratio it can be checked
 * against a multimeter in one reading. Recalibrate if the divider is rebuilt:
 *
 *     BOARD_VBAT_VOLTS_PER_COUNT = (pack volts) / (battery_last_counts())
 */
#define BOARD_VBAT_VOLTS_PER_COUNT 0.011424f

/* Below this, the sensed rail is not carrying a usable pack voltage.
 *
 * NOT a battery-detect. The divider senses the main power RAIL, so it cannot
 * tell a pack from a bench supply or a BEC feeding the same rail -- measured on
 * this board reading 10.69 V with no battery connected at all. What it can tell
 * is whether the rail is energised, which is the question that matters.
 *
 * A disconnected pack does NOT read zero. The divider keeps measuring that
 * rail, and with nothing driving it the rail sits on residual charge in
 * the ESC bulk capacitors with nothing to drain it: measured 2.65 V on this
 * board (232 counts), steady, not drifting. So the reading is honest -- it is
 * just not a battery, and reporting it as one invited exactly the wrong
 * conclusion once already.
 *
 * 5.0 V is chosen to sit in a wide dead band rather than close to either side:
 * ~1.9x above the measured residual, and far below anything flyable, since a 3S
 * is scrap by 7.5 V (2.5 V/cell) and the board's own regulator gives up before
 * that. Nothing here senses cell count, so this is a PRESENCE floor and not a
 * low-battery warning -- a 2S would pass it.
 *
 * Re-measure with the pack disconnected if the divider is rebuilt: the residual
 * depends on what is on the rail, not on the resistors. */
#define BOARD_VBAT_PRESENT_MIN_V 5.0f

/* ---- Timers -------------------------------------------------------------
 * The general-purpose timer behind driver/timer_callbacks.h: the sub-
 * millisecond scheduler the 1 ms SysTick cannot serve (IMU pacing, the HF
 * monotonic tick). Separate from BOARD_ESC_TIMER, which is PWM only. */
#define BOARD_HF_TIMER TIM5
#define BOARD_HF_TIMER_IRQ TIM5_IRQn

/* ---- Serial links -------------------------------------------------------
 * Telemetry goes out the ESP bridge on USART6; the RC receiver drives USART2.
 * SITL inverts this pair -- see docs/scratch/resource-ownership-map.md.
 *
 * The IRQ vectors belong with them: comm/irq_owner.h wants to know which
 * vector a link uses, and that is a board fact, not a transport one. */
#define BOARD_TELEMETRY_UART HAL_UART_6
#define BOARD_TELEMETRY_UART_IRQ USART6_IRQn
#define BOARD_TELEMETRY_TX_DMA_IRQ DMA2_Stream7_IRQn

#define BOARD_RC_UART HAL_UART_2
#define BOARD_RC_UART_IRQ USART2_IRQn
#define BOARD_RC_TX_DMA_IRQ DMA1_Stream6_IRQn

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* VAYU_BOARD_NAVIXDEV_H */
