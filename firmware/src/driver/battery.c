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
/** @file driver/battery.c @brief Pack voltage (see driver/battery.h). */

#include "driver/battery.h"

#include "family/rcc_reg.h"
#include "hub/hub.h"   /* battery_publish */
#include "sys/clock.h" /* vayu_clock_cycles */
#include "vaios.h"     /* v_delay */
#include "navhal.h"
#include "vayu_board.h"

#include <stdint.h>

/* ADC1 on the STM32F4. NavHAL has no adc_reg.h, so the handful of registers
 * this needs are declared here rather than inventing a whole family header for
 * one peripheral one driver uses. See the note in driver/battery.h: this is a
 * placeholder for hal_adc, not a design. Offsets are RM0368 §11.12. */
#define ADC1_BASE_ADDR 0x40012000u
typedef struct {
  volatile uint32_t SR;    /* 0x00 status                                    */
  volatile uint32_t CR1;   /* 0x04 control 1                                 */
  volatile uint32_t CR2;   /* 0x08 control 2                                 */
  volatile uint32_t SMPR1; /* 0x0C sample time, channels 10-18              */
  volatile uint32_t SMPR2; /* 0x10 sample time, channels 0-9                */
  volatile uint32_t JOFR[4];
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1; /* 0x2C regular sequence 1 (length)               */
  volatile uint32_t SQR2;
  volatile uint32_t SQR3; /* 0x34 regular sequence 3 (first conversion)     */
  volatile uint32_t JSQR;
  volatile uint32_t JDR[4];
  volatile uint32_t DR; /* 0x4C regular data                                */
} adc_regs_t;
#define ADC1_REGS ((adc_regs_t *)ADC1_BASE_ADDR)

#define ADC_SR_EOC (1u << 1)       /* end of conversion                       */
#define ADC_CR2_ADON (1u << 0)     /* converter on                            */
#define ADC_CR2_SWSTART (1u << 30) /* start a regular conversion            */
#define RCC_APB2ENR_ADC1EN (1u << 8)

/* The prescaler is in the COMMON block, shared by ADC1/2/3 -- not in ADC1's own
 * registers, which is easy to miss. PCLK2 is 42 MHz here (clock.c sets APB2 to
 * SYSCLK/2), so the reset default of /2 gives 21 MHz and is already inside the
 * F401's 36 MHz ceiling. Set it anyway: it is a silicon limit, not a
 * coincidence, and a future clock tree that raises PCLK2 to 84 would silently
 * overshoot it. /4 is in spec for anything up to 144 MHz. */
#define ADC_COMMON_CCR_ADDR 0x40012304u
#define ADC_CCR_ADCPRE_DIV4 (1u << 16)

/* Sample time for channel 0: 480 cycles, the longest available. The divider is
 * a 37k/4k pair, so its source impedance is ~3.6k -- far above what a short
 * sample can charge the sample-and-hold through, and the symptom of getting
 * that wrong is a reading that sags toward zero. Time is free here: one
 * conversion at 480 cycles is ~19 us at 25 MHz ADCCLK, sampled at 10 Hz. */
#define ADC_SMP_480_CYCLES 7u

/* A conversion takes ~20 us; this is a loop bound, not a timeout to tune. */
#define ADC_EOC_SPINS 20000u

/* Conversions averaged per reading.
 *
 * A single conversion spread 1038..1123 counts -- nearly a volt peak to peak --
 * against a multimeter that did not move, with the motors STOPPED and a 10 uF
 * cap on the divider. So it is not the pack, and it is not PWM getting through
 * the filter: it is per-conversion noise, mostly on the 3.3 V rail this uses as
 * its reference. Averaging is the only thing that touches that.
 *
 * The cap's corner is ~4 Hz (37 ms into the divider's 3.7k), so these sixteen
 * conversions all land on essentially one point of the filtered signal -- which
 * is the intent. They average the CONVERTER's noise, not the pack's. */
#define BATTERY_OVERSAMPLE 16u

static volatile float s_last_volts;
static volatile uint16_t s_last_counts;
static uint8_t s_ready;

/* Bring-up instrumentation. A reading of zero counts is ambiguous -- a pin at
 * 0 V and a conversion that never ran look identical in the value alone -- and
 * on the first attempt that ambiguity cost a debug cycle. These are readable
 * over SWD and go out with the telemetry, so the next failure names itself. */
static volatile int32_t s_init_rc = -99; /* vayu_status_t from battery_init  */
static volatile int32_t s_read_rc = -99; /* ...from the last read            */
static volatile uint32_t s_last_sr;      /* ADC1->SR at the end of that read */
static volatile uint32_t s_last_cr2;     /* ADC1->CR2 likewise               */
static volatile uint32_t s_timeouts;     /* conversions that never finished  */

/** @implements SNS-BATT-001 */
vayu_status_t battery_init(void) {
  /* Pin first: analog mode disconnects the digital input buffer, without which
   * the pin both loads the divider and leaks. */
  hal_gpio_enable_clock(BOARD_VBAT_PIN);
  hal_gpio_set_mode(BOARD_VBAT_PIN, HAL_GPIO_MODE_ANALOG, HAL_GPIO_PULL_NONE);

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* Prescaler before anything else touches the converter. */
  volatile uint32_t *ccr = (volatile uint32_t *)ADC_COMMON_CCR_ADDR;
  *ccr = (*ccr & ~(3u << 16)) | ADC_CCR_ADCPRE_DIV4;

  adc_regs_t *a = ADC1_REGS;
  a->CR1 = 0u;  /* 12-bit, no scan, no interrupts */
  a->CR2 = 0u;  /* single conversion, right-aligned, software trigger */
  a->SQR1 = 0u; /* sequence length 1 */
  a->SQR3 = (uint32_t)BOARD_VBAT_ADC_CHANNEL;
  a->SMPR2 = (uint32_t)ADC_SMP_480_CYCLES
             << (3u * BOARD_VBAT_ADC_CHANNEL); /* channels 0-9, 3 bits each */

  a->CR2 |= ADC_CR2_ADON;
  /* The converter needs a moment out of power-down (tSTAB, ~3 us). Spin rather
   * than delay: this runs before the scheduler. */
  for (volatile uint32_t i = 0; i < 2000u; i++) {
  }
  if ((a->CR2 & ADC_CR2_ADON) == 0u) {
    s_init_rc = (int32_t)VAYU_ERR_FAULT;
    return VAYU_ERR_FAULT;
  }
  s_ready = 1u;
  s_init_rc = (int32_t)VAYU_OK;
  return VAYU_OK;
}

/** @implements SNS-BATT-001 */
vayu_status_t battery_read_volts(float *volts_out) {
  if (volts_out == NULL || !s_ready) {
    return VAYU_ERR_INVALID;
  }
  adc_regs_t *a = ADC1_REGS;

  uint32_t acc = 0u;
  for (uint32_t n = 0; n < BATTERY_OVERSAMPLE; n++) {
    a->SR &= ~ADC_SR_EOC; /* EOC is cleared by writing 0, or by reading DR */
    a->CR2 |= ADC_CR2_SWSTART;

    uint32_t spins = 0;
    while ((a->SR & ADC_SR_EOC) == 0u) {
      if (++spins >= ADC_EOC_SPINS) {
        s_last_sr = a->SR;
        s_last_cr2 = a->CR2;
        s_timeouts++;
        s_read_rc = (int32_t)VAYU_ERR_TIMEOUT;
        return VAYU_ERR_TIMEOUT;
      }
    }
    acc += (uint32_t)(a->DR & 0x0FFFu); /* reading DR clears EOC */
  }
  /* Rounded, not truncated: half a count is 6 mV of systematic bias, small but
   * free to avoid. */
  const uint16_t counts =
      (uint16_t)((acc + BATTERY_OVERSAMPLE / 2u) / BATTERY_OVERSAMPLE);
  /* Straight from counts. Going via a nominal VREF and a nominal divider ratio
   * gave a reading 9% off a multimeter and invited two wrong conclusions about
   * the battery before the scale was checked. */
  const float pack_v = (float)counts * BOARD_VBAT_VOLTS_PER_COUNT;

  s_last_counts = counts;
  s_last_volts = pack_v;
  s_last_sr = a->SR;
  s_last_cr2 = a->CR2;
  s_read_rc = (int32_t)VAYU_OK;
  *volts_out = pack_v;
  return VAYU_OK;
}

/** @noreq observability accessor */
float battery_last_volts(void) { return s_last_volts; }
/** @noreq observability accessor */
uint16_t battery_last_counts(void) { return s_last_counts; }
/** @noreq bring-up: how init and the last read ended, and the raw registers */
void battery_debug(int32_t *init_rc, int32_t *read_rc, uint32_t *sr,
                   uint32_t *cr2, uint32_t *timeouts) {
  if (init_rc)
    *init_rc = s_init_rc;
  if (read_rc)
    *read_rc = s_read_rc;
  if (sr)
    *sr = s_last_sr;
  if (cr2)
    *cr2 = s_last_cr2;
  if (timeouts)
    *timeouts = s_timeouts;
}

/* ---------------------------------------------------------------------------
 * Acquisition
 *
 * Its own task rather than a ride-along: there is no bus to share and nothing
 * to interleave with, and at 4 Hz the cost is a conversion and a publish. A
 * pack voltage that moves meaningfully faster than this is already failing.
 *
 * The task publishes to the hub and nothing else. Telemetry and the recorder
 * read it from there, so no gated section ever names this driver.
 * -------------------------------------------------------------------------*/
#define BATTERY_SAMPLE_PERIOD_MS 250u

/** @implements SNS-BATT-001 */
void battery_task(void *args) {
  (void)args;

  while (1) {
    float v = 0.0f;
    const bool ok = (battery_read_volts(&v) == VAYU_OK);

    /* Published either way. An unplugged pack reads a low but real voltage and
     * a failed conversion publishes valid=0, so a consumer can tell "the pack
     * is flat" from "the measurement is broken" -- which reading the value
     * alone never could. */
    battery_sample_t s = {.volts = ok ? v : 0.0f,
                          .t_cyc = vayu_clock_cycles(),
                          .counts = battery_last_counts(),
                          .valid = ok ? 1u : 0u,
                          .instance = 0u};
    battery_publish(&s);

    v_delay(MS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS));
  }
}
