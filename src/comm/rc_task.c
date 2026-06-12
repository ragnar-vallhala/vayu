#include "vayu_tasks.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "ipc.h"
#include "navhal.h"
#include "sys/state.h"
#include "utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

#define IBUS_DMA_BUF_SIZE 128
static uint8_t ibus_dma_buf[IBUS_DMA_BUF_SIZE];
static ibus_data_t ibus_raw_data;

/* The RC watchdog + arm-precondition policy lives in src/comm/rc_safety.c
 * (DMA/UART-free, host-buildable, unit-verified). rc_mark_frame_valid,
 * rc_has_signal, rc_watchdog_step and arm_preconditions_met are declared
 * in comm/ibus.h and called below. */

/* Binary semaphore given by the USART2 idle-line ISR. The iBus receiver is on
 * USART2 (PA2/PA3, 115200) — telemetry owns USART6. The UART fills a circular
 * DMA ring; the idle line fires at the inter-frame gap, so the task blocks here
 * until a whole frame has landed instead of polling. */
static SemaphoreHandle_t _ibus_frame_sema = NULL;

/* Runs in USART2 IRQ context at a maskable priority (set by NavHAL), so the
 * ISR-safe give is correct. Coalesces to a single wake; the OVERWRITE-free
 * circular DMA keeps filling regardless. */
static void ibus_idle_isr(void) {
  if (_ibus_frame_sema != NULL) {
    int woken = 0;
    v_semaphore_give_from_isr(_ibus_frame_sema, &woken);
    (void)woken;
  }
}

/* Apply the per-frame policy to a freshly parsed iBus frame: centre-deadband
 * the roll/pitch/yaw sticks, run the arm/disarm state machine, and publish to
 * the control + telemetry queues. Runs once per COMPLETE frame, not per byte. */
static void rc_apply_frame(void) {
  rc_mark_frame_valid();

  /* Centre-deadband roll/pitch/yaw (throttle, ch index 2, is skipped). */
  for (int i = 0; i < 4; i++) {
    if (i != 2 && (ibus_raw_data.channels[i] > 1500 - RADIO_AVOID_BAND &&
                   ibus_raw_data.channels[i] < 1500 + RADIO_AVOID_BAND)) {
      ibus_raw_data.channels[i] = 1500;
    }
  }

  if (rc_throttle_failsafe_step(ibus_raw_data.channels[2])) {
    /* FlySky link-loss: throttle snapped high and held. Clamp throttle to min
     * so the failsafe value never reaches the mixer, trip FAILSAFE, and skip
     * the normal arm logic so it cannot revert the state this frame. The
     * staleness watchdog (rc_watchdog_step) remains the backstop. */
    ibus_raw_data.channels[2] = RC_THROTTLE_MIN_RAW;
    ibus_raw_data.is_failsafe = true;
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  } else {
    ibus_raw_data.is_failsafe = false;
    sys_state_t current_state = system_state_get();
    if (rc_arm_engaged(&ibus_raw_data)) {
      // Switch is UP (Armed position)
      if (current_state == SYSTEM_STATE_STANDBY &&
          arm_preconditions_met(&ibus_raw_data)) {
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
      } else if (current_state == SYSTEM_STATE_STANDBY) {
        /* Arm-switch requested ARM but a precondition failed — throttle high,
         * RC marginal, or estimator unhealthy. */
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
      }
    } else if (current_state == SYSTEM_STATE_ARMED ||
               current_state == SYSTEM_STATE_FAILSAFE) {
      // Switch is DOWN (Disarmed position)
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
    }
  }

  rc_queue_control_push(&ibus_raw_data);
  rc_queue_telemetry_push(&ibus_raw_data);
}

#ifdef VAYU_SIM
/* SITL override: when sim_rc_enabled is non-zero, rc_ibus_task skips
 * iBus parsing and uses sim_rc_channels[] as the live RC input. The
 * host SITL driver writes these directly. */
volatile uint8_t  sim_rc_enabled = 0;
volatile uint16_t sim_rc_channels[14] = {1500, 1500, 1000, 1500, 1000,
                                         1000, 1500, 1500, 1500, 1500,
                                         1500, 1500, 1500, 1500};
/* When non-zero, suppresses rc_mark_frame_valid() so SITL scenarios
 * can exercise the RC-loss watchdog without disabling the synth feed. */
volatile uint8_t sim_rc_force_loss = 0;
#endif

void rc_ibus_task(void *args) {
  (void)args;

  // Initialize iBus library
  ibus_init(&ibus_raw_data);
  rc_throttle_failsafe_reset();

  /* Created empty so the first take() blocks until the idle ISR fires. */
  _ibus_frame_sema = v_semaphore_create_binary();

  // iBus on USART2 (PA2/PA3 per Vayu PCB; 115200 baud). Telemetry owns USART6.
  hal_uart_config_t ibus_uart_cfg = {.baudrate = 115200};
  hal_uart_init(HAL_UART_2, &ibus_uart_cfg);
  hal_uart_init_dma_rx(HAL_UART_2, ibus_dma_buf, IBUS_DMA_BUF_SIZE);
  /* Wake on the inter-frame idle gap instead of polling NDTR. */
  hal_uart_attach_idle_callback(HAL_UART_2, ibus_idle_isr);

  /* Seed the watchdog clock so a cold-booted vehicle has the full
   * RC_LOSS_TIMEOUT_MS to receive the first frame before FAILSAFE. */
  rc_mark_frame_valid();

  uint16_t read_ptr = 0;

  /* Cap the blocking wait so the RC-loss watchdog keeps ticking when frames
   * stop (no idle IRQ fires once the link is dead). A quarter of the loss
   * timeout bounds FAILSAFE latency to ~RC_LOSS_TIMEOUT_MS + one tick. */
  const uint32_t watchdog_tick = MS_TO_TICKS(RC_LOSS_TIMEOUT_MS / 4);

  while (1) {
#ifdef VAYU_SIM
    if (sim_rc_enabled) {
      for (int i = 0; i < 14; i++) {
        ibus_raw_data.channels[i] = sim_rc_channels[i];
      }
      ibus_raw_data.is_failsafe = false;
      /* Run the same state-machine logic the parser-driven path runs. */
      sys_state_t current_state = system_state_get();
      for (int i = 0; i < 4; i++) {
        if (i != 2 && (ibus_raw_data.channels[i] > 1500 - RADIO_AVOID_BAND &&
                       ibus_raw_data.channels[i] < 1500 + RADIO_AVOID_BAND)) {
          ibus_raw_data.channels[i] = 1500;
        }
      }
      if (rc_arm_engaged(&ibus_raw_data)) {
        if (current_state == SYSTEM_STATE_STANDBY &&
            arm_preconditions_met(&ibus_raw_data)) {
          VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
        } else if (current_state == SYSTEM_STATE_STANDBY) {
          VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
        }
      } else {
        if (current_state == SYSTEM_STATE_ARMED ||
            current_state == SYSTEM_STATE_FAILSAFE) {
          VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
        }
      }
      if (!sim_rc_force_loss) {
        rc_mark_frame_valid();
      }
      rc_queue_control_push(&ibus_raw_data);
      rc_queue_telemetry_push(&ibus_raw_data);
      rc_watchdog_step();
      v_delay(20); /* 50 Hz, matches real iBus rate */
      continue;
    }
#endif

    /* Block until the idle ISR signals a frame gap (or the watchdog cap). */
    (void)v_semaphore_take(_ibus_frame_sema, watchdog_tick);

    /* Drain the DMA ring up to where the DMA has written so far. The write
     * index comes from the HAL (correct RX stream) — no DMA-register poke. */
    uint16_t write_ptr = read_ptr;
    if (hal_uart_dma_rx_index(HAL_UART_2, &write_ptr) != HAL_OK) {
      write_ptr = read_ptr; /* RX DMA not ready — nothing to drain this pass */
    }

    while (read_ptr != write_ptr) {
      uint8_t b = ibus_dma_buf[read_ptr];
      read_ptr = (uint16_t)((read_ptr + 1) % IBUS_DMA_BUF_SIZE);

      /* Frame assembly is the only per-byte work; the per-frame policy runs
       * once a complete, valid frame is parsed (not 32x per frame). */
      if (ibus_parse_byte(b, &ibus_raw_data)) {
        rc_apply_frame();
      }
    }

    /* Latency to FAILSAFE on loss is bounded by watchdog_tick +
     * RC_LOSS_TIMEOUT_MS. */
    rc_watchdog_step();
  }
}
