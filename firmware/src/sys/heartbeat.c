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
#include "comm/comm.h"
#include "driver/indicator.h"
#include "sys/heartbeat.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "sys/types.h"
#include "vaios.h"
#include "sys/heartbeat.h"
#include "vayu_tasks.h"
/* Link-activity override on the blue LED. The router used to blink this pin
 * itself on a 10 Hz timer of its own while heartbeat drove the same pin from
 * the flight state -- two writers, last one wins (F5). Now the router only
 * reports that something arrived and this task renders it, so there is one
 * writer and the override is explicit. */
#define HEARTBEAT_ACTIVITY_MS 1000u
#define HEARTBEAT_ACTIVITY_HALF_MS 50u /* 10 Hz blink => 50 ms half-period */

static uint32_t _activity_until; /* 0 = idle, else deadline in ticks */
static uint32_t _activity_toggled;

/** @implements SYS-HMI-001 */
void heartbeat_note_link_activity(void) {
  uint32_t until = v_get_ticks() + HEARTBEAT_ACTIVITY_MS;
  _activity_until = until ? until : 1u; /* keep 0 as the idle sentinel */
}

/* True while the activity blink owns the blue LED. Wrap-safe. */
static inline bool _activity_owns_blue(uint32_t now) {
  if (_activity_until == 0u) {
    return false;
  }
  if ((int32_t)(now - _activity_until) >= 0) {
    _activity_until = 0u;
    return false;
  }
  return true;
}

/* Render the activity override. Runs on the task's 20 ms cadence, AFTER the
 * flight-state pattern, so while the window is open the blue LED is whatever
 * this decides -- one writer, and a defined precedence, which is the part the
 * two-timer version never had.
 *
 * On expiry the pin is driven low and the next state tick re-establishes the
 * pattern, up to `period` later. That is what the router's own blink did too,
 * so the visible behaviour is unchanged. */
static void _service_activity(void) {
  static bool was_active = false;
  uint32_t now = v_get_ticks();

  if (!_activity_owns_blue(now)) {
    if (was_active) {
      was_active = false;
      indicator_set(IND_LED_BLUE, false);
    }
    return;
  }

  if (!was_active) {
    was_active = true;
    _activity_toggled = now;
    indicator_set(IND_LED_BLUE, true);
    return;
  }
  if ((now - _activity_toggled) >= HEARTBEAT_ACTIVITY_HALF_MS) {
    _activity_toggled = now;
    indicator_toggle(IND_LED_BLUE);
  }
}

/* @implements SYS-HMI-001 */
static inline void _system_init(void) {
  static uint8_t _first_time = 1;
  if (_first_time) {
    _first_time = 0;
    indicator_set(IND_BUZZER, true);
    v_delay(100);
    indicator_set(IND_BUZZER, false);
  }
  indicator_toggle(IND_LED_BLUE);
}

/* @implements SYS-HMI-001 */
static inline void _system_standby(void) { indicator_toggle(IND_LED_GREEN); }

/* @implements SYS-HMI-001 */
static inline void _system_prearm(void) {
  indicator_toggle(IND_LED_GREEN);
  indicator_toggle(IND_LED_BLUE);
}

/* @implements SYS-HMI-001 */
static inline void _system_armed(void) {
  indicator_toggle(IND_LED_GREEN);
  indicator_set(IND_LED_RED, true);
}

/* @implements SYS-HMI-001 */
static inline void _system_in_air(void) {
  indicator_toggle(IND_LED_GREEN);
  indicator_toggle(IND_LED_RED);
}

/* @implements SYS-HMI-101 */
static inline void _system_failsafe(void) {
  uint32_t boot_flags = (uint32_t)system_boot_check_state_get();

  // Master Failsafe Blink (Red + Buzzer)
  indicator_toggle(IND_LED_RED);
  static uint8_t buzz_cnt = 0;
  if (++buzz_cnt % 2 == 0) {
    indicator_toggle(IND_BUZZER);
  }

  // Diagnostic: Solid Blue = Clock Mismatch
  if (boot_flags & BOOT_CHECK_SYSTEM_CLOCK_CHECK_FAIL) {
    indicator_set(IND_LED_BLUE, true);
  } else {
    indicator_set(IND_LED_BLUE, false);
  }

  // Diagnostic: Solid Green = SD Card Failure
  if (boot_flags & BOOT_CHECK_SD_CARD_CHECK_FAIL) {
    indicator_set(IND_LED_GREEN, true);
  } else {
    indicator_set(IND_LED_GREEN, false);
  }
}

/* @implements SYS-HMI-001 */
static inline void _system_esc_calib(void) {
  /* All three together, which no other state does: STANDBY is green alone,
   * PREARM green+blue, IN_AIR green+red, CALIBRATING blue+green. Without a
   * case here ESC_CALIB fell to `default:` and left every LED dark, so the one
   * state in which a disarmed aircraft drives its motors to 100% looked
   * exactly like a board that had not booted.
   *
   * Deliberately NO buzzer, unlike FAILSAFE and TERMINATED. The operator is
   * listening for the ESCs' own beeps -- that is the only confirmation they
   * saw maximum as they woke -- and sounding ours on the same tick would mask
   * the thing the procedure exists to produce. */
  indicator_toggle(IND_LED_BLUE);
  indicator_toggle(IND_LED_GREEN);
  indicator_toggle(IND_LED_RED);
}

/* @implements SYS-HMI-001 */
static inline void _system_terminated(void) {
  indicator_set(IND_LED_RED, true);
  indicator_set(IND_BUZZER, true);
}

/* @implements SYS-HMI-001 */
static inline void _run_heartbeat(channel_t *channel, uint32_t period) {
  (void)channel; /* LED heartbeat is state-driven; channel unused */
  static uint32_t last_time = 0;
  uint32_t current_time = v_get_ticks();
  if (current_time - last_time < period) {
    return;
  }
  last_time = current_time;

  static sys_state_t last_state = SYSTEM_STATE_UNINITIALIZED;
  sys_state_t current_state = system_state_get();

  if (current_state != last_state) {
    // Clear all LEDs on transition to ensure a clean slate for the new state
    indicator_set(IND_LED_BLUE, false);
    indicator_set(IND_LED_GREEN, false);
    indicator_set(IND_LED_RED, false);
    last_state = current_state;
  }

  switch (current_state) {
  case SYSTEM_STATE_UNINITIALIZED:
    // Do nothing
    break;
  case SYSTEM_STATE_INIT:
    _system_init();
    break;
  case SYSTEM_STATE_STANDBY:
    _system_standby();
    break;
  case SYSTEM_STATE_PREARM:
    _system_prearm();
    break;
  case SYSTEM_STATE_ARMED:
    _system_armed();
    break;
  case SYSTEM_STATE_IN_AIR:
    _system_in_air();
    break;
  case SYSTEM_STATE_FAILSAFE:
    _system_failsafe();
    break;
  case SYSTEM_STATE_TERMINATED:
    _system_terminated();
    break;
  case SYSTEM_STATE_CALIBRATING:
    indicator_toggle(IND_LED_BLUE);
    indicator_toggle(IND_LED_GREEN);
    break;
  case SYSTEM_STATE_ESC_CALIB:
    _system_esc_calib();
    break;
    /* No `default:`, deliberately. With one, -Wswitch cannot see a state that
     * nobody rendered -- which is how ESC_CALIB shipped with every LED dark.
     * Without it, adding a sys_state_t and forgetting the annunciator is a
     * build error under -Werror, not something discovered on a bench. */
  }
}
/* @implements SYS-HMI-001 */
#ifdef VAYU_MPU_GUARD_SELFTEST
/* Prove the MPU stack guard actually bites.
 *
 * Walks DOWNWARD from this task's current frame, byte by byte, past the bottom of
 * its 512 B block. The guard is the lowest 32 B of that block and is programmed
 * no-access, so the first write that reaches it must raise MemManage with MMFAR
 * inside [mem_block, mem_block + VAIOS_MPU_GUARD_SIZE). If it does not, the guard
 * is not armed and the only thing standing between a deep task and the heap below
 * it is the software watermark this build switched off.
 *
 * Writes below the frame on purpose, rather than recursing: the MPU faults on the
 * ADDRESS, so this reaches the guard without needing the compiler to move SP, and
 * it reaches it at a predictable byte.
 *
 * Build-gated and NOT in any default build. It halts the FC by design.
 */
static void mpu_guard_selftest(void) {
  volatile uint8_t *fp = (volatile uint8_t *)__builtin_frame_address(0);
  for (uint32_t i = 0; i < 4096u; i++) {
    fp[-(int32_t)i] = (uint8_t)i;
  }
}
#endif

void heartbeat_task(void *args) {
#ifdef VAYU_MPU_GUARD_SELFTEST
  mpu_guard_selftest();
#endif

  // Configure Physical Heartbeat
  indicator_init();
  while (g_telemetry_channel.handle == NULL) {
    v_delay(10);
  }
  uint32_t period = _HEARTBEAT_DEFAULT_TIMEPERIOD;
  if (args) {
    period = *(uint32_t *)args;
  }
  // clamp to 250ms for a max of 2Hz blink
  period = period >= 250 ? period : 250;

  while (1) {
    _run_heartbeat(&g_telemetry_channel, period);
    _service_activity();
    v_delay(20);
  }
}
