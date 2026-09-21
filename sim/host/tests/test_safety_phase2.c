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
 * @file sim/host/tests/test_safety_phase2.c
 * @brief SITL verification suite for the Phase-2 safety cluster.
 *
 * Links the real firmware safety sources (compiled into
 * libvayu_sitl_core) and drives their public contracts directly,
 * asserting the observable behaviour each requirement promises. No
 * test-only reimplementation of the logic: every function exercised
 * here is the exact one the flight tasks call.
 *
 * Time is the host monotonic clock (1 tick = 1 ms), advanced with
 * v_delay(), so the RC-loss and estimator-degraded horizons are
 * exercised against the same v_get_ticks() the firmware reads.
 *
 *   @verifies SYS-SAFE-002   RC loss -> FAILSAFE
 *   @verifies SYS-SAFE-003   sensor-fault -> FAILSAFE
 *   @verifies SYS-SAFE-005   arming preconditions
 *   @verifies SYS-SAFE-006   state-transition validation
 *   @verifies SYS-STATE-002  state read accessor
 *   @verifies COMM-RC-002    RC loss detection
 *   @verifies EST-MAH-002    fault-sample rejection / degraded flag
 *   @verifies CTRL-ARM-001   arming preconditions (RC path)
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "comm/comm.h"
#include "est/est.h"
#include "sys/state.h"
#include "vayu_status.h"

/* vaios host clock (sim/host/src/host_vaios.c). */
extern uint32_t v_get_ticks(void);
extern void v_delay(uint32_t ms);

/* ----------------------------------------------------------------------------
 * Tiny check framework
 * --------------------------------------------------------------------------*/
static int g_checks = 0;
static int g_fails = 0;

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

/* Drive the state machine to `target` via legal edges (FAILSAFE is
 * reachable from anywhere; STANDBY follows FAILSAFE; the flight states
 * follow STANDBY). Used to set up each scenario's precondition. */
static void force_state(sys_state_t target) {
  VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  if (target == SYSTEM_STATE_FAILSAFE) {
    return;
  }
  VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
  if (target == SYSTEM_STATE_STANDBY) {
    return;
  }
  if (target == SYSTEM_STATE_IN_AIR) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
  }
  VAYU_DISCARD(system_state_set(target));
}

/* Drive the estimator into the degraded state: open a rejection window,
 * let it exceed EST_DEGRADED_TIMEOUT_MS, then post one more reject. */
static void force_estimator_degraded(void) {
  estimator_mark_sample(false); /* opens rejection window */
  v_delay(EST_DEGRADED_TIMEOUT_MS + 30U);
  estimator_mark_sample(false); /* elapsed > horizon -> raise */
}

/* ----------------------------------------------------------------------------
 * SYS-SAFE-006 / SYS-STATE-002 — validated transitions
 * --------------------------------------------------------------------------*/
static void test_state_transition_table(void) {
  printf("  test_state_transition_table\n");

  system_state_init();
  CHECK(system_state_get() == SYSTEM_STATE_INIT, "init -> INIT");

  CHECK(system_state_set(SYSTEM_STATE_STANDBY) == VAYU_OK, "INIT->STANDBY ok");
  CHECK(system_state_get() == SYSTEM_STATE_STANDBY, "state is STANDBY");

  /* STANDBY->IN_AIR is not a legal edge. */
  CHECK(system_state_set(SYSTEM_STATE_IN_AIR) == VAYU_ERR_INVALID,
        "STANDBY->IN_AIR rejected");
  CHECK(system_state_get() == SYSTEM_STATE_STANDBY,
        "rejected edge leaves state unchanged");

  CHECK(system_state_set(SYSTEM_STATE_ARMED) == VAYU_OK, "STANDBY->ARMED ok");

  /* Self-loop: no-op, but a success. */
  CHECK(system_state_set(SYSTEM_STATE_ARMED) == VAYU_OK,
        "ARMED->ARMED no-op ok");
  CHECK(system_state_get() == SYSTEM_STATE_ARMED, "still ARMED");

  /* FAILSAFE reachable from any state regardless of the table. */
  CHECK(system_state_set(SYSTEM_STATE_FAILSAFE) == VAYU_OK,
        "ARMED->FAILSAFE always allowed");
  CHECK(system_state_get() == SYSTEM_STATE_FAILSAFE, "state is FAILSAFE");

  /* FAILSAFE->ARMED is not a legal recovery edge. */
  CHECK(system_state_set(SYSTEM_STATE_ARMED) == VAYU_ERR_INVALID,
        "FAILSAFE->ARMED rejected");
  CHECK(system_state_set(SYSTEM_STATE_STANDBY) == VAYU_OK,
        "FAILSAFE->STANDBY ok (recovery)");
}

/* ----------------------------------------------------------------------------
 * COMM-RC-002 — RC loss detection
 * --------------------------------------------------------------------------*/
static void test_rc_signal_timeout(void) {
  printf("  test_rc_signal_timeout\n");

  rc_mark_frame_valid();
  CHECK(rc_has_signal() == true, "fresh frame -> signal present");
  CHECK(rc_loss() == false, "fresh frame -> no rc_loss");

  /* Two-tier: rc_loss (100 ms) trips well before rc_has_signal (1.0 s). */
  v_delay(RC_LOSS_DETECT_MS + 50U);
  CHECK(rc_loss() == true, "past 100 ms -> rc_loss tripped (COMM-RC-002)");
  CHECK(rc_has_signal() == true, "but still within 1.0 s -> signal present");

  v_delay(RC_LOSS_TIMEOUT_MS + 100U);
  CHECK(rc_has_signal() == false, "no frame past timeout -> signal lost");

  rc_mark_frame_valid();
  CHECK(rc_has_signal() == true, "new frame -> signal recovered");
  CHECK(rc_loss() == false, "new frame -> rc_loss cleared");
}

/* ----------------------------------------------------------------------------
 * SYS-SAFE-002 — RC loss drives FAILSAFE
 * --------------------------------------------------------------------------*/
static void test_rc_watchdog_failsafe(void) {
  printf("  test_rc_watchdog_failsafe\n");

  force_state(SYSTEM_STATE_STANDBY);
  rc_mark_frame_valid();
  rc_watchdog_step();
  CHECK(system_state_get() == SYSTEM_STATE_STANDBY,
        "signal present -> no failsafe");

  v_delay(RC_LOSS_TIMEOUT_MS + 100U);
  rc_watchdog_step();
  CHECK(system_state_get() == SYSTEM_STATE_FAILSAFE,
        "signal lost in STANDBY -> FAILSAFE");

  /* Idempotent once already in FAILSAFE. */
  rc_watchdog_step();
  CHECK(system_state_get() == SYSTEM_STATE_FAILSAFE, "stays FAILSAFE");

  /* Watchdog is inactive on the bench (CALIBRATING) even with no link. */
  force_state(SYSTEM_STATE_CALIBRATING); /* signal still lost */
  rc_watchdog_step();
  CHECK(system_state_get() == SYSTEM_STATE_CALIBRATING,
        "no watchdog in CALIBRATING despite RC loss");

  rc_mark_frame_valid(); /* restore link for later tests */
}

/* ----------------------------------------------------------------------------
 * EST-MAH-002 — fault-sample rejection raises the degraded flag
 * --------------------------------------------------------------------------*/
static void test_estimator_degraded(void) {
  printf("  test_estimator_degraded\n");

  estimator_mark_sample(true);
  CHECK(estimator_is_degraded() == false, "valid samples -> not degraded");

  estimator_mark_sample(false);
  CHECK(estimator_is_degraded() == false,
        "brief rejection (< horizon) -> not yet degraded");

  v_delay(EST_DEGRADED_TIMEOUT_MS + 30U);
  estimator_mark_sample(false);
  CHECK(estimator_is_degraded() == true,
        "continuous rejection past horizon -> degraded");

  estimator_mark_sample(true);
  CHECK(estimator_is_degraded() == false, "one valid sample clears degraded");
}

/* ----------------------------------------------------------------------------
 * SYS-SAFE-003 — persistent estimator fault drives FAILSAFE
 * --------------------------------------------------------------------------*/
static void test_estimator_safety_failsafe(void) {
  printf("  test_estimator_safety_failsafe\n");

  /* Healthy estimator in a flight state: no transition. */
  estimator_mark_sample(true);
  force_state(SYSTEM_STATE_ARMED);
  estimator_safety_step();
  CHECK(system_state_get() == SYSTEM_STATE_ARMED,
        "healthy estimator -> no failsafe");

  /* Degraded estimator in a flight state: FAILSAFE. */
  force_estimator_degraded();
  CHECK(estimator_is_degraded() == true, "estimator degraded (setup)");
  estimator_safety_step();
  CHECK(system_state_get() == SYSTEM_STATE_FAILSAFE,
        "degraded estimator in ARMED -> FAILSAFE");

  estimator_mark_sample(true); /* clear for later tests */
}

/* ----------------------------------------------------------------------------
 * SYS-SAFE-005 / CTRL-ARM-001 — arming preconditions
 * --------------------------------------------------------------------------*/
static void test_arm_preconditions(void) {
  printf("  test_arm_preconditions\n");

  ibus_data_t rc;
  memset(&rc, 0, sizeof rc);
  rc.channels[2] = 1000; /* throttle at minimum */

  /* All preconditions satisfied. */
  rc_mark_frame_valid();
  estimator_mark_sample(true);
  CHECK(arm_preconditions_met(&rc) == true,
        "throttle low + RC + healthy estimator -> arm allowed");

  /* Throttle not at minimum blocks arming. */
  rc.channels[2] = 1500;
  CHECK(arm_preconditions_met(&rc) == false, "high throttle -> arm blocked");
  rc.channels[2] = 1000;

  /* Degraded estimator blocks arming. */
  force_estimator_degraded();
  CHECK(arm_preconditions_met(&rc) == false,
        "degraded estimator -> arm blocked");
  estimator_mark_sample(true);

  /* RC loss blocks arming. */
  v_delay(RC_LOSS_TIMEOUT_MS + 100U);
  CHECK(arm_preconditions_met(&rc) == false, "RC loss -> arm blocked");
  rc_mark_frame_valid();
  CHECK(arm_preconditions_met(&rc) == true,
        "preconditions restored -> allowed");
}

/* ----------------------------------------------------------------------------
 * CMD_ARM/CMD_DISARM software-arm latch (rc_arm_engaged) — lets a 4-channel
 * stick (no physical arm channel: ch5 fills to 1500) arm from the GCS.
 *   @verifies CTRL-ARM-001
 * --------------------------------------------------------------------------*/
static void test_software_arm_latch(void) {
  printf("  test_software_arm_latch\n");

  ibus_data_t rc;
  memset(&rc, 0, sizeof rc);
  rc.channels[2] = 1000; /* throttle at minimum */
  rc.channels[4] = 1500; /* 4-ch HID: arm channel absent -> filled 1500 */

  /* Predicate: latch clear + no switch -> not arm-engaged. */
  g_sw_arm_request = 0;
  CHECK(rc_arm_engaged(&rc) == false, "ch5==1500 + latch clear -> not engaged");

  /* CMD_ARM sets the latch -> engaged even with ch5 at 1500. */
  g_sw_arm_request = 1;
  CHECK(rc_arm_engaged(&rc) == true, "software-arm latch -> engaged");

  /* Physical switch still works independently of the latch. */
  g_sw_arm_request = 0;
  rc.channels[4] = 2000;
  CHECK(rc_arm_engaged(&rc) == true, "ch5>1500 -> engaged (latch clear)");
  rc.channels[4] = 1500;

  /* End-to-end: with the latch set + preconditions met, the per-frame arm
   * decision the RC task makes transitions STANDBY -> ARMED. */
  force_state(SYSTEM_STATE_STANDBY);
  rc_mark_frame_valid();
  estimator_mark_sample(true);
  g_sw_arm_request = 1; /* CMD_ARM */
  if (rc_arm_engaged(&rc) && system_state_get() == SYSTEM_STATE_STANDBY &&
      arm_preconditions_met(&rc)) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
  }
  CHECK(system_state_get() == SYSTEM_STATE_ARMED,
        "CMD_ARM latch + preconditions -> ARMED");

  /* CMD_DISARM clears the latch -> next frame disarms to STANDBY. */
  g_sw_arm_request = 0; /* CMD_DISARM */
  if (!rc_arm_engaged(&rc) && (system_state_get() == SYSTEM_STATE_ARMED ||
                               system_state_get() == SYSTEM_STATE_FAILSAFE)) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
  }
  CHECK(system_state_get() == SYSTEM_STATE_STANDBY,
        "CMD_DISARM latch clear -> STANDBY");

  /* High throttle blocks a latched arm just like the switch path. */
  force_state(SYSTEM_STATE_STANDBY);
  rc.channels[2] = 1500; /* throttle up */
  g_sw_arm_request = 1;
  CHECK(rc_arm_engaged(&rc) == true && arm_preconditions_met(&rc) == false,
        "latched arm + high throttle -> preconditions block ARMED");
  g_sw_arm_request = 0;
  rc.channels[2] = 1000;
}

/* FlySky throttle-failsafe: jump-AND-held to >1900, with no false trip on a
 * continuous ramp or a one-frame glitch, and clean recovery. */
static void test_rc_throttle_failsafe(void) {
  printf("  test_rc_throttle_failsafe\n");

  /* A slow, continuous ramp into full throttle must NOT trip, even above the
   * threshold — no single-frame jump ever arms the detector. */
  rc_throttle_failsafe_reset();
  bool tripped = false;
  for (uint16_t t = 1500; t <= 2000; t = (uint16_t)(t + 40)) {
    tripped |= rc_throttle_failsafe_step(t);
  }
  CHECK(!tripped, "slow ramp to full throttle does not trip failsafe");

  /* A sudden jump to >1900 held for the confirmation window DOES trip. */
  rc_throttle_failsafe_reset();
  CHECK(!rc_throttle_failsafe_step(1500), "low throttle: no failsafe");
  bool fs = rc_throttle_failsafe_step(1980); /* the snap */
  for (unsigned i = 1; i < RC_FAILSAFE_HOLD_FRAMES; i++) {
    fs = rc_throttle_failsafe_step(1980);
  }
  CHECK(fs, "sudden jump to >1900, held, trips failsafe");

  /* A one-frame spike that drops back next frame must NOT latch. */
  rc_throttle_failsafe_reset();
  (void)rc_throttle_failsafe_step(1500);
  (void)rc_throttle_failsafe_step(1980); /* jump */
  CHECK(!rc_throttle_failsafe_step(1450),
        "jump then immediate drop does not latch");

  /* Once latched, returning to a normal throttle clears it. */
  rc_throttle_failsafe_reset();
  (void)rc_throttle_failsafe_step(1500);
  for (unsigned i = 0; i <= RC_FAILSAFE_HOLD_FRAMES; i++) {
    (void)rc_throttle_failsafe_step(1980);
  }
  CHECK(rc_throttle_failsafe_step(1980), "stays latched while held high");
  CHECK(!rc_throttle_failsafe_step(1400),
        "clears once throttle returns normal");
}

int main(void) {
  printf("== Phase-2 safety SITL verification ==\n");

  test_state_transition_table();
  test_rc_signal_timeout();
  test_rc_watchdog_failsafe();
  test_estimator_degraded();
  test_estimator_safety_failsafe();
  test_arm_preconditions();
  test_software_arm_latch();
  test_rc_throttle_failsafe();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
