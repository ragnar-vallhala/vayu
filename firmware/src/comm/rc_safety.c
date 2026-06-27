/**
 * @file src/comm/rc_safety.c
 * @brief RC-link watchdog and arm-precondition policy.
 *
 * @implements COMM-RC-002, SYS-SAFE-002, SYS-SAFE-005, CTRL-ARM-001
 *
 * The pure safety policy that the iBUS task (src/comm/rc_task.c) drives.
 * Deliberately free of any DMA / UART / register dependency so it builds
 * and runs on the host SITL target and can be verified in isolation
 * (sim/host/tests/test_safety_phase2.c). rc_task.c calls these
 * via the declarations in comm/ibus.h.
 */
#include "comm/ibus.h"
#include "est/est.h"   /* estimator_is_degraded */
#include "sys/state.h"
#include "utils.h"                 /* v_get_ticks (vaios) */
#include "vayu_status.h"           /* VAYU_DISCARD */

#include <stdbool.h>
#include <stdint.h>

/* ----------------------------------------------------------------------------
 * RC link watchdog state (COMM-RC-002 / SYS-SAFE-002).
 *
 * Single uint32_t ms-timestamp of the most recent valid frame. 32-bit
 * aligned, declared volatile — readers and writers do not contend on
 * any lock (R8.6: no locks in hot loop paths).
 * --------------------------------------------------------------------------*/
static volatile uint32_t s_last_valid_frame_ms = 0;

/**
 * @implements COMM-RC-002
 */
void rc_mark_frame_valid(void) {
  s_last_valid_frame_ms = v_get_ticks();
}

/**
 * @implements SYS-SAFE-002
 */
bool rc_has_signal(void) {
  uint32_t now = v_get_ticks();
  uint32_t last = s_last_valid_frame_ms;
  /* Unsigned subtraction is well-defined and wraps correctly so long as
   * one tick interval (1 ms) is shorter than the wrap horizon (~49 days). */
  return (now - last) <= RC_LOSS_TIMEOUT_MS;
}

/**
 * @implements COMM-RC-002
 *
 * Fast COMM-layer loss detect: trips after RC_LOSS_DETECT_MS (100 ms),
 * an order of magnitude before rc_has_signal()'s 1.0 s failsafe horizon,
 * so higher layers can flag a degraded link early.
 */
bool rc_loss(void) {
  uint32_t now = v_get_ticks();
  uint32_t last = s_last_valid_frame_ms;
  return (now - last) > RC_LOSS_DETECT_MS;
}

/* ----------------------------------------------------------------------------
 * FlySky throttle-failsafe (SYS-SAFE-002 companion).
 *
 * FlySky receivers have no immediate link-loss flag; on loss the receiver snaps
 * the throttle channel (ch3 = channels[2]) up to its configured failsafe preset
 * (>1900 here). We treat a *sudden jump* to >RC_FAILSAFE_THROTTLE_RAW that is
 * then *held* for RC_FAILSAFE_HOLD_FRAMES as failsafe:
 *   - the jump (a one-frame rise no human stick can produce) rejects a
 *     legitimate, continuous full-throttle push, and
 *   - the hold rejects a single-frame glitch.
 * Edge case: if the pilot is already near the preset when the link drops, the
 * jump may be too small to see — the 1 s staleness watchdog is the backstop.
 * Single-task state (RC task), no locks (R8.6).
 * --------------------------------------------------------------------------*/
static uint16_t s_prev_throttle = RC_THROTTLE_MIN_RAW;
static uint8_t  s_throttle_high_frames = 0;
static bool     s_throttle_failsafe = false;

/** @noreq throttle-failsafe detector state reset (boot/tests) */
void rc_throttle_failsafe_reset(void) {
  s_prev_throttle = RC_THROTTLE_MIN_RAW;
  s_throttle_high_frames = 0;
  s_throttle_failsafe = false;
}

bool rc_throttle_failsafe_step(uint16_t throttle_raw) {
  bool high = throttle_raw > RC_FAILSAFE_THROTTLE_RAW;
  bool jumped = high && throttle_raw > s_prev_throttle &&
                (uint16_t)(throttle_raw - s_prev_throttle) >= RC_FAILSAFE_JUMP_DELTA;
  s_prev_throttle = throttle_raw;

  if (!high) {
    /* Back in the normal throttle range — clear the latch. */
    s_throttle_high_frames = 0;
    s_throttle_failsafe = false;
    return false;
  }

  /* High. Arm the confirmation window only on the jump edge, so a slow ramp
   * into full throttle (high but never jumped) never arms it. Once armed,
   * count consecutive high frames until the hold is confirmed. */
  if (s_throttle_high_frames == 0 && !jumped) {
    return false;
  }
  if (s_throttle_high_frames < RC_FAILSAFE_HOLD_FRAMES) {
    s_throttle_high_frames++;
  }
  if (s_throttle_high_frames >= RC_FAILSAFE_HOLD_FRAMES) {
    s_throttle_failsafe = true;
  }
  return s_throttle_failsafe;
}

/* States in which RC loss should drive a FAILSAFE transition. INIT and
 * CALIBRATING are excluded by design: the vehicle is on the bench and
 * the operator may legitimately have the transmitter off. */
/** @noreq state-set predicate helper for rc_watchdog_step (SYS-SAFE-002) */
static bool rc_watchdog_active_for(sys_state_t s) {
  return s == SYSTEM_STATE_STANDBY || s == SYSTEM_STATE_PREARM ||
         s == SYSTEM_STATE_ARMED   || s == SYSTEM_STATE_IN_AIR;
}

/* ----------------------------------------------------------------------------
 * Arm preconditions (SYS-SAFE-005 / CTRL-ARM-001).
 *
 * STANDBY → ARMED is gated on every condition the spec lists *that we
 * can currently measure*:
 *   - throttle stick at minimum   (rc->channels[2])
 *   - RC link healthy             (rc_has_signal)
 *   - estimator not degraded      (estimator_is_degraded)
 *
 * Calibration-freshness is in the spec text (SYS-SAFE-005) but has no
 * implementable predicate today — no calibration timestamp persists
 * across the boot, so it cannot be gated on until the persistence layer
 * grows the timestamp.
 * --------------------------------------------------------------------------*/
#define ARM_THROTTLE_MAX_RAW 1100U

/**
 * @implements SYS-SAFE-005, CTRL-ARM-001
 */
bool arm_preconditions_met(const ibus_data_t *rc) {
  return rc->channels[2] < ARM_THROTTLE_MAX_RAW &&
         rc_has_signal() &&
         !estimator_is_degraded();
}

/* GCS software-arm latch (CMD_ARM/CMD_DISARM). See ibus.h / rc_arm_engaged(). */
volatile uint8_t g_sw_arm_request = 0;

/**
 * @brief Arm requested when the RC arm switch (ch5 > 1500) OR the GCS
 *        software-arm latch is engaged. Lets a 4-channel stick (no physical
 *        arm channel) arm via the GCS. Preconditions remain the caller's job.
 * @implements CTRL-ARM-001
 */
bool rc_arm_engaged(const ibus_data_t *rc) {
  return rc->channels[4] > 1500 || g_sw_arm_request != 0;
}

/**
 * @implements SYS-SAFE-002
 */
void rc_watchdog_step(void) {
  sys_state_t cur = system_state_get();
  if (!rc_has_signal() && rc_watchdog_active_for(cur) &&
      cur != SYSTEM_STATE_FAILSAFE) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  }
}
