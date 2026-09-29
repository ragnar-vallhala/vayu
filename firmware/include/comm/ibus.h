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
#ifndef VAYU_COMM_IBUS_H
#define VAYU_COMM_IBUS_H

#include <stdbool.h>
#include <stdint.h>

#define IBUS_MAX_CHANNELS 14
#define IBUS_PACKET_SIZE 32
#define IBUS_START_BYTE 0x20
#define IBUS_CMD_CHANNELS 0x40

/** iBus channels are 12-bit; the top nibble of the leading channels carries
 *  the extended 15-18 set, which this decoder does not read. */
#define IBUS_CHANNEL_MASK 0x0FFFu

typedef struct {
  uint16_t channels[IBUS_MAX_CHANNELS];
  bool is_failsafe;
} ibus_data_t;

typedef enum {
  IBUS_STATE_WAIT_START,
  IBUS_STATE_WAIT_CMD,
  IBUS_STATE_PAYLOAD,
  IBUS_STATE_CHECKSUM_L,
  IBUS_STATE_CHECKSUM_H
} ibus_state_t;

void ibus_init(ibus_data_t *data);
bool ibus_parse_byte(uint8_t byte, ibus_data_t *data);

/* ----------------------------------------------------------------------------
 * RC link watchdog (SYS-SAFE-002 / COMM-RC-002)
 * --------------------------------------------------------------------------*/

/** Failsafe horizon (ms): RC loss this long drives FAILSAFE (SYS-SAFE-002). */
#define RC_LOSS_TIMEOUT_MS 1000U

/** Fast COMM-layer detect horizon (ms): rc_loss() trips this quickly so
 *  higher layers can react before the slower failsafe transition (COMM-RC-002). */
#define RC_LOSS_DETECT_MS 100U

/* FlySky throttle-failsafe tunables. FlySky receivers expose no immediate
 * link-loss flag; on loss the receiver snaps the throttle channel (ch3 =
 * channels[2]) up to its configured failsafe preset (>1900 on this airframe).
 * rc_throttle_failsafe_step() trips on a sudden JUMP to >threshold that is then
 * HELD — both conditions, so a continuous full-throttle push (no single-frame
 * jump) and a one-frame glitch (not held) don't false-trip. Tune on hardware. */
#define RC_FAILSAFE_THROTTLE_RAW 1900U /**< ch3 above this counts as "high". */
#define RC_FAILSAFE_JUMP_DELTA 300U /**< one-frame rise no human stick makes. */
#define RC_FAILSAFE_HOLD_FRAMES 4U  /**< consecutive high frames to confirm. */
#define RC_THROTTLE_MIN_RAW 1000U   /**< iBUS throttle minimum (0% throttle). */

/**
 * @brief Mark "right now" as the most recent valid RC frame.
 *
 * Called by the iBUS task whenever a complete frame parses cleanly.
 * Lock-free single 32-bit aligned store — safe from any context.
 *
 * @implements COMM-RC-002
 */
void rc_mark_frame_valid(void);

/**
 * @brief Predicate: was a valid RC frame received within
 *        RC_LOSS_TIMEOUT_MS (1.0 s) of now? Backs the failsafe gate.
 *
 * Lock-free; safe to call from the rate-loop hot path (R8.6).
 *
 * @implements SYS-SAFE-002
 */
bool rc_has_signal(void);

/**
 * @brief Predicate: has the RC link been lost for more than
 *        RC_LOSS_DETECT_MS (100 ms)? The fast COMM-layer rc_loss flag,
 *        distinct from (and tripping well before) the 1.0 s failsafe.
 *
 * Lock-free; safe from any context (R8.6).
 *
 * @implements COMM-RC-002
 */
bool rc_loss(void);

/**
 * @brief Step the FlySky throttle-failsafe detector with this frame's raw ch3
 *        (throttle, channels[2]).
 *
 * Returns true once a sudden jump to >RC_FAILSAFE_THROTTLE_RAW has been HELD
 * for RC_FAILSAFE_HOLD_FRAMES consecutive frames (the latch stays set while
 * throttle remains high and clears when it returns to the normal range). A
 * slow ramp into full throttle (no single-frame jump) never trips it; the 1 s
 * staleness watchdog (rc_watchdog_step) remains the guaranteed backstop.
 *
 * Single-caller (the RC task), lock-free.
 *
 * @implements SYS-SAFE-002
 */
bool rc_throttle_failsafe_step(uint16_t throttle_raw);

/** @brief Reset the throttle-failsafe detector (boot / tests). */
void rc_throttle_failsafe_reset(void);

/* ----------------------------------------------------------------------------
 * Channel plausibility (CTRL-ANGLE-103)
 *
 * A valid RC pulse is ~1000..2000 us. Anything outside this band is not a
 * stick position -- a channel that is still 0 at boot, a glitch frame, or a
 * receiver emitting something that is not a channel value at all. The control
 * layer substitutes CENTRED for such a reading rather than mapping it, which
 * is the right thing to do and was also completely silent: a guard that eats
 * input without counting is indistinguishable from one that never fires, and
 * that is exactly how a receiver failsafe got misread as a decoder bug.
 *
 * The predicate lives here so the RC layer and the control layer share ONE
 * definition of "implausible" -- and so it is counted once per FRAME, which
 * only the RC task sees. The blackbox rc stream is decimated to
 * HSL_RC_RATE_HZ (10 Hz) and the control loop runs on whatever was last
 * queued, so neither of those sees every frame.
 * --------------------------------------------------------------------------*/

#define RC_RAW_MIN_VALID 900U  /**< below this is not a stick position. */
#define RC_RAW_MAX_VALID 2100U /**< above this is not a stick position. */

/** @brief Is this raw channel value outside the plausible pulse band? */
bool rc_channel_implausible(uint16_t raw);

/**
 * @brief Note one parsed frame; returns true if any of the four flight
 *        channels (roll/pitch/throttle/yaw) was implausible.
 *
 * Counts frames, not channels, and logs the first one plus every
 * RC_IMPLAUSIBLE_LOG_EVERY after it, so a persistent fault reports itself
 * over the link without flooding it. Single-caller (the RC task).
 */
bool rc_note_implausible(const ibus_data_t *data);

/** How many frames carried at least one implausible flight channel. */
uint32_t rc_implausible_frames(void);

/** @brief Reset the implausible-frame counter (boot / tests). */
void rc_implausible_reset(void);

/**
 * @brief One RC-watchdog tick: if the link is lost in a flight-relevant
 *        state, request SYSTEM_STATE_FAILSAFE. No-op otherwise.
 *
 * Defined in src/comm/rc_safety.c.
 *
 * @implements SYS-SAFE-002
 */
void rc_watchdog_step(void);

/**
 * @brief STANDBY -> ARMED precondition gate: throttle at minimum, RC
 *        link healthy, estimator not degraded. Defined in
 *        src/comm/rc_safety.c.
 *
 * @implements SYS-SAFE-005, CTRL-ARM-001
 */
bool arm_preconditions_met(const ibus_data_t *rc);

/**
 * @brief GCS software-arm latch. Set by CMD_ARM, cleared by CMD_DISARM
 *        (src/comm/comm_processor.c). OR'd with the RC arm switch by
 *        rc_arm_engaged() so the vehicle can be armed from the GCS when no
 *        physical arm channel is available (e.g. a 4-channel USB-HID stick in
 *        SITL). Defined in src/comm/rc_safety.c.
 */
extern volatile uint8_t g_sw_arm_request;

/**
 * @brief Unified arm-request predicate: true when the RC arm switch
 *        (channel 5 > 1500) OR the GCS software-arm latch is engaged.
 *        Arm preconditions (throttle, link, estimator) are still enforced
 *        separately by the caller. Defined in src/comm/rc_safety.c.
 */
bool rc_arm_engaged(const ibus_data_t *rc);

#ifdef VAYU_SIM
/**
 * SITL-only fault-injection hook. When non-zero, the iBUS task
 * suppresses rc_mark_frame_valid() calls so a SITL scenario can
 * simulate RC loss without disabling the synth feed.
 */
extern volatile uint8_t sim_rc_force_loss;
#endif

#endif // VAYU_COMM_IBUS_H
