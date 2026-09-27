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
#ifndef VAYU_ACTUATOR_ESC_CALIB_H
#define VAYU_ACTUATOR_ESC_CALIB_H

#include "comm/ibus.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * ESC endpoint calibration.
 *
 * An ESC learns its throttle band by being shown the extremes: it is powered
 * up while already seeing maximum, acknowledges with a beep, is then shown
 * minimum, and stores the pair. Until it has been shown them, its idea of
 * "minimum" is whatever the factory left it with -- which is how one motor
 * ends up refusing to start at the 15% idle floor while its three siblings
 * spin, the fault this exists to fix.
 *
 * WHY THIS IS A TWO-BOOT PROCEDURE. "Powered up while already seeing maximum"
 * is the whole mechanism, and it cannot be arranged at runtime: on a
 * battery-only aircraft the FC and the ESCs come up together, so by the time
 * any stick gesture can be performed the ESCs are already running, and showing
 * a running ESC maximum simply spins the motor to full. So the gesture does
 * not calibrate anything -- it writes a request to the card and asks for a
 * power cycle. The next boot drives maximum from startup, while the ESCs are
 * still waking, which is the only moment that works.
 *
 * THIS DRIVES EVERY MOTOR TO 100%. It is the only procedure in the firmware
 * that does so from a disarmed aircraft, so it is gated like arming rather
 * than like a setting:
 *
 *   - it has its own flight state, SYSTEM_STATE_ESC_CALIB, reachable only
 *     from STANDBY, and motor_task refuses to leave zero in any other state;
 *   - it starts and ends on deliberate two-stick gestures, held, not on a
 *     single command that a stray frame could imitate;
 *   - arming is impossible while it runs (no ESC_CALIB -> ARMED transition);
 *   - losing the RC link drops to FAILSAFE and zeroes the motors, because the
 *     gesture that ends it arrives over that link;
 *   - it self-aborts after ESC_CALIB_TIMEOUT_MS however it was entered.
 *
 * PROPS OFF. Nothing in software can check that, which is exactly why the
 * entry gesture is awkward enough that nobody performs it by accident.
 */

/* Stick thresholds on the raw iBUS scale (1000..2000, centre 1500). Full
 * deflection, not a nudge: a gesture that triggers near centre would be
 * reachable while handling the aircraft. */
#define ESC_CALIB_STICK_LO 1150u
#define ESC_CALIB_STICK_HI 1850u

/* How long a gesture must be held. Long enough that a stick swept through the
 * corner on its way somewhere else does not count. */
#define ESC_CALIB_GESTURE_MS 1500u

/* How long minimum is held after the closing gesture, so the ESC has time to
 * store the endpoint and acknowledge before the outputs go away. */
#define ESC_CALIB_SETTLE_MS 2000u

/* Backstop. If the closing gesture never arrives -- operator walked away, RC
 * left on a bench -- the motors must not sit at full throttle indefinitely. */
#define ESC_CALIB_TIMEOUT_MS 60000u

/** Marker file holding a pending calibration request. 8.3 name. */
#define ESC_CALIB_STORE_PATH "0:esccal.bin"
#define ESC_CALIB_STORE_MAGIC 0x4C414345u /* 'E''C''A''L' */

/**
 * Boot-time entry. Call once, before the scheduler, AFTER the filesystem and
 * the state machine are up.
 *
 * Reads the request marker and clears it immediately -- one attempt per
 * request, so a power cut mid-calibration cannot leave the aircraft latched
 * into driving maximum on every future boot. If a request was pending, enters
 * SYSTEM_STATE_ESC_CALIB so the motor task drives maximum from startup.
 */
void esc_calib_boot_init(void);

/**
 * Per-RC-frame service. Call from the RC task with the frame the FC acted on.
 *
 * While STANDBY: detects the entry gesture and writes the request marker --
 * it does NOT drive the motors. While ESC_CALIB (entered at boot): detects the
 * closing gesture, drives the phase, and enforces the timeout.
 */
void esc_calib_rc_step(const ibus_data_t *rc);

/** True once a request is written and waiting for the power cycle. */
bool esc_calib_request_pending(void);

/**
 * Normalised output every motor should be given right now, 0..1.
 *
 * Only meaningful while SYSTEM_STATE_ESC_CALIB; motor_task asks for it instead
 * of reading the mixer queue in that state. 1.0 is the maximum endpoint and
 * 0.0 the minimum, which esc.c maps to the 1.0..2.0 ms pulse band.
 */
float esc_calib_output(void);

/** True while calibration is running. */
bool esc_calib_active(void);

#endif /* VAYU_ACTUATOR_ESC_CALIB_H */
