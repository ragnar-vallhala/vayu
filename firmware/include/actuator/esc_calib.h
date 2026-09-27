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

/* Entry cuts the PWM signal entirely before showing maximum, so a powered ESC
 * stops its motor first instead of being handed full throttle while running.
 * Long enough for an ESC to register signal loss and shut down -- typical
 * timeouts are a few hundred ms. */
#define ESC_CALIB_SIGNAL_CUT_MS 2000u

/* How long minimum is held after the closing gesture, so the ESC has time to
 * store the endpoint and acknowledge before the outputs go away. */
#define ESC_CALIB_SETTLE_MS 2000u

/* Backstop. If the closing gesture never arrives -- operator walked away, RC
 * left on a bench -- the motors must not sit at full throttle indefinitely. */
#define ESC_CALIB_TIMEOUT_MS 60000u

/**
 * Per-RC-frame service. Call from the RC task with the frame the FC acted on.
 *
 * Detects the entry gesture while STANDBY and the closing gesture while
 * ESC_CALIB, drives the phase, and enforces the timeout. Owns every transition
 * into and out of SYSTEM_STATE_ESC_CALIB.
 */
void esc_calib_rc_step(const ibus_data_t *rc);

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

/**
 * True while the PWM signal should be OFF entirely -- no pulses at all, not a
 * zero-width one.
 *
 * The first phase after entry. A powered ESC that is handed maximum while
 * running just spins the motor up; cutting the signal makes it stop first, so
 * entering calibration does not spin anything. motor_task stops the timer
 * outputs while this is true and restarts them after.
 */
bool esc_calib_signal_off(void);

#endif /* VAYU_ACTUATOR_ESC_CALIB_H */
