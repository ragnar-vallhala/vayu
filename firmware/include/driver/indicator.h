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
/* The annunciator: three status LEDs and a buzzer.
 *
 * This is the ONLY code that writes those four pins. It used to be two --
 * heartbeat.c rendering flight state and navlink_router.c blinking the blue
 * LED on link activity -- which fought: whichever ran last won, heartbeat's
 * software state tracking desynced from the pin, and the LED meant nothing
 * reliable (fault line F5).
 *
 * Output pins cannot be read back reliably, so the on/off state is tracked
 * here in software. That is also why toggle is an operation of this module and
 * not something a caller does with a read-modify-write.
 *
 * Policy -- what each state looks like, and who wins when two things want the
 * blue LED -- belongs to sys/heartbeat.c. This module only knows how to make a
 * pin high or low.
 */
#ifndef VAYU_INDICATOR_H
#define VAYU_INDICATOR_H

#include <stdbool.h>

typedef enum {
  IND_LED_BLUE = 0,
  IND_LED_GREEN,
  IND_LED_RED,
  IND_BUZZER,
  IND_COUNT
} indicator_t;

/** Drive all four pins as outputs. Call once, before any other entry point. */
void indicator_init(void);

/** Set one indicator. Out-of-range is ignored. */
void indicator_set(indicator_t which, bool on);

/** Invert one indicator, using the tracked state. */
void indicator_toggle(indicator_t which);

/** Current tracked state. False for an out-of-range index. */
bool indicator_get(indicator_t which);

/** Everything off, including the buzzer. */
void indicator_all_off(void);

#endif // VAYU_INDICATOR_H
