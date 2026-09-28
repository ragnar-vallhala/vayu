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
/* Annunciator policy: what the LEDs and buzzer mean.
 *
 * heartbeat_task is the single writer of the indicators (driver/indicator.h).
 * Anything else that wants to say something asks here instead of reaching for
 * a pin -- that is what fault line F5 was. */
#ifndef VAYU_HEARTBEAT_H
#define VAYU_HEARTBEAT_H

/* Report link activity. The blue LED blinks fast for
 * HEARTBEAT_ACTIVITY_MS, then the flight-state pattern resumes.
 *
 * Call it as often as you like; repeat calls extend the window rather than
 * restart a blink. Cheap enough for a per-frame call path: two stores. */
void heartbeat_note_link_activity(void);

#endif // VAYU_HEARTBEAT_H
