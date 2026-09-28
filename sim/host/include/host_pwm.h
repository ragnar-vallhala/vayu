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
/* What the host's PWM stubs last saw, per channel (1..4 = motors 1..4).
 *
 * The four ESCs share one timer, so "did touching motor 1 disturb motor 3"
 * is a question worth being able to ask. See sim/host/tests/test_esc.c. */
#ifndef HOST_PWM_H
#define HOST_PWM_H

#include <stdbool.h>

/** Raw duty last written to a channel, 0..1. 0 for an invalid channel. */
float host_pwm_duty(unsigned channel);

/** Whether the channel's output is enabled. False for an invalid channel. */
bool host_pwm_started(unsigned channel);

#endif // HOST_PWM_H
