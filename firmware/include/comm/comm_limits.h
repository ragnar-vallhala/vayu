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
 * @file comm/comm_limits.h
 * @brief How much the link is allowed to cost in RAM, and how big a frame gets.
 *
 * These are sizing decisions, not tunables: each one buys buffer space, and on
 * a 96 KiB part that is the scarce resource. The numbers carry their own
 * reasoning because "raise it if you need more" is how the heap got tight.
 */
#ifndef VAYU_COMM_LIMITS_H
#define VAYU_COMM_LIMITS_H

/* One slot per UART that get_handler() is actually asked for. Today that is
 * telemetry (USART6) and nothing else -- RC drives its UART directly rather
 * than through a channel. Each slot costs 4112 B of .bss (2 x 2048 B ping-pong
 * TX), so a spare slot is not free the way an unused #define usually is.
 * Raise this when a second channel is claimed; get_handler_serial returns
 * ERROR on exhaustion and main.c checks it. */
#define MAX_SERIAL_HANDLERS 1
#define INCOMING_PACKET_BUFFER 3

#define RADIO_AVOID_BAND 10 /* channels to keep clear around the link */

#define ENABLE_BINARY_NAVLINK_PKT 1
#define ENABLE_BINARY_NAVLINK_PKT_LOGGING 1
#define NAVLINK_HEADER_SIZE 8
#define NAVLINK_MAX_PAYLOAD_SIZE 256
#define NAVLINK_CRC_SIZE 4
#define NAVLINK_MAX_SIZE                                                       \
  (NAVLINK_HEADER_SIZE + NAVLINK_MAX_PAYLOAD_SIZE + NAVLINK_CRC_SIZE)

#endif // VAYU_COMM_LIMITS_H
