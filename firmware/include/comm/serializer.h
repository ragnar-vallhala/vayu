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
#ifndef VAYU_SERIALIZER_H
#define VAYU_SERIALIZER_H
#include "channel.h"
#include "comm/comm_types.h"
#include "common/hal_types.h"
#include "sys/types.h"
#include <stdint.h>

/* Telemetry-UART RX ISR (registered as the USART6 RX callback). */
void uart2_packet_recv_callback(void);

/* Drain up to `max` raw telemetry-UART RX bytes into `out`, returning the count.
 * The RX ISR mirrors every byte into a lock-free ring so the NavLink v2 parser
 * can run in task context — its handlers apply commands / send frames, which is
 * not ISR-safe. Single consumer only (comm_processor_task). */
uint16_t comm_rx_raw_drain(uint8_t *out, uint16_t max);

#ifdef VAYU_SIM
/* Test-only: push bytes into the RX ring as if the ISR had received them.
 * Sim builds only (see serializer.c). */
void comm_rx_raw_inject(const uint8_t *data, uint16_t n);
#endif
#endif // !VAYU_SERIALIZER_H