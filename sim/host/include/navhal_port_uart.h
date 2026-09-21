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
#ifndef VAYU_SIM_NAVHAL_PORT_UART_H
#define VAYU_SIM_NAVHAL_PORT_UART_H

#include "common/hal_status.h"
#include "utils/uart_types.h"
#include <stdint.h>

/* Host SITL stub. The DMA-write entry point the firmware calls (e.g. channel.c)
 * is implemented synchronously in host_navhal.c; declared here so host
 * translation units see a real prototype instead of an implicit declaration. */
hal_status_t hal_uart_write_dma(hal_uart_t uart, const uint8_t *buf,
                                uint16_t len);

#endif
