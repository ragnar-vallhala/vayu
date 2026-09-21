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
#ifndef VAYU_COMM_PERF_TELEMETRY_H
#define VAYU_COMM_PERF_TELEMETRY_H

/**
 * @file perf_telemetry.h
 * @brief Periodic task that streams the vaios perf/observability snapshot to
 *        the GCS as fragmented PACKET_TYPE_PERF_STATS packets.
 */

#include "comm/perf_packet.h"
#include "structure.h"

/* Periodic perf reporter (FC -> GCS). One report = GLOBAL + TASKS + FIFOS. */
void perf_telemetry_task(void *args);

/* Fill one FIFO wire row from a live SPSC fifo. Used by the buffer modules to
 * expose their otherwise-static fifos without leaking the pointers. */
void perf_fifo_fill_row(perf_fifo_row_t *row, uint8_t fifo_id,
                        const spsc_fifo_t *f);

#endif /* VAYU_COMM_PERF_TELEMETRY_H */
