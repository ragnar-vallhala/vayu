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
 * @file perf_packet.c
 * @brief Shared helper for the perf telemetry wire format.
 *
 * Kept separate from perf_telemetry.c so the buffer modules (imu/rc/control)
 * can fill FIFO rows without pulling in the reporter task's kernel/serializer
 * dependencies (v_delay, send_packet, …). This translation unit needs only the
 * SPSC accessors, so it is host-buildable for SITL alongside structure.c.
 */

#include "comm/perf_telemetry.h" /* prototype + perf_packet.h + structure.h */

/** @noreq perf-row fill helper (observability glue) */
void perf_fifo_fill_row(perf_fifo_row_t *row, uint8_t fifo_id,
                        const spsc_fifo_t *f) {
  uint32_t d = spsc_drops(f);
  row->fifo_id = fifo_id;
  row->_pad = 0;
  row->peak = (uint16_t)spsc_peak(f);
  /* One slot is reserved by the ring; report the usable capacity. */
  row->capacity = (uint16_t)(f->capacity ? f->capacity - 1u : 0u);
  row->drops = (uint16_t)(d > 0xFFFFu ? 0xFFFFu : d);
}
