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
 * @file sys/sys_utils.h
 * @brief System utilities: monotonic timestamp, device id, CRC32.
 *
 * System-level services, distinct from the text logging in the LOG module.
 */
#ifndef VAYU_SYS_UTILS_H
#define VAYU_SYS_UTILS_H

#include <stdint.h>

/* High-frequency monotonic timer (ticks bumped by the HF timer ISR). */
uint64_t get_timestamp(void);
void increment_high_freq_timer(void);
/* Disciplined wall-clock in ms. 64-bit so it can hold full GCS epoch ms after a
 * wide time-sync correction (a 32-bit ms clock can't represent epoch). */
uint64_t get_timestamp_unix(void);
void set_timestamp(uint64_t timestamp);

/* Time-sync clock discipline (docs/telemetry/time_sync.md). The GCS computes a
 * filtered FC->GCS correction and pushes it here; the offset is slewed (not
 * jammed) so get_timestamp_unix() stays monotonic — the slew rate is clamped
 * well below the 1 ms/ms tick rate, so the unix clock can never run backward.
 * The first-ever correction steps directly (cold-start bootstrap). */
void time_sync_set_offset(int32_t offset_ms);
/* Wide correction (§10): apply a full 64-bit FC->GCS offset, used by the
 * REQUEST_WIDE path when the deviation exceeds int32 ms (~24.8 days) — e.g. a
 * cold-start FC uptime clock vs GCS epoch. Steps immediately (it is always a
 * large jump), then ongoing int32 residuals discipline via time_sync_set_offset. */
void time_sync_set_offset64(int64_t offset_ms);
/* Slew the applied offset toward the commanded target. Call periodically from a
 * low-rate task (the telemetry loop); dt is derived from the HF tick counter. */
void time_sync_discipline_tick(void);

/* §10.5 security gate: 1 once the GCS has disciplined our clock at least once
 * (a time-sync correction was applied), else 0. An unsynchronised FC MUST reject
 * every command (navlink_router's command_gate enforces this). */
uint8_t time_sync_is_synced(void);

/* Device identity (set from the GCS heartbeat). */
uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);

/* CRC32 (hardware peripheral, mutex-guarded). */
uint32_t utils_compute_crc32(const uint8_t *data, uint32_t len);
uint32_t utils_try_compute_crc32(const uint8_t *data, uint32_t len);

#endif // VAYU_SYS_UTILS_H
