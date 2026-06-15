/**
 * @file sys/sys_utils.h
 * @brief System utilities: monotonic timestamp, device id, CRC32.
 *
 * Moved here from utils/utils.h per Phase 4 R2.6 — these are
 * system-level services, distinct from the text logging that moved to
 * the LOG module.
 */
#ifndef VAYU_SYS_UTILS_H
#define VAYU_SYS_UTILS_H

#include <stdint.h>

/* High-frequency monotonic timer (ticks bumped by the HF timer ISR). */
uint64_t get_timestamp(void);
void increment_high_freq_timer(void);
uint32_t get_timestamp_unix(void);
void set_timestamp(uint32_t timestamp);

/* Time-sync clock discipline (docs/telemetry/time_sync.md). The GCS computes a
 * filtered FC->GCS correction and pushes it here; the offset is slewed (not
 * jammed) so get_timestamp_unix() stays monotonic — the slew rate is clamped
 * well below the 1 ms/ms tick rate, so the unix clock can never run backward.
 * The first-ever correction steps directly (cold-start bootstrap). */
void time_sync_set_offset(int32_t offset_ms);
/* Slew the applied offset toward the commanded target. Call periodically from a
 * low-rate task (the telemetry loop); dt is derived from the HF tick counter. */
void time_sync_discipline_tick(void);

/* Device identity (set from the GCS heartbeat). */
uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);

/* CRC32 (hardware peripheral, mutex-guarded). */
uint32_t utils_compute_crc32(const uint8_t *data, uint32_t len);
uint32_t utils_try_compute_crc32(const uint8_t *data, uint32_t len);

#endif // VAYU_SYS_UTILS_H
