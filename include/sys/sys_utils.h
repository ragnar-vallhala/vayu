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

/* Device identity (set from the GCS heartbeat). */
uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);

/* CRC32 (hardware peripheral, mutex-guarded). */
uint32_t utils_compute_crc32(const uint8_t *data, uint32_t len);
uint32_t utils_try_compute_crc32(const uint8_t *data, uint32_t len);

#endif // VAYU_SYS_UTILS_H
