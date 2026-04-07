#ifndef VAYU_UTILS_H
#define VAYU_UTILS_H
#include "structure.h"
#include <stdint.h>

extern mpmc_queue_t vayu_log_queue;
#define VAYU_LOG_QUEUE_SIZE 128

uint64_t get_timestamp(void);
void increment_high_freq_timer(void);
uint32_t get_timestamp_unix(void);
void set_timestamp(uint32_t timestamp);

uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);
void vayu_log(const char *fmt, ...);

uint32_t utils_compute_crc32(const uint8_t *data, uint32_t len);
uint32_t utils_try_compute_crc32(const uint8_t *data, uint32_t len);

#endif //! VAYU_UTILS_H