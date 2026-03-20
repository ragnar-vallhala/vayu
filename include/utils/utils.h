#ifndef VAYU_UTILS_H
#define VAYU_UTILS_H
#include <stdint.h>

uint64_t get_timestamp(void);
void increment_high_freq_timer(void);
uint32_t get_timestamp_unix(void);
void set_timestamp(uint32_t timestamp);

uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);
void vayu_log(const char *fmt, ...);
#endif //! VAYU_UTILS_H