#ifndef VAYU_UTILS_H
#define VAYU_UTILS_H
#include <stdint.h>

uint32_t get_timestamp(void);
void set_timestamp(uint32_t timestamp);

uint8_t get_device_id(void);
void set_device_id(uint8_t device_id);
#endif //! VAYU_UTILS_H