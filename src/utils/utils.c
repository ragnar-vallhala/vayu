#include "utils/utils.h"
#include <stdint.h>

static uint32_t _time_stamp = 0;
static uint8_t _device_id = 0;

uint32_t get_timestamp(void) { return _time_stamp; }
void set_timestamp(uint32_t timestamp) { _time_stamp = timestamp; }

uint8_t get_device_id(void) { return _device_id; }
void set_device_id(uint8_t device_id) { _device_id = device_id; }