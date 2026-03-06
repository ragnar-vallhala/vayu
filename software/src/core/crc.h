#pragma once

#include <cstdint>

// Matches STM32 hardware CRC with polynomial 0x04C11DB7, initial value
// 0xFFFFFFFF
class CRC32 {
public:
  static uint32_t calculate(const uint8_t *data, uint32_t length);
};
