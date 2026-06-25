#include "crc.h"

uint32_t CRC32::calculate(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF; // initial value

  // Byte-wise, MSB-first. Matches the STM32 HAL CRC block configured for
  // byte-aligned input on the firmware side.
  for (uint32_t i = 0; i < length; i++) {
    crc ^= (uint32_t)(data[i] << 24);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80000000) {
        crc = (crc << 1) ^ 0x04C11DB7; // Polynomial
      } else {
        crc = (crc << 1);
      }
    }
  }

  return crc;
}
