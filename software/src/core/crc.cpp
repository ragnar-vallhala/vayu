#include "crc.h"

uint32_t CRC32::calculate(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF; // initial value

  for (uint32_t i = 0; i < length; i++) {
    crc ^= (uint32_t)(data[i] << 24); // Process each byte (STM32 hardware CRC
                                      // accumulates 32-bit words, but we must
                                      // process byte-by-byte as transmitted)
    // Actually, STM32 HAL CRC accumulates 32 bytes or 8 bytes depending on
    // input format. The vayu firmware does `calculate_crc((uint8_t *)(&packet),
    // header_size + payload_size)` wait, HAL_CRC_Accumulate might treat byte
    // arrays differently depending on endianness.
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
