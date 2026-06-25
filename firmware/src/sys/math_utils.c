#include "sys/math_utils.h"

uint16_t float32_to_float16(float f) {
  union {
    float f;
    uint32_t i;
  } u;
  u.f = f;

  uint16_t sign = (u.i >> 16) & 0x8000;
  int16_t exp = ((u.i >> 23) & 0xff);
  uint32_t mant = (u.i & 0x7fffff);

  if (exp == 0) { // Zero or subnormal
    return sign;
  } else if (exp == 0xff) { // Inf or NaN
    return sign | 0x7c00 | (mant ? 0x200 : 0);
  } else {
    int new_exp = (int)exp - 127 + 15;
    if (new_exp >= 31) { // Overflow
      return sign | 0x7c00;
    } else if (new_exp <= 0) { // Underflow
      return sign;
    } else {
      return sign | (uint16_t)(new_exp << 10) | (uint16_t)(mant >> 13);
    }
  }
}
