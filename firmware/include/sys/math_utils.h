#ifndef VAYU_MATH_UTILS_H
#define VAYU_MATH_UTILS_H

#include <stdint.h>

/**
 * @brief Converts a 32-bit float to a 16-bit half-precision float (IEEE 754).
 *
 * @param f The 32-bit float to convert.
 * @return uint16_t The 16-bit float representation.
 */
uint16_t float32_to_float16(float f);

#endif // VAYU_MATH_UTILS_H
