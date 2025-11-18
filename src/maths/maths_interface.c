#include "maths/maths_interface.h"

// -------------------------
// Backend Selection
// -------------------------
// #define USE_ARM_DSP
#define USE_STANDARD_MATH

#if defined(USE_STANDARD_MATH)
    #include <math.h>

#elif defined(USE_ARM_DSP)
    #include "arm_math.h"

#else
    #error "Math backend not implemented"
#endif


// -------------------------
// sin(x)
// -------------------------
float sf_sin(float x)
{
#if defined(USE_STANDARD_MATH)
    return sinf(x);

#elif defined(USE_ARM_DSP)
    return arm_sin_f32(x);

#else
    return 0.0f;
#endif
}


// -------------------------
// cos(x)
// -------------------------
float sf_cos(float x)
{
#if defined(USE_STANDARD_MATH)
    return cosf(x);

#elif defined(USE_ARM_DSP)
    return arm_cos_f32(x);

#else
    return 0.0f;
#endif
}


// -------------------------
// atan2(y, x)
// -------------------------
// ⚠ CMSIS-DSP DOES NOT HAVE arm_atan2_f32()
// You must use the standard math version.
float sf_atan2(float y, float x)
{
#if defined(USE_STANDARD_MATH)
    return atan2f(y, x);

#elif defined(USE_ARM_DSP)
    return atan2f(y, x);       // no CMSIS version exists

#else
    return 0.0f;
#endif
}


// -------------------------
// sqrt(x)
// -------------------------
float sf_sqrt(float x)
{
#if defined(USE_STANDARD_MATH)
    return sqrtf(x);

#elif defined(USE_ARM_DSP)
    return sqrtf(x);

#else
    return 0.0f;
#endif
}


// -------------------------
// pow(base, exp)
// -------------------------
// ⚠ no CMSIS pow() implementation exists
float sf_pow(float base, float exp)
{
#if defined(USE_STANDARD_MATH)
    return powf(base, exp);

#elif defined(USE_ARM_DSP)
    return powf(base, exp);   // fallback

#else
    return 0.0f;
#endif
}


// -------------------------
// normalize vector
// -------------------------
void normalize_vector(vector_t *v)
{
    if (!v || !v->values || v->length <= 0)
        return;

    float norm = 0.0f;

    for (int i = 0; i < v->length; i++)
        norm += v->values[i] * v->values[i];

    norm = sf_sqrt(norm);

    if (norm > 0.0f)
    {
        for (int i = 0; i < v->length; i++)
            v->values[i] /= norm;
    }
}
