#include "maths_interface.h"
#ifdef USE_STANDARD_MATH
#include <math.h>
#else
#error "Math backend not implemented"
#endif
float sf_sin(float x)
{
#ifdef USE_STANDARD_MATH
    return sin(x);
#else
    return 0.0f; // Placeholder
#endif
}
float sf_cos(float x)
{
#ifdef USE_STANDARD_MATH
    return cos(x);
#else
    return 0.0f; // Placeholder
#endif
}
float sf_atan2(float y, float x)
{
#ifdef USE_STANDARD_MATH
    return atan2(y, x);
#else
    return 0.0f; // Placeholder
#endif
}
float sf_sqrt(float x)
{
#ifdef USE_STANDARD_MATH
    return sqrt(x);
#else
    return 0.0f; // Placeholder
#endif
}
float sf_pow(float base, float exp)
{
#ifdef USE_STANDARD_MATH
    return pow(base, exp);
#else
    return 0.0f; // Placeholder
#endif
}

void normalize_vector(vector_t *v){
    if (v == NULL || v->values == NULL || v->length <= 0) {
        return; // Handle invalid input
    }

    float norm = 0.0f;
    for (int i = 0; i < v->length; i++) {
        norm += v->values[i] * v->values[i];
    }
    norm = sf_sqrt(norm);

    if (norm > 0.0f) {
        for (int i = 0; i < v->length; i++) {
            v->values[i] /= norm;
        }
    }
}