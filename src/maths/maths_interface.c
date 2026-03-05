#include "maths/maths_interface.h"
#include <math.h>

// -------------------------
// sin(x)
// -------------------------
float sf_sin(float x) { return sinf(x); }

// -------------------------
// cos(x)
// -------------------------
float sf_cos(float x) { return cosf(x); }

// -------------------------
// atan2(y, x)
// -------------------------
float sf_atan2(float y, float x) { return atan2f(y, x); }

// -------------------------
// sqrt(x)
// -------------------------
float sf_sqrt(float x) { return sqrtf(x); }

// -------------------------
// pow(base, exp)
// -------------------------
float sf_pow(float base, float exp) { return powf(base, exp); }

// -------------------------
// normalize vector
// -------------------------
void normalize_vector(vector_t *v) {
  if (!v || !v->values || v->length <= 0)
    return;

  float norm = 0.0f;

  for (int i = 0; i < v->length; i++)
    norm += v->values[i] * v->values[i];

  norm = sf_sqrt(norm);

  if (norm > 0.0f) {
    for (int i = 0; i < v->length; i++)
      v->values[i] /= norm;
  }
}
