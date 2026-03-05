#include "maths/maths_interface.h"

// -------------------------
// sin(x)
// -------------------------
float m_sin(float x) {
  // TODO: Implement custom sin algorithm
  return 0.0f;
}

// -------------------------
// cos(x)
// -------------------------
float m_cos(float x) {
  // TODO: Implement custom cos algorithm
  return 0.0f;
}

// -------------------------
// atan2(y, x)
// -------------------------
float m_atan2(float y, float x) {
  // TODO: Implement custom atan2 algorithm
  return 0.0f;
}

// -------------------------
// sqrt(x)
// -------------------------
float m_sqrt(float x) {
  // TODO: Implement custom sqrt algorithm
  return 0.0f;
}

// -------------------------
// pow(base, exp)
// -------------------------
float m_pow(float base, float exp) {
  // TODO: Implement custom pow algorithm
  return 0.0f;
}

// -------------------------
// normalize vector
// -------------------------
void normalize_vector(vector_t *v) {
  if (!v || !v->values || v->length <= 0)
    return;

  float norm = 0.0f;

  for (int i = 0; i < v->length; i++)
    norm += v->values[i] * v->values[i];

  norm = m_sqrt(norm);

  if (norm > 0.0f) {
    for (int i = 0; i < v->length; i++)
      v->values[i] /= norm;
  }
}
