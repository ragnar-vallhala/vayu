#include "maths/maths_interface.h"
#include <math.h>

// -------------------------
// sin(x)
// -------------------------
float m_sin(float x) { return sinf(x); }

// -------------------------
// cos(x)
// -------------------------
float m_cos(float x) { return cosf(x); }

// -------------------------
// asin(x)
// -------------------------
float m_asin(float x) { return asinf(x); }

// -------------------------
// atan2(y, x)
// -------------------------
float m_atan2(float y, float x) { return atan2f(y, x); }

// -------------------------
// sqrt(x)
// -------------------------
float m_sqrt(float x) { return sqrtf(x); }

// -------------------------
// pow(base, exp)
// -------------------------
float m_pow(float base, float exp) { return powf(base, exp); }

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

void normalize_quaternion(quaternion_t *q) {
  float norm = m_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (norm > 0.0f) {
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
  }
}

void quaternion_multiply(const quaternion_t *qa, const quaternion_t *qb,
                         quaternion_t *out) {
  out->w = qa->w * qb->w - qa->x * qb->x - qa->y * qb->y - qa->z * qb->z;
  out->x = qa->w * qb->x + qa->x * qb->w + qa->y * qb->z - qa->z * qb->y;
  out->y = qa->w * qb->y - qa->x * qb->z + qa->y * qb->w + qa->z * qb->x;
  out->z = qa->w * qb->z + qa->x * qb->y - qa->y * qb->x + qa->z * qb->w;
}
