/**
 * @file linalg.h
 * @brief Header-only fixed-size linear-algebra and quaternion-geometry kernel.
 *
 * Small, allocation-free `static inline` primitives shared across the
 * estimation and control code (EKF covariance propagation, attitude
 * geometry, future nav filters). All matrices are dense, row-major
 * `float` arrays passed with explicit dimensions; the caller owns the
 * storage. Built on the float-only `m_*` math backend (no libm doubles),
 * so it is safe to use from -Wconversion -Werror translation units.
 *
 * Conventions:
 *  - Quaternions are Hamilton, scalar-first (quaternion_t = {w,x,y,z}).
 *  - R(q) rotates a vector from BODY to WORLD; R(q)^T world to body.
 *  - Matmul outputs must NOT alias their inputs (use a temp).
 */
#ifndef VAYU_MATHS_LINALG_H
#define VAYU_MATHS_LINALG_H

#include "maths/maths_interface.h"
#include <stdbool.h>

/* ----------------------------------------------------------------------------
 * Dense matrix primitives (row-major, explicit dims, no aliasing of out/in)
 * --------------------------------------------------------------------------*/

/** C(ar x bc) = A(ar x ac) * B(ac x bc).
 *
 * Plain ijk with the dot product accumulated in a register: on the M4F (small
 * fixed-size matrices, no data cache) this minimises memory traffic — one store
 * per output element. A cache-oriented ikj reorder is slower here. */
static inline void m_mat_mul(const float *A, int ar, int ac, const float *B,
                             int bc, float *C) {
  for (int i = 0; i < ar; i++) {
    for (int j = 0; j < bc; j++) {
      float s = 0.0f;
      for (int k = 0; k < ac; k++)
        s += A[i * ac + k] * B[k * bc + j];
      C[i * bc + j] = s;
    }
  }
}

/** C(ar x br) = A(ar x ac) * B(br x ac)^T. */
static inline void m_mat_mul_abt(const float *A, int ar, int ac, const float *B,
                                 int br, float *C) {
  for (int i = 0; i < ar; i++) {
    for (int j = 0; j < br; j++) {
      float s = 0.0f;
      for (int k = 0; k < ac; k++)
        s += A[i * ac + k] * B[j * ac + k];
      C[i * br + j] = s;
    }
  }
}

/** T(ac x ar) = A(ar x ac)^T. */
static inline void m_mat_transpose(const float *A, int ar, int ac, float *T) {
  for (int i = 0; i < ar; i++)
    for (int j = 0; j < ac; j++)
      T[j * ar + i] = A[i * ac + j];
}

/** C = A + B, all (r x c). */
static inline void m_mat_add(const float *A, const float *B, int r, int c,
                             float *C) {
  int n = r * c;
  for (int i = 0; i < n; i++)
    C[i] = A[i] + B[i];
}

/** C = A - B, all (r x c). */
static inline void m_mat_sub(const float *A, const float *B, int r, int c,
                             float *C) {
  int n = r * c;
  for (int i = 0; i < n; i++)
    C[i] = A[i] - B[i];
}

/** dst[0..count) = src[0..count). */
static inline void m_mat_copy(float *dst, const float *src, int count) {
  for (int i = 0; i < count; i++)
    dst[i] = src[i];
}

/** M(n x n) = identity. */
static inline void m_mat_identity(float *M, int n) {
  int nn = n * n;
  for (int i = 0; i < nn; i++)
    M[i] = 0.0f;
  for (int i = 0; i < n; i++)
    M[i * n + i] = 1.0f;
}

/** In-place symmetrize: M = (M + M^T) / 2  (n x n).
 *
 * A covariance must stay symmetric for the Kalman gain to behave, but the
 * Joseph and Phi P Phi^T products accumulate float-rounding asymmetry on every
 * step. Forcing symmetry after each update stops that drift from compounding
 * (and keeps an asymmetric P from masquerading as a real attitude kick). */
static inline void m_mat_symmetrize(float *M, int n) {
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++) {
      float avg = 0.5f * (M[i * n + j] + M[j * n + i]);
      M[i * n + j] = avg;
      M[j * n + i] = avg;
    }
}

/**
 * @brief 3x3 inverse via cofactors. Returns false (leaving @p inv untouched)
 *        if the matrix is numerically singular.
 */
static inline bool m_mat3_inv(const float *m, float *inv) {
  float c00 = m[4] * m[8] - m[5] * m[7];
  float c01 = m[3] * m[8] - m[5] * m[6];
  float c02 = m[3] * m[7] - m[4] * m[6];
  float det = m[0] * c00 - m[1] * c01 + m[2] * c02;
  /* Relative conditioning guard: an absolute floor (e.g. 1e-20) is meaningless
   * across scales and never trips for a well-scaled matrix. Reference the
   * largest magnitude element cubed (det's units) so a near-singular matrix is
   * rejected before 1/det amplifies float32 noise into a huge, wrong inverse.
   * eps ~ 1e-6 sits an order of magnitude above float32 machine epsilon. */
  float scale = m_fabsf(m[0]);
  for (int i = 1; i < 9; i++) {
    float a = m_fabsf(m[i]);
    if (a > scale)
      scale = a;
  }
  if (m_fabsf(det) <= 1e-6f * scale * scale * scale)
    return false;
  float idet = 1.0f / det;
  inv[0] = c00 * idet;
  inv[1] = -(m[1] * m[8] - m[2] * m[7]) * idet;
  inv[2] = (m[1] * m[5] - m[2] * m[4]) * idet;
  inv[3] = -c01 * idet;
  inv[4] = (m[0] * m[8] - m[2] * m[6]) * idet;
  inv[5] = -(m[0] * m[5] - m[2] * m[3]) * idet;
  inv[6] = c02 * idet;
  inv[7] = -(m[0] * m[7] - m[1] * m[6]) * idet;
  inv[8] = (m[0] * m[4] - m[1] * m[3]) * idet;
  return true;
}

/**
 * @brief 4x4 inverse via Gauss-Jordan elimination with partial pivoting.
 *        Returns false (leaving @p inv untouched) if the matrix is numerically
 *        singular. @p m and @p inv are row-major [16]; @p inv may alias nothing
 *        that is read after the call (the input is fully copied up front).
 */
static inline bool m_mat4_inv(const float *m, float *inv) {
  float a[4][8];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      a[i][j] = m[i * 4 + j];
      a[i][j + 4] = (i == j) ? 1.0f : 0.0f;
    }
  /* Relative singularity floor, same rationale as m_mat3_inv: reference the
   * largest element magnitude so the guard is scale-invariant. */
  float scale = m_fabsf(m[0]);
  for (int i = 1; i < 16; i++) {
    float v = m_fabsf(m[i]);
    if (v > scale)
      scale = v;
  }
  float eps = 1e-6f * (scale > 0.0f ? scale : 1.0f);
  for (int col = 0; col < 4; col++) {
    int piv = col;
    float best = m_fabsf(a[col][col]);
    for (int r = col + 1; r < 4; r++) {
      float v = m_fabsf(a[r][col]);
      if (v > best) {
        best = v;
        piv = r;
      }
    }
    if (best <= eps)
      return false;
    if (piv != col)
      for (int j = 0; j < 8; j++) {
        float t = a[col][j];
        a[col][j] = a[piv][j];
        a[piv][j] = t;
      }
    float invp = 1.0f / a[col][col];
    for (int j = 0; j < 8; j++)
      a[col][j] *= invp;
    for (int r = 0; r < 4; r++) {
      if (r == col)
        continue;
      float f = a[r][col];
      if (m_fabsf(f) < 1e-20f)
        continue;
      for (int j = 0; j < 8; j++)
        a[r][j] -= f * a[col][j];
    }
  }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      inv[i * 4 + j] = a[i][j + 4];
  return true;
}

/* ----------------------------------------------------------------------------
 * 3-vector helpers
 * --------------------------------------------------------------------------*/

/** Skew-symmetric (cross-product) matrix M(3x3) such that M*x = v cross x. */
static inline void m_skew3(const float v[3], float M[9]) {
  M[0] = 0.0f;   M[1] = -v[2]; M[2] = v[1];
  M[3] = v[2];   M[4] = 0.0f;  M[5] = -v[0];
  M[6] = -v[1];  M[7] = v[0];  M[8] = 0.0f;
}

static inline float m_vec3_dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline float m_vec3_norm(const float a[3]) {
  return m_sqrt(m_vec3_dot(a, a));
}

/* ----------------------------------------------------------------------------
 * Quaternion geometry (Hamilton, scalar-first)
 * --------------------------------------------------------------------------*/

static inline void m_quat_normalize(quaternion_t *q) {
  float n = m_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (n > 0.0f) {
    float inv = 1.0f / n;
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
  }
}

/** out = a (x) b  (Hamilton product). out must not alias a or b. */
static inline void m_quat_mul(const quaternion_t *a, const quaternion_t *b,
                              quaternion_t *out) {
  out->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
  out->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
  out->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
  out->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

/** out = R(q) * v   (body -> world). */
static inline void m_quat_rotate(const quaternion_t *q, const float v[3],
                                 float out[3]) {
  float w = q->w, x = q->x, y = q->y, z = q->z;
  float xx = x * x, yy = y * y, zz = z * z;
  float xy = x * y, xz = x * z, yz = y * z;
  float wx = w * x, wy = w * y, wz = w * z;
  out[0] = (1.0f - 2.0f * (yy + zz)) * v[0] + 2.0f * (xy - wz) * v[1] +
           2.0f * (xz + wy) * v[2];
  out[1] = 2.0f * (xy + wz) * v[0] + (1.0f - 2.0f * (xx + zz)) * v[1] +
           2.0f * (yz - wx) * v[2];
  out[2] = 2.0f * (xz - wy) * v[0] + 2.0f * (yz + wx) * v[1] +
           (1.0f - 2.0f * (xx + yy)) * v[2];
}

/** out = R(q)^T * v   (world -> body). */
static inline void m_quat_rotate_inv(const quaternion_t *q, const float v[3],
                                     float out[3]) {
  quaternion_t qc = {q->w, -q->x, -q->y, -q->z};
  m_quat_rotate(&qc, v, out);
}

/**
 * @brief Exponential map: small rotation vector (rad) -> unit quaternion.
 *
 * dq = [cos(|t|/2), sin(|t|/2) * t/|t|]; first-order near zero. Used to
 * fold an error-state attitude correction into a nominal quaternion.
 */
static inline void m_quat_exp(const float dtheta[3], quaternion_t *dq) {
  float t2 = dtheta[0] * dtheta[0] + dtheta[1] * dtheta[1] +
             dtheta[2] * dtheta[2];
  float theta = m_sqrt(t2);
  if (theta < 1e-6f) {
    dq->w = 1.0f;
    dq->x = 0.5f * dtheta[0];
    dq->y = 0.5f * dtheta[1];
    dq->z = 0.5f * dtheta[2];
  } else {
    float half = 0.5f * theta;
    float s = m_sin(half) / theta;
    dq->w = m_cos(half);
    dq->x = s * dtheta[0];
    dq->y = s * dtheta[1];
    dq->z = s * dtheta[2];
  }
  m_quat_normalize(dq);
}

/** ZYX Euler (degrees) -> unit quaternion. */
static inline void m_quat_from_euler_deg(float roll, float pitch, float yaw,
                                         quaternion_t *q) {
  float r = to_radians(roll) * 0.5f;
  float p = to_radians(pitch) * 0.5f;
  float y = to_radians(yaw) * 0.5f;
  float cr = m_cos(r), sr = m_sin(r);
  float cp = m_cos(p), sp = m_sin(p);
  float cy = m_cos(y), sy = m_sin(y);
  q->w = cr * cp * cy + sr * sp * sy;
  q->x = sr * cp * cy - cr * sp * sy;
  q->y = cr * sp * cy + sr * cp * sy;
  q->z = cr * cp * sy - sr * sp * cy;
}

/** Unit quaternion -> ZYX Euler (degrees), pitch clamped at the poles. */
static inline void m_quat_to_euler_deg(const quaternion_t *q, float *roll,
                                       float *pitch, float *yaw) {
  *roll = to_degrees(m_atan2(2.0f * (q->w * q->x + q->y * q->z),
                             1.0f - 2.0f * (q->x * q->x + q->y * q->y)));
  float sinp = 2.0f * (q->w * q->y - q->z * q->x);
  if (sinp > 1.0f)
    sinp = 1.0f;
  else if (sinp < -1.0f)
    sinp = -1.0f;
  *pitch = to_degrees(m_asin(sinp));
  *yaw = to_degrees(m_atan2(2.0f * (q->w * q->z + q->x * q->y),
                            1.0f - 2.0f * (q->y * q->y + q->z * q->z)));
}

#endif /* VAYU_MATHS_LINALG_H */
