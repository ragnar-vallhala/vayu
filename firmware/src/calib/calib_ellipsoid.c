/* Sensor-agnostic ellipsoid fit — small fixed-size float linear algebra used to
 * recover an offset (ellipsoid centre) and full 3x3 shape matrix from samples
 * that should lie on a sphere. Shared by the magnetometer and accelerometer (and
 * any future sensor) so they reuse the same math.
 * See include/calib/calib_ellipsoid.h. */

#include "calib/calib_ellipsoid.h"
#include "maths/maths_interface.h"

#define FABS_F(x) ((x) < 0.0f ? -(x) : (x))
#define SQRT_F(x) m_sqrt(x)

/* Positive cube root via range reduction (cbrt(8x)=2 cbrt(x)) + Newton.
 * Uses only multiply/compare so it is independent of libm extras. */
static float cbrt_pos(float x) {
  if (x <= 0.0f)
    return 0.0f;
  float f = 1.0f;
  while (x > 1.0f) {
    x *= 0.125f;
    f *= 2.0f;
  }
  while (x < 0.125f) {
    x *= 8.0f;
    f *= 0.5f;
  }
  float y = 0.75f; // x now in [0.125, 1], cube root in [0.5, 1]
  for (int i = 0; i < 10; i++)
    y = (2.0f * y + x / (y * y)) / 3.0f;
  return y * f;
}

/* Solve A x = b for a 9x9 system via Gauss-Jordan with partial pivoting.
 * A (row-major) and b are destroyed. Returns 0 on success, -1 if singular. */
static int solve9x9(float A[81], float b[9], float x[9]) {
  const int n = 9;
  for (int col = 0; col < n; col++) {
    int prow = col;
    float best = FABS_F(A[col * n + col]);
    for (int r = col + 1; r < n; r++) {
      float v = FABS_F(A[r * n + col]);
      if (v > best) {
        best = v;
        prow = r;
      }
    }
    if (best < 1e-12f)
      return -1;
    if (prow != col) {
      for (int c = 0; c < n; c++) {
        float tmp = A[col * n + c];
        A[col * n + c] = A[prow * n + c];
        A[prow * n + c] = tmp;
      }
      float tb = b[col];
      b[col] = b[prow];
      b[prow] = tb;
    }
    float invd = 1.0f / A[col * n + col];
    for (int c = 0; c < n; c++)
      A[col * n + c] *= invd;
    b[col] *= invd;
    for (int r = 0; r < n; r++) {
      if (r == col)
        continue;
      float fct = A[r * n + col];
      for (int c = 0; c < n; c++)
        A[r * n + c] -= fct * A[col * n + c];
      b[r] -= fct * b[col];
    }
  }
  for (int i = 0; i < n; i++)
    x[i] = b[i];
  return 0;
}

/* Symmetric 3x3 eigen-decomposition via cyclic Jacobi rotations.
 * Eigenvalues -> w[3]; eigenvectors as columns of V (row-major 3x3). */
static void jacobi_eig3(const float Ain[9], float w[3], float V[9]) {
  float a[9];
  for (int i = 0; i < 9; i++)
    a[i] = Ain[i];
  V[0] = 1; V[1] = 0; V[2] = 0;
  V[3] = 0; V[4] = 1; V[5] = 0;
  V[6] = 0; V[7] = 0; V[8] = 1;
  const int pq[3][2] = {{0, 1}, {0, 2}, {1, 2}};
  for (int sweep = 0; sweep < 50; sweep++) {
    if (FABS_F(a[1]) + FABS_F(a[2]) + FABS_F(a[5]) < 1e-12f)
      break;
    for (int k = 0; k < 3; k++) {
      int p = pq[k][0], q = pq[k][1];
      float apq = a[p * 3 + q];
      if (FABS_F(apq) < 1e-15f)
        continue;
      float phi = 0.5f * (a[q * 3 + q] - a[p * 3 + p]) / apq;
      float tt = (phi >= 0.0f ? 1.0f : -1.0f) /
                 (FABS_F(phi) + SQRT_F(phi * phi + 1.0f));
      float c = 1.0f / SQRT_F(tt * tt + 1.0f);
      float s = tt * c;
      for (int i = 0; i < 3; i++) { // A <- A J (columns p,q)
        float aip = a[i * 3 + p], aiq = a[i * 3 + q];
        a[i * 3 + p] = c * aip - s * aiq;
        a[i * 3 + q] = s * aip + c * aiq;
      }
      for (int i = 0; i < 3; i++) { // A <- J^T A (rows p,q)
        float api = a[p * 3 + i], aqi = a[q * 3 + i];
        a[p * 3 + i] = c * api - s * aqi;
        a[q * 3 + i] = s * api + c * aqi;
      }
      for (int i = 0; i < 3; i++) { // V <- V J
        float vip = V[i * 3 + p], viq = V[i * 3 + q];
        V[i * 3 + p] = c * vip - s * viq;
        V[i * 3 + q] = s * vip + c * viq;
      }
    }
  }
  w[0] = a[0];
  w[1] = a[4];
  w[2] = a[8];
}

/* Inverse of a 3x3 (row-major). Returns 0 on success, -1 if singular. */
static int inv3x3(const float m[9], float out[9]) {
  float c00 = m[4] * m[8] - m[5] * m[7];
  float c01 = m[5] * m[6] - m[3] * m[8];
  float c02 = m[3] * m[7] - m[4] * m[6];
  float det = m[0] * c00 + m[1] * c01 + m[2] * c02;
  if (FABS_F(det) < 1e-20f)
    return -1;
  float invdet = 1.0f / det;
  out[0] = c00 * invdet;
  out[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
  out[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
  out[3] = c01 * invdet;
  out[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
  out[5] = (m[2] * m[3] - m[0] * m[5]) * invdet;
  out[6] = c02 * invdet;
  out[7] = (m[1] * m[6] - m[0] * m[7]) * invdet;
  out[8] = (m[0] * m[4] - m[1] * m[3]) * invdet;
  return 0;
}

/* Fit an ellipsoid to the accumulated normal equations S p = t (each sample
 * contributed row r = [x^2,y^2,z^2,2yz,2xz,2xy,2x,2y,2z], target 1), then
 * recover the offset and a volume-preserving 3x3 shape matrix:
 *   model:  x^T Q x + 2 u^T x = 1
 *   center: c = -Q^-1 u                              (hard iron / bias)
 *   soft:   M = detQ^(-1/6) * Q^(1/2)                (maps ellipsoid -> sphere
 *           of radius = geometric-mean semi-axis, so corrected |v| stays in the
 *           physical range)
 * Returns 0 on success; -1 if the system is singular or Q is not
 * positive-definite (degenerate / planar data). */
int calib_fit_ellipsoid(float S[81], float t[9], float offset[3], float soft[9]) {
  float p[9];
  if (solve9x9(S, t, p) != 0)
    return -1;

  float Q[9] = {p[0], p[5], p[4], p[5], p[1], p[3], p[4], p[3], p[2]};
  float u[3] = {p[6], p[7], p[8]};

  float Qinv[9];
  if (inv3x3(Q, Qinv) != 0)
    return -1;
  offset[0] = -(Qinv[0] * u[0] + Qinv[1] * u[1] + Qinv[2] * u[2]);
  offset[1] = -(Qinv[3] * u[0] + Qinv[4] * u[1] + Qinv[5] * u[2]);
  offset[2] = -(Qinv[6] * u[0] + Qinv[7] * u[1] + Qinv[8] * u[2]);

  float w[3], V[9];
  jacobi_eig3(Q, w, V);
  if (w[0] <= 0.0f || w[1] <= 0.0f || w[2] <= 0.0f)
    return -1; // not an ellipsoid (hyperboloid / planar data)

  float sq[3] = {SQRT_F(w[0]), SQRT_F(w[1]), SQRT_F(w[2])};
  float detQ = w[0] * w[1] * w[2];
  float scale = 1.0f / cbrt_pos(SQRT_F(detQ)); // detQ^(-1/6)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float acc = 0.0f; // (V diag(sqrt(w)) V^T)_{ij}
      for (int k = 0; k < 3; k++)
        acc += V[i * 3 + k] * sq[k] * V[j * 3 + k];
      soft[i * 3 + j] = scale * acc;
    }
  }
  return 0;
}
