/* Sensor-agnostic ellipsoid fit — small fixed-size float linear algebra used to
 * recover an offset (ellipsoid centre) and full 3x3 shape matrix from samples
 * that should lie on a sphere. Shared by the magnetometer and accelerometer (and
 * any future sensor) so they reuse the same math.
 * See include/calib/calib_ellipsoid.h. */

#include "calib/calib_ellipsoid.h"
#include "maths/maths_interface.h"
#include <stddef.h> /* NULL */

#define FABS_F(x) ((x) < 0.0f ? -(x) : (x))
#define SQRT_F(x) m_sqrt(x)

/* Positive cube root via range reduction (cbrt(8x)=2 cbrt(x)) + Newton.
 * Uses only multiply/compare so it is independent of libm extras.
 * @noreq Math primitive (cube root) for the ellipsoid fit. */
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
 * A (row-major) and b are destroyed. Returns 0 on success, -1 if singular.
 * @noreq Math primitive (9x9 linear solver) for the ellipsoid fit. */
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
 * Eigenvalues -> w[3]; eigenvectors as columns of V (row-major 3x3).
 * @noreq Math primitive (symmetric 3x3 eigen-decomposition) for the fit. */
static void jacobi_eig3(const float Ain[9], float w[3], float V[9]) {
  float a[9];
  for (int i = 0; i < 9; i++)
    a[i] = Ain[i];
  V[0] = 1;
  V[1] = 0;
  V[2] = 0;
  V[3] = 0;
  V[4] = 1;
  V[5] = 0;
  V[6] = 0;
  V[7] = 0;
  V[8] = 1;
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

/* Inverse of a 3x3 (row-major). Returns 0 on success, -1 if singular.
 * @noreq Math primitive (3x3 inverse) for the ellipsoid fit. */
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
 * positive-definite (degenerate / planar data).
 *
 * @implements SNS-CAL-101, SNS-CAL-103 */
int calib_fit_ellipsoid(float S[81], float t[9], float offset[3],
                        float soft[9]) {
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

/* See include/calib/calib_ellipsoid.h. Closed-form 6-side accel fit. */
int calib_fit_sixpoint(const float (*pts)[3], int npts, float g,
                       float offset[3], float soft[9]) {
  if (pts == NULL || npts < 6)
    return -1;

  /* Select the best-aligned pose for each side (axis a, sign s: 0=+,1=-) by the
   * cosine between the pose direction and the axis. A missing side leaves a low
   * best-cosine (caught by COS_MIN); classify by direction so pose order and the
   * exact hold angle don't matter. */
  const float COS_MIN =
      0.80f; /* ~37deg: a genuine face hold clears this easily */
  int sel[3][2];
  float bestcos[3][2];
  for (int a = 0; a < 3; a++)
    for (int s = 0; s < 2; s++) {
      sel[a][s] = -1;
      bestcos[a][s] = COS_MIN;
    }
  for (int p = 0; p < npts; p++) {
    float n2 =
        pts[p][0] * pts[p][0] + pts[p][1] * pts[p][1] + pts[p][2] * pts[p][2];
    if (n2 < 1e-6f)
      continue;
    float inv_n = 1.0f / SQRT_F(n2);
    for (int a = 0; a < 3; a++) {
      float c = pts[p][a] * inv_n; /* cosine with +a axis */
      if (c > bestcos[a][0]) {
        bestcos[a][0] = c;
        sel[a][0] = p;
      }
      if (-c > bestcos[a][1]) {
        bestcos[a][1] = -c;
        sel[a][1] = p;
      }
    }
  }

  /* Every side must be present and each pose used at most once. */
  int used[6], nu = 0;
  for (int a = 0; a < 3; a++)
    for (int s = 0; s < 2; s++) {
      if (sel[a][s] < 0)
        return -1; /* a side is missing */
      for (int u = 0; u < nu; u++)
        if (used[u] == sel[a][s])
          return -1; /* same pose claimed two sides -> not a 6-side set */
      used[nu++] = sel[a][s];
    }

  const float *Pp[3][2];
  for (int a = 0; a < 3; a++) {
    Pp[a][0] = pts[sel[a][0]];
    Pp[a][1] = pts[sel[a][1]];
  }

  /* offset = midpoint of opposing sides, per axis. */
  for (int k = 0; k < 3; k++)
    offset[k] = 0.5f * (Pp[k][0][k] + Pp[k][1][k]);

  /* A = the three +g poses minus offset (row k = +k pose). */
  float A[9];
  for (int k = 0; k < 3; k++)
    for (int j = 0; j < 3; j++)
      A[k * 3 + j] = Pp[k][0][j] - offset[j];

  /* We need accel_T with  accel_T * (pose_+k - offset) = g*e_k for each axis k,
   * i.e. accel_T * A^T = g*I  ->  accel_T = g * (A^-1)^T = g/det * cofactor(A).
   * (PX4 forms g*A^-1 and keeps only its diagonal, where the transpose is a
   * no-op; vayu applies the full matrix so we keep the cofactor form.) */
  float c00 = A[4] * A[8] - A[5] * A[7]; /* C00 */
  float c01 = A[5] * A[6] - A[3] * A[8]; /* C01 */
  float c02 = A[3] * A[7] - A[4] * A[6]; /* C02 */
  float det = A[0] * c00 + A[1] * c01 + A[2] * c02;
  if (FABS_F(det) < 1e-9f)
    return -1;
  float inv = g / det; /* fold the *g into the inverse scaling */
  soft[0] = c00 * inv;
  soft[1] = c01 * inv;
  soft[2] = c02 * inv;
  soft[3] = (A[2] * A[7] - A[1] * A[8]) * inv; /* C10 */
  soft[4] = (A[0] * A[8] - A[2] * A[6]) * inv; /* C11 */
  soft[5] = (A[1] * A[6] - A[0] * A[7]) * inv; /* C12 */
  soft[6] = (A[1] * A[5] - A[2] * A[4]) * inv; /* C20 */
  soft[7] = (A[2] * A[3] - A[0] * A[5]) * inv; /* C21 */
  soft[8] = (A[0] * A[4] - A[1] * A[3]) * inv; /* C22 */

  /* PX4-faithful: keep only the per-axis scale, discard misalignment. */
  soft[1] = soft[2] = soft[3] = soft[5] = soft[6] = soft[7] = 0.0f;
  return 0;
}
