// dsp_interface.c
#include "dsp_interface.h"
#include "memory.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
//========================
// DSP Math Fucntions
// =======================
float dsp_sin(float x) {
  return arm_sin_f32(x);
}

float dsp_cos(float x) {
  return arm_cos_f32(x);
}

float dsp_atan2(float y, float x) {
  float result;
  arm_atan2_f32(y, x, &result);
  return result;
}

// -----------------
// Square root wrapper
// -----------------
float dsp_sqrt(float x) {
  float result;
  arm_sqrt_f32(x, &result);
  return result;
}

//------------------------------
// Helper
// -----------------------------
void *memset(void *s, int c, size_t n) {
  uint8_t *bytes = (uint8_t *)s;
  for (int i = 0; i < n; i++) {
    *bytes = c;
    bytes++;
  }
  return s;
}
void *memcpy(void *dest, const void *src, size_t n) {
  uint8_t *d = (uint8_t *)dest;
  const uint8_t *s = (const uint8_t *)src;
  for (size_t i = 0; i < n; i++) {
    d[i] = s[i];
  }
  return dest;
}

// -----------------------------
// PID
// -----------------------------
void dsp_pid_init(dsp_pid_t *pid, float32_t kp, float32_t ki, float32_t kd) {
  if (!pid)
    return;
  pid->pid.Kp = kp;
  pid->pid.Ki = ki;
  pid->pid.Kd = kd;
  /* arm_pid_init_f32 expects a pointer and a reset flag */
  arm_pid_init_f32(&pid->pid, 1);
}

float32_t dsp_pid_update(dsp_pid_t *pid, float32_t error, float32_t dt) {
  if (!pid)
    return 0.0f;

  /* CMSIS PID is designed to be called with the error value.
     If you want time-scaling for Ki/Kd you can pre-scale error or
     tune Ki/Kd accordingly. Here we ignore dt and rely on tuned gains. */
  (void)dt;
  return arm_pid_f32(&pid->pid, error);
}

// -----------------------------
// FIR
// -----------------------------
bool dsp_fir_init(dsp_fir_t *fir, uint16_t num_taps, const float32_t *coeffs,
                  float32_t *state, uint32_t block_size) {
  if (!fir || !coeffs || !state || num_taps == 0 || block_size == 0)
    return false;

  arm_fir_init_f32(&fir->fir, (uint32_t)num_taps, (float32_t *)coeffs, state,
                   block_size);
  return true;
}

float32_t dsp_fir_update(dsp_fir_t *fir, float32_t input) {
  if (!fir)
    return 0.0f;

  float32_t in = input;
  float32_t out = 0.0f;
  /* process single sample blocks using arm_fir_f32 (blockSize = 1) */
  arm_fir_f32(&fir->fir, &in, &out, 1);
  return out;
}

// -----------------------------
// IIR (biquad cascade DF1)
// -----------------------------
bool dsp_iir_init(dsp_iir_t *iir, uint8_t num_stages, const float32_t *coeffs,
                  float32_t *state) {
  if (!iir || num_stages == 0 || !coeffs || !state)
    return false;
  arm_biquad_cascade_df1_init_f32(&iir->iir, (uint32_t)num_stages,
                                  (float32_t *)coeffs, state);
  return true;
}

float32_t dsp_iir_update(dsp_iir_t *iir, float32_t input) {
  if (!iir)
    return 0.0f;
  float32_t in = input;
  float32_t out = 0.0f;
  arm_biquad_cascade_df1_f32(&iir->iir, &in, &out, 1);
  return out;
}

// -----------------------------
// Utility
// -----------------------------
float32_t dsp_vector_norm(const float32_t *vec, uint32_t len) {
  if (!vec || len == 0)
    return 0.0f;
  float32_t dot = 0.0f;
  arm_dot_prod_f32(vec, vec, len, &dot);
  float32_t out;
  arm_sqrt_f32(dot, &out);
  return out;
}

float32_t dsp_limit(float32_t value, float32_t min_val, float32_t max_val) {
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}

// -----------------------------
// Kalman filter (matrix form)
// -----------------------------
// Notes on memory ownership:
// - The user must provide buffers for A, B, H, Q, R, P, x (flat row-major
// arrays).
// - dsp_kalman_init will initialize arm_matrix_instance_f32 for these buffers.
// - dsp_kalman_init allocates a buffer for K internally (size state_dim x
// meas_dim)
//   and stores it in kf->K. This buffer is NOT freed by this implementation.
//   If you want to free it, add a dsp_kalman_deinit(kf) that frees kf->K.pData.
// - Temporary matrices used inside predict/update are allocated and freed
// inside calls.

bool dsp_kalman_init(dsp_kalman_t *kf, uint16_t state_dim, uint16_t meas_dim,
                     uint16_t control_dim, float32_t *A, float32_t *B,
                     float32_t *H, float32_t *Q, float32_t *R, float32_t *P,
                     float32_t *x) {
  if (!kf || state_dim == 0 || meas_dim == 0)
    return false;
  if (!A || !H || !Q || !R || !P || !x)
    return false; // B may be NULL if no control input

  /* initialize instances for provided buffers */
  arm_mat_init_f32(&kf->A, (uint16_t)state_dim, (uint16_t)state_dim, A);
  if (B) {
    arm_mat_init_f32(&kf->B, (uint16_t)state_dim, (uint16_t)control_dim, B);
  } else {
    /* make B a 0x0 (or 0xN with NULL data) — we avoid using it if NULL */
    arm_mat_init_f32(&kf->B, (uint16_t)state_dim, (uint16_t)control_dim, NULL);
  }
  arm_mat_init_f32(&kf->H, (uint16_t)meas_dim, (uint16_t)state_dim, H);
  arm_mat_init_f32(&kf->Q, (uint16_t)state_dim, (uint16_t)state_dim, Q);
  arm_mat_init_f32(&kf->R, (uint16_t)meas_dim, (uint16_t)meas_dim, R);
  arm_mat_init_f32(&kf->P, (uint16_t)state_dim, (uint16_t)state_dim, P);
  arm_mat_init_f32(&kf->x, (uint16_t)state_dim, (uint16_t)1, x);

  /* allocate K matrix buffer: state_dim x meas_dim */
  uint32_t k_elems = (uint32_t)state_dim * (uint32_t)meas_dim;
  if (k_elems == 0)
    return false;
  float32_t *kbuf = (float32_t *)v_malloc(sizeof(float32_t) * k_elems);
  if (!kbuf)
    return false;
  memset(kbuf, 0, sizeof(float32_t) * k_elems);
  arm_mat_init_f32(&kf->K, (uint16_t)state_dim, (uint16_t)meas_dim, kbuf);

  return true;
}

/* Helper: allocate a matrix instance with allocated data (rows*cols) */
static arm_matrix_instance_f32 allocate_temp_mat(uint16_t rows, uint16_t cols) {
  arm_matrix_instance_f32 M;
  uint32_t n = (uint32_t)rows * (uint32_t)cols;
  float32_t *buf = NULL;
  if (n > 0) {
    buf = (float32_t *)v_malloc(sizeof(float32_t) * n);
    if (buf)
      memset(buf, 0, sizeof(float32_t) * n);
  }
  arm_mat_init_f32(&M, rows, cols, buf);
  return M;
}

/* Helper: free temp matrix data buffer (does not free the instance itself) */
static void free_temp_mat(arm_matrix_instance_f32 *M) {
  if (!M)
    return;
  if (M->pData) {
    v_free(M->pData);
    M->pData = NULL;
  }
}

/* Predict: x = A*x + B*u
           P = A*P*A^T + Q
   u may be NULL if no control input.
*/
void dsp_kalman_predict(dsp_kalman_t *kf, const float32_t *u) {
  if (!kf)
    return;

  uint16_t n = kf->A.numRows; /* state dimension */
  uint16_t m = kf->H.numRows; /* measurement dimension */
  uint16_t c = kf->B.numCols; /* control_dim (might be 0) */

  arm_status status;

  /* temp vectors / matrices */
  /* tmp1 = A * x  (n x 1) */
  arm_matrix_instance_f32 tmp1 = allocate_temp_mat(n, 1);
  if (!tmp1.pData)
    return;

  status = arm_mat_mult_f32(&kf->A, &kf->x, &tmp1);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&tmp1);
    return;
  }

  /* if B and u provided, add B*u */
  if (kf->B.pData != NULL && u != NULL && c > 0) {
    arm_matrix_instance_f32 um;                  /* control vector c x 1 */
    arm_mat_init_f32(&um, c, 1, (float32_t *)u); /* u assumed contiguous */

    arm_matrix_instance_f32 Bu = allocate_temp_mat(n, 1);
    if (!Bu.pData) {
      free_temp_mat(&tmp1);
      return;
    }
    status = arm_mat_mult_f32(&kf->B, &um, &Bu);
    if (status == ARM_MATH_SUCCESS) {
      /* tmp1 = tmp1 + Bu */
      for (uint32_t i = 0; i < (uint32_t)n; ++i) {
        tmp1.pData[i] += Bu.pData[i];
      }
    }
    free_temp_mat(&Bu);
  }

  /* x <- tmp1 */
  memcpy(kf->x.pData, tmp1.pData, sizeof(float32_t) * n);
  free_temp_mat(&tmp1);

  /* P = A*P*A^T + Q */
  /* tmpA = A * P  (n x n) */
  arm_matrix_instance_f32 tmpA = allocate_temp_mat(n, n);
  if (!tmpA.pData)
    return;
  status = arm_mat_mult_f32(&kf->A, &kf->P, &tmpA);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&tmpA);
    return;
  }

  /* tmpAT = tmpA * A^T  (n x n) */
  arm_matrix_instance_f32 AT = allocate_temp_mat(kf->A.numCols, kf->A.numRows);
  if (!AT.pData) {
    free_temp_mat(&tmpA);
    return;
  }
  status = arm_mat_trans_f32(&kf->A, &AT);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&tmpA);
    free_temp_mat(&AT);
    return;
  }

  arm_matrix_instance_f32 tmpP = allocate_temp_mat(n, n);
  if (!tmpP.pData) {
    free_temp_mat(&tmpA);
    free_temp_mat(&AT);
    return;
  }
  status = arm_mat_mult_f32(&tmpA, &AT, &tmpP);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&tmpA);
    free_temp_mat(&AT);
    free_temp_mat(&tmpP);
    return;
  }

  /* P = tmpP + Q */
  for (uint32_t i = 0; i < (uint32_t)(n * n); ++i) {
    kf->P.pData[i] = tmpP.pData[i] + kf->Q.pData[i];
  }

  free_temp_mat(&tmpA);
  free_temp_mat(&AT);
  free_temp_mat(&tmpP);
}

/* Update: z is measurement vector of size meas_dim (m x 1) */
void dsp_kalman_update(dsp_kalman_t *kf, const float32_t *z) {
  if (!kf || !z)
    return;

  uint16_t n = kf->A.numRows; /* state dimension */
  uint16_t m = kf->H.numRows; /* measurement dimension */

  arm_status status;

  /* y = z - H*x  (m x 1) */
  arm_matrix_instance_f32 zl;
  arm_mat_init_f32(
      &zl, m, 1,
      (float32_t *)z); /* z is const but arm_mat functions don't modify it */

  arm_matrix_instance_f32 Hx = allocate_temp_mat(m, 1);
  if (!Hx.pData)
    return;
  status = arm_mat_mult_f32(&kf->H, &kf->x, &Hx);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    return;
  }

  arm_matrix_instance_f32 y = allocate_temp_mat(m, 1);
  if (!y.pData) {
    free_temp_mat(&Hx);
    return;
  }
  /* y = z - Hx */
  for (uint32_t i = 0; i < (uint32_t)m; ++i) {
    y.pData[i] = zl.pData[i] - Hx.pData[i];
  }

  /* S = H * P * H^T + R  (m x m) */
  arm_matrix_instance_f32 HP = allocate_temp_mat(m, n);
  if (!HP.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    return;
  }
  status = arm_mat_mult_f32(&kf->H, &kf->P, &HP);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    return;
  }

  arm_matrix_instance_f32 HT = allocate_temp_mat(kf->H.numCols, kf->H.numRows);
  if (!HT.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    return;
  }
  status = arm_mat_trans_f32(&kf->H, &HT);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    return;
  }

  arm_matrix_instance_f32 S = allocate_temp_mat(m, m);
  if (!S.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    return;
  }
  status = arm_mat_mult_f32(&HP, &HT, &S);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    return;
  }

  /* S = S + R */
  for (uint32_t i = 0; i < (uint32_t)(m * m); ++i) {
    S.pData[i] += kf->R.pData[i];
  }

  /* Compute inverse of S */
  arm_matrix_instance_f32 S_inv = allocate_temp_mat(m, m);
  if (!S_inv.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    return;
  }
  status = arm_mat_inverse_f32(&S, &S_inv);
  if (status != ARM_MATH_SUCCESS) {
    /* cannot invert S: numerical issue - bail out */
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    free_temp_mat(&S_inv);
    return;
  }

  /* Compute K = P * H^T * S_inv  (n x m) */
  arm_matrix_instance_f32 PHt = allocate_temp_mat(n, m);
  if (!PHt.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    free_temp_mat(&S_inv);
    return;
  }
  status = arm_mat_mult_f32(&kf->P, &HT, &PHt);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    free_temp_mat(&S_inv);
    free_temp_mat(&PHt);
    return;
  }

  arm_matrix_instance_f32 Ktmp = allocate_temp_mat(n, m);
  if (!Ktmp.pData) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    free_temp_mat(&S_inv);
    free_temp_mat(&PHt);
    return;
  }
  status = arm_mat_mult_f32(&PHt, &S_inv, &Ktmp);
  if (status != ARM_MATH_SUCCESS) {
    free_temp_mat(&Hx);
    free_temp_mat(&y);
    free_temp_mat(&HP);
    free_temp_mat(&HT);
    free_temp_mat(&S);
    free_temp_mat(&S_inv);
    free_temp_mat(&PHt);
    free_temp_mat(&Ktmp);
    return;
  }

  /* copy Ktmp into kf->K (our persistent buffer) */
  if (kf->K.pData) {
    memcpy(kf->K.pData, Ktmp.pData,
           sizeof(float32_t) * ((uint32_t)n * (uint32_t)m));
  }

  /* x = x + K * y   (n x 1) */
  arm_matrix_instance_f32 Ky = allocate_temp_mat(n, 1);
  if (!Ky.pData) { /* still must free temps */
  }
  status = arm_mat_mult_f32(&kf->K, &y, &Ky);
  if (status == ARM_MATH_SUCCESS) {
    for (uint32_t i = 0; i < (uint32_t)n; ++i) {
      kf->x.pData[i] += Ky.pData[i];
    }
  }

  /* P = (I - K*H) * P  -- compute KH first */
  arm_matrix_instance_f32 KH = allocate_temp_mat(n, n);
  if (!KH.pData) { /* continue to cleanup */
  }
  status = arm_mat_mult_f32(&kf->K, &kf->H, &KH);
  if (status == ARM_MATH_SUCCESS) {
    /* build I - KH in place in KH */
    for (uint32_t r = 0; r < n; ++r) {
      for (uint32_t cidx = 0; cidx < n; ++cidx) {
        uint32_t idx = r * (uint32_t)n + cidx;
        float32_t val = KH.pData[idx];
        if (r == cidx)
          KH.pData[idx] = 1.0f - val;
        else
          KH.pData[idx] = -val;
      }
    }

    arm_matrix_instance_f32 newP = allocate_temp_mat(n, n);
    if (newP.pData) {
      status = arm_mat_mult_f32(&KH, &kf->P, &newP);
      if (status == ARM_MATH_SUCCESS) {
        memcpy(kf->P.pData, newP.pData,
               sizeof(float32_t) * (uint32_t)n * (uint32_t)n);
      }
      free_temp_mat(&newP);
    }
  }

  /* cleanup temps */
  free_temp_mat(&Hx);
  free_temp_mat(&y);
  free_temp_mat(&HP);
  free_temp_mat(&HT);
  free_temp_mat(&S);
  free_temp_mat(&S_inv);
  free_temp_mat(&PHt);
  free_temp_mat(&Ktmp);
  free_temp_mat(&Ky);
  free_temp_mat(&KH);
}
