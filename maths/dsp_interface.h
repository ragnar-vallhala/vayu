#ifndef DSP_INTERFACE_H
#define DSP_INTERFACE_H

#include "arm_math.h" // CMSIS-DSP core header
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//========================
// DSP Math Fucntions
// =======================
#define M_PI 3.14159265358979323846f
#define M_PI_OVER_180 (M_PI / 180.0f)
#define M_INV_PI (1.0f / M_PI)

// Trigonometric
float m_sin(float x);            // sin(x) in radians
float m_cos(float x);            // cos(x) in radians
float m_atan2(float y, float x); // atan2(y, x) in radians

// Square root
float m_sqrt(float x);

// Degree/Radian conversions
static inline float m_deg2rad(float deg) { return deg * M_PI_OVER_180; }
static inline float m_rad2deg(float rad) { return rad * (180.0f / M_PI); }

// =============================
// DSP Interface for Drone FC
// =============================

// ---------- Types ----------
typedef struct {
  arm_pid_instance_f32 pid;
} dsp_pid_t;

typedef struct {
  arm_biquad_casd_df1_inst_f32 iir;
} dsp_iir_t;

typedef struct {
  arm_fir_instance_f32 fir;
} dsp_fir_t;

typedef struct {
  arm_matrix_instance_f32 A; // State transition
  arm_matrix_instance_f32 B; // Control input
  arm_matrix_instance_f32 H; // Measurement
  arm_matrix_instance_f32 Q; // Process noise
  arm_matrix_instance_f32 R; // Measurement noise
  arm_matrix_instance_f32 P; // Estimate error covariance
  arm_matrix_instance_f32 x; // State vector
  arm_matrix_instance_f32 K; // Kalman gain
} dsp_kalman_t;

// ---------- PID ----------
void dsp_pid_init(dsp_pid_t *pid, float32_t kp, float32_t ki, float32_t kd);
float32_t dsp_pid_update(dsp_pid_t *pid, float32_t error, float32_t dt);

// ---------- FIR ----------
bool dsp_fir_init(dsp_fir_t *fir, uint16_t num_taps, const float32_t *coeffs,
                  float32_t *state, uint32_t block_size);
float32_t dsp_fir_update(dsp_fir_t *fir, float32_t input);

// ---------- IIR ----------
bool dsp_iir_init(dsp_iir_t *iir, uint8_t num_stages, const float32_t *coeffs,
                  float32_t *state);
float32_t dsp_iir_update(dsp_iir_t *iir, float32_t input);

// ---------- Kalman ----------
bool dsp_kalman_init(dsp_kalman_t *kf, uint16_t state_dim, uint16_t meas_dim,
                     uint16_t control_dim, float32_t *A, float32_t *B,
                     float32_t *H, float32_t *Q, float32_t *R, float32_t *P,
                     float32_t *x);

void dsp_kalman_predict(dsp_kalman_t *kf, const float32_t *u);
void dsp_kalman_update(dsp_kalman_t *kf, const float32_t *z);

// ---------- Utility ----------
float32_t dsp_vector_norm(const float32_t *vec, uint32_t len);
float32_t dsp_limit(float32_t value, float32_t min_val, float32_t max_val);

#ifdef __cplusplus
}
#endif

#endif // DSP_INTERFACE_H
