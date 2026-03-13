#ifndef VAYU_MATHS_LPF_H
#define VAYU_MATHS_LPF_H

typedef struct {
  float alpha;
  float output;
} lpf_t;

/**
 * @brief Initialize the LPF
 * @param lpf Pointer to the LPF structure
 * @param alpha Smoothing factor (0.0 to 1.0). Smaller value = more filtering.
 */
void lpf_init(lpf_t *lpf, float alpha);

/**
 * @brief Apply the LPF to a new input sample
 * @param lpf Pointer to the LPF structure
 * @param input New input sample
 * @return Filtered output
 */
float lpf_apply(lpf_t *lpf, float input);

#endif // VAYU_MATHS_LPF_H
