#include "est/est.h"

/* @noreq first-order LPF support util (init). */
void lpf_init(lpf_t *lpf, float alpha) {
  lpf->alpha = alpha;
  lpf->output = 0.0f;
}

/* @noreq first-order LPF support util (apply). */
float lpf_apply(lpf_t *lpf, float input) {
  lpf->output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->output);
  return lpf->output;
}
