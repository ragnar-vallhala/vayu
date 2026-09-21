/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
