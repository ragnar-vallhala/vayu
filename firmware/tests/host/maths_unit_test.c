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
/* maths_unit_test.c — host unit test for maths_interface.c.
 *
 * Covers the quaternion / vector / scalar helpers used across the estimator and
 * control paths (the FFT half of this module has its own fft_unit_test.c).
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/maths_unit_test.c \
 *       src/maths/maths_interface.c -lm -o /tmp/mt && /tmp/mt
 */
#include "maths/maths_interface.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}
static int near(float a, float b) { return fabsf(a - b) < 1e-4f; }

int main(void) {
  /* 1. clamp */
  printf("Test 1: m_clamp\n");
  check("below -> min", near(m_clamp(-5, -1, 1), -1));
  check("above -> max", near(m_clamp(5, -1, 1), 1));
  check("inside -> val", near(m_clamp(0.3f, -1, 1), 0.3f));

  /* 2. degree/radian macros */
  printf("Test 2: to_radians / to_degrees\n");
  check("180 deg == pi rad", near(to_radians(180.0f), PI));
  check("pi rad == 180 deg", near(to_degrees(PI), 180.0f));

  /* 3. NaN / finite classification */
  printf("Test 3: m_isnan / m_isfinite\n");
  check("nan detected", m_isnan(0.0f / 0.0f) != 0);
  check("finite detected", m_isfinite(1.0f) != 0);
  check("1.0 not nan", m_isnan(1.0f) == 0);

  /* 4. vector normalisation -> unit length */
  printf("Test 4: normalize_vector\n");
  {
    float v[3] = {3.0f, 0.0f, 4.0f};
    vector_t vec = {v, 3};
    normalize_vector(&vec);
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    check("|v| == 1 after normalize", near(n, 1.0f));
    check("3/5 component", near(v[0], 0.6f));
    check("4/5 component", near(v[2], 0.8f));
  }

  /* 5. quaternion normalisation -> unit norm */
  printf("Test 5: normalize_quaternion\n");
  {
    quaternion_t q = {2.0f, 0.0f, 0.0f, 0.0f};
    normalize_quaternion(&q);
    check("scalar quat normalizes to identity",
          near(q.w, 1.0f) && near(q.x, 0));
  }

  /* 6. identity euler -> identity quaternion */
  printf("Test 6: quaternion_from_euler(0,0,0)\n");
  {
    quaternion_t q;
    quaternion_from_euler(0, 0, 0, &q);
    check("identity",
          near(q.w, 1) && near(q.x, 0) && near(q.y, 0) && near(q.z, 0));
  }

  /* 7. 90deg roll -> known quaternion (cos45, sin45, 0, 0) */
  printf("Test 7: quaternion_from_euler(90,0,0)\n");
  {
    quaternion_t q;
    quaternion_from_euler(90.0f, 0, 0, &q);
    float c = cosf((float)(PI / 4)), s = sinf((float)(PI / 4));
    check("w == cos45", near(q.w, c));
    check("x == sin45", near(q.x, s));
    check("y,z == 0", near(q.y, 0) && near(q.z, 0));
  }

  /* 8. multiply by identity is a no-op; conjugate flips the vector part */
  printf("Test 8: quaternion_multiply / conjugate\n");
  {
    quaternion_t id = {1, 0, 0, 0};
    quaternion_t q = {0.5f, 0.5f, 0.5f, 0.5f}; /* already unit */
    quaternion_t out;
    quaternion_multiply(&id, &q, &out);
    check("id * q == q", near(out.w, q.w) && near(out.x, q.x) &&
                             near(out.y, q.y) && near(out.z, q.z));
    quaternion_t cj;
    quaternion_conjugate(&q, &cj);
    check("conjugate negates vector part",
          near(cj.w, q.w) && near(cj.x, -q.x) && near(cj.y, -q.y) &&
              near(cj.z, -q.z));
    /* q * conj(q) == identity for a unit quaternion */
    quaternion_t prod;
    quaternion_multiply(&q, &cj, &prod);
    check("q * conj(q) == identity", near(prod.w, 1) && near(prod.x, 0) &&
                                         near(prod.y, 0) && near(prod.z, 0));
  }

  /* 9. scalar libm wrappers */
  printf("Test 9: scalar wrappers\n");
  check("m_sqrt(9)==3", near(m_sqrt(9.0f), 3.0f));
  check("m_fabsf(-2)==2", near(m_fabsf(-2.0f), 2.0f));
  check("m_atan2(1,1)==pi/4", near(m_atan2(1.0f, 1.0f), (float)(PI / 4)));

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
