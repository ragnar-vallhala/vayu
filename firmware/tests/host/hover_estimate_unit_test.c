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
/* hover_estimate_unit_test.c — host unit test for the in-flight hover estimate.
 *
 * Pure function under test: hover_est_update(). What matters is that it learns
 * the real hover in steady level flight, and that it REFUSES to learn from
 * anything else — a wrong hover is what flew the airframe into the ceiling, so
 * the gates are the safety-relevant part, not the convergence.
 */
#include "est/hover_estimate.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}
static int near(float a, float b, float tol) { return fabsf(a - b) < tol; }

#define DT 0.004f
#define SEED 0.38f

/* Hold steady level flight at `coll` for `secs`. */
static void steady(hover_est_t *h, float coll, float secs) {
  for (int i = 0; i < (int)(secs / DT); i++)
    hover_est_update(h, true, coll, 0.0f, 0.0f, 1.0f, DT);
}

int main(void) {
  printf("hover estimate\n");

  /* 1. Starts at the seed and is usable immediately. */
  {
    hover_est_t h;
    hover_est_init(&h, SEED);
    check("seeded from the airframe constant", near(h.estimate, SEED, 1e-6f));
    check("not flagged as measured", !h.measured);
  }

  /* 2. Learns the real hover in steady level flight. Seed says 0.38, the craft
   *    is really hovering at 0.30 — it must converge down. */
  {
    hover_est_t h;
    hover_est_init(&h, SEED);
    steady(&h, 0.30f, 30.0f);
    check("converges toward the observed hover",
          near(h.estimate, 0.30f, 0.01f));
    check("flagged as measured", h.measured);
  }

  /* 3. Rate-limited: a single step can only move it by slew*dt, so one bad
   *    sample can never jump the estimate. */
  {
    hover_est_t h;
    hover_est_init(&h, SEED);
    steady(&h, 0.38f, HOVER_EST_SETTLE_S + 0.01f); /* clear the settle gate */
    float before = h.estimate;
    hover_est_update(&h, true, 0.80f, 0.0f, 0.0f, 1.0f, DT);
    check("one sample moves it by at most slew*dt",
          h.estimate - before <= HOVER_EST_SLEW_PER_S * DT + 1e-6f);
  }

  /* 4. THE GATES. None of these may teach it anything. */
  {
    float bad[][5] = {
        /* in_air, climb,  accel, cos_tilt, note-index */
        {0.0f, 0.0f, 0.0f, 1.00f, 0}, /* on the ground */
        {1.0f, 2.0f, 0.0f, 1.00f, 1}, /* climbing */
        {1.0f, 0.0f, 3.0f, 1.00f, 2}, /* accelerating */
        {1.0f, 0.0f, 0.0f, 0.80f, 3}, /* banked ~37 deg */
    };
    const char *why[] = {"on the ground", "while climbing",
                         "while accelerating", "while banked"};
    for (int i = 0; i < 4; i++) {
      hover_est_t h;
      hover_est_init(&h, SEED);
      for (int k = 0; k < 5000; k++)
        hover_est_update(&h, bad[i][0] > 0.5f, 0.60f, bad[i][1], bad[i][2],
                         bad[i][3], DT);
      char msg[80];
      snprintf(msg, sizeof msg, "learns nothing %s", why[i]);
      check(msg, near(h.estimate, SEED, 1e-6f) && !h.measured);
    }
  }

  /* 5. An implausible collective is refused even when everything else looks
   *    steady — at the top of a ballistic arc it briefly does. */
  {
    hover_est_t h;
    hover_est_init(&h, SEED);
    steady(&h, 0.95f, 20.0f);
    check("refuses a collective above the sane band",
          near(h.estimate, SEED, 1e-6f) && !h.measured);
    hover_est_init(&h, SEED);
    steady(&h, 0.02f, 20.0f);
    check("refuses a collective below the sane band",
          near(h.estimate, SEED, 1e-6f) && !h.measured);
  }

  /* 6. Conditions must HOLD — a transient brush past "steady" on the way
   *    through a manoeuvre teaches nothing. */
  {
    hover_est_t h;
    hover_est_init(&h, SEED);
    for (int cycle = 0; cycle < 50; cycle++) {
      for (int i = 0; i < (int)(HOVER_EST_SETTLE_S / DT) - 5; i++)
        hover_est_update(&h, true, 0.30f, 0.0f, 0.0f, 1.0f, DT); /* nearly */
      hover_est_update(&h, true, 0.30f, 5.0f, 0.0f, 1.0f, DT);   /* broken */
    }
    check("a transient never completes the settle window",
          near(h.estimate, SEED, 1e-6f) && !h.measured);
  }

  /* 7. Clamped to the sane band regardless of how long it runs. */
  {
    hover_est_t h;
    hover_est_init(&h, 5.0f);
    check("a silly seed is clamped into the band",
          h.estimate <= HOVER_EST_MAX + 1e-6f && h.estimate >= HOVER_EST_MIN);
  }

  printf(fails ? "\nFAILED (%d)\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
