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
/* The IRQ ownership registry (firmware/src/sys/irq_registry.c).
 *
 * An interrupt vector holds one callback, so a second claim silently evicts
 * the first and the symptom shows up somewhere else entirely, much later. That
 * happened for real: the telemetry UART's TX DMA shared a stream with SDIO,
 * SDIO re-grabbed the completion IRQ on every card write, and telemetry died
 * after the first save (fault line F12).
 *
 * So the check that matters is the CONFLICT one -- claim must return false
 * when somebody else already holds the vector. A registry that always says yes
 * is worse than none, because it looks like the question was asked. */
#include <stdio.h>

#include "sys/irq_registry.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

int main(void) {
  printf("== irq_registry ==\n");

  printf("  [1] an unclaimed vector has no owner\n");
  CHECK(irq_registry_owner(42) == NULL, "starts unowned");

  printf("  [2] a first claim is granted and recorded\n");
  CHECK(irq_registry_claim(42, "telemetry"), "claim granted");
  CHECK(irq_registry_owner(42) != NULL, "owner recorded");

  printf("  [3] the same owner may re-claim; it is not a conflict\n");
  {
    /* Init paths get re-run -- a channel reopened, a task restarted. Reporting
     * a collision against yourself would train people to ignore the log. */
    CHECK(irq_registry_claim(42, "telemetry"), "idempotent re-claim");
    CHECK(irq_registry_owner(42) != NULL, "still owned");
  }

  printf("  [4] a DIFFERENT owner is refused, and takes it anyway\n");
  {
    /* This is the whole point. The caller is about to attach its callback no
     * matter what we return -- the vector will change hands. False is how the
     * registry says "you just evicted somebody", and the new owner is recorded
     * because it is now the truth. */
    CHECK(!irq_registry_claim(42, "sdio"), "conflict reported");
    const char *now = irq_registry_owner(42);
    CHECK(now != NULL && now[0] == 's', "the evictor is recorded as owner");
  }

  printf("  [5] vectors are independent\n");
  {
    CHECK(irq_registry_claim(7, "rc"), "a second vector claims cleanly");
    CHECK(irq_registry_owner(7) != NULL, "recorded");
    CHECK(irq_registry_owner(42) != NULL, "the first is undisturbed");
    CHECK(irq_registry_owner(99) == NULL, "an untouched vector stays free");
  }

  printf("  [6] release frees the slot, but only for its owner\n");
  {
    irq_registry_release(7, "not-rc");
    CHECK(irq_registry_owner(7) != NULL, "a stranger cannot release it");
    irq_registry_release(7, "rc");
    CHECK(irq_registry_owner(7) == NULL, "the owner can");
    CHECK(irq_registry_claim(7, "rc-again"), "and the slot is reusable");
    irq_registry_release(7, "rc-again");
  }

  printf("  [7] a full table refuses rather than scribbling\n");
  {
    /* 42 is still held from [4], so IRQ_REGISTRY_MAX-1 more fill it. */
    for (int i = 0; i < IRQ_REGISTRY_MAX - 1; i++) {
      CHECK(irq_registry_claim(1000 + i, "filler"), "fills");
    }
    CHECK(!irq_registry_claim(5000, "overflow"), "a full table refuses");
    CHECK(irq_registry_owner(5000) == NULL, "and records nothing");
    CHECK(irq_registry_owner(42) != NULL, "existing entries survive");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
