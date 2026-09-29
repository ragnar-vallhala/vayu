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
/* See sys/irq_registry.h.
 *
 * A flat array, not a map: there are a dozen vectors in the whole firmware and
 * every claim happens once at init, so a linear scan is free and a table that
 * can be read in a debugger is worth more than a clever structure.
 *
 * Owner strings are stored by POINTER, not copied. Every caller passes a
 * string literal, which lives in flash forever; a caller passing a stack
 * buffer would leave a dangling pointer, so do not. */
#include "sys/irq_registry.h"

#include "storage/fs_owner.h" /* vayu_log */

typedef struct {
  uint32_t vector;
  const char *owner; /* NULL = slot free */
} irq_slot_t;

static irq_slot_t _slots[IRQ_REGISTRY_MAX];

/* Claims happen at init, before the scheduler matters, so no lock. If that
 * ever stops being true this needs one -- the table is shared mutable state. */
static irq_slot_t *_find(uint32_t vector) {
  for (int i = 0; i < IRQ_REGISTRY_MAX; i++) {
    if (_slots[i].owner != NULL && _slots[i].vector == vector) {
      return &_slots[i];
    }
  }
  return NULL;
}

static int _same(const char *a, const char *b) {
  if (a == b) {
    return 1;
  }
  if (a == NULL || b == NULL) {
    return 0;
  }
  while (*a && *a == *b) {
    a++;
    b++;
  }
  return *a == *b;
}

/** @noreq IRQ ownership bookkeeping (fault line F12). */
bool irq_registry_claim(uint32_t vector, const char *owner) {
  irq_slot_t *held = _find(vector);
  if (held != NULL) {
    if (_same(held->owner, owner)) {
      return true; /* idempotent re-claim */
    }
    /* The callback about to be attached will evict the current one. Say so
     * loudly: the symptom otherwise is a peripheral that stops working at
     * some later, unrelated moment. */
    vayu_log("[IRQ] vector %u: %s is taking it from %s -- one of them will "
             "stop working",
             (unsigned)vector, owner ? owner : "?", held->owner);
    held->owner = owner;
    return false;
  }

  for (int i = 0; i < IRQ_REGISTRY_MAX; i++) {
    if (_slots[i].owner == NULL) {
      _slots[i].vector = vector;
      _slots[i].owner = owner;
      return true;
    }
  }
  vayu_log("[IRQ] registry full (%d); vector %u by %s is unrecorded",
           IRQ_REGISTRY_MAX, (unsigned)vector, owner ? owner : "?");
  return false;
}

/** @noreq IRQ ownership bookkeeping. */
void irq_registry_release(uint32_t vector, const char *owner) {
  irq_slot_t *held = _find(vector);
  if (held != NULL && _same(held->owner, owner)) {
    held->owner = NULL;
  }
}

/** @noreq IRQ ownership bookkeeping. */
const char *irq_registry_owner(uint32_t vector) {
  irq_slot_t *held = _find(vector);
  return held ? held->owner : NULL;
}

/** @noreq IRQ ownership bookkeeping. */
void irq_registry_dump(void) {
  for (int i = 0; i < IRQ_REGISTRY_MAX; i++) {
    if (_slots[i].owner != NULL) {
      vayu_log("[IRQ] %u -> %s", (unsigned)_slots[i].vector, _slots[i].owner);
    }
  }
}
