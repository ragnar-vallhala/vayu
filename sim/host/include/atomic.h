#ifndef VAIOS_ATOMIC_H
#define VAIOS_ATOMIC_H
// Host-native atomic_t for the SITL build. vaios keeps atomic_t in the port
// layer (portable/<arch>/atomic.h), so the host port supplies its own — same
// plain-ops shape the host test stub uses. The SITL scheduler drives the
// firmware tasks cooperatively, so a plain volatile counter is sufficient
// (no LDREX/STREX, which is Cortex-M only).
#include <stdint.h>

typedef struct atomic {
  volatile int32_t counter;
} atomic_t;

static inline void atomic_set(atomic_t *v, int32_t i) { v->counter = i; }
static inline int32_t atomic_get(atomic_t *v) { return v->counter; }
static inline void atomic_inc(atomic_t *v) { v->counter++; }
static inline void atomic_dec(atomic_t *v) { v->counter--; }

#endif // VAIOS_ATOMIC_H
