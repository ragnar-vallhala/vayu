#ifndef VAIOS_ATOMIC_H
#define VAIOS_ATOMIC_H
// Host-native atomic_t for the SITL build. vaios moved atomic_t into the port
// layer (portable/<arch>/atomic.h) and removed the generic include/atomic.h,
// so the host port must supply its own — same plain-ops shape the host test
// stub uses. The SITL scheduler drives the firmware tasks cooperatively, so a
// plain volatile counter is sufficient (no LDREX/STREX, which is Cortex-M only).
#include <stdint.h>

typedef struct atomic {
  volatile int32_t counter;
} atomic_t;

static inline void atomic_set(atomic_t *v, int32_t i) { v->counter = i; }
static inline int32_t atomic_get(atomic_t *v) { return v->counter; }
static inline void atomic_inc(atomic_t *v) { v->counter++; }
static inline void atomic_dec(atomic_t *v) { v->counter--; }

#endif // VAIOS_ATOMIC_H
