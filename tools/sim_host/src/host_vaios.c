/*
 * host_vaios.c -- host-side replacements for the vaios kernel primitives
 * the controller and helper sources call.
 *
 * Maps:
 *   task_create / task_exit / task_yield  -> pthreads
 *   v_delay / v_get_ticks / task_delay    -> CLOCK_MONOTONIC + nanosleep
 *   v_semaphore_*                          -> POSIX sem_t (heap-allocated)
 *   v_mutex_*                              -> pthread_mutex_t (heap-allocated)
 *   v_malloc / v_free / v_mem*             -> libc
 *   v_panic                                -> fprintf + abort
 *   scheduler_*, v_system_init, v_start    -> no-ops (pthreads carry the work)
 *
 * vaios's spsc_* is portable C — we link against extern/vaios/kernel/structure.c
 * directly, so it's not redefined here.
 */
#define _GNU_SOURCE
#include "ipc.h"
#include "task.h"
#include "utils.h"
#include "vaios.h"

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ---- monotonic clock (1 tick = 1 ms) ---------------------------------- */
static struct timespec ts_origin;
static int origin_set = 0;

static void ensure_origin(void) {
    if (!origin_set) {
        clock_gettime(CLOCK_MONOTONIC, &ts_origin);
        origin_set = 1;
    }
}

uint32_t v_get_ticks(void) {
    ensure_origin();
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t ms = (uint64_t)(now.tv_sec - ts_origin.tv_sec) * 1000ULL
                + (now.tv_nsec - ts_origin.tv_nsec) / 1000000;
    return (uint32_t)ms;
}

/* ---- delays ----------------------------------------------------------- */
void v_delay(uint32_t ms) {
    struct timespec req = { .tv_sec = ms / 1000,
                            .tv_nsec = (long)(ms % 1000) * 1000000L };
    while (nanosleep(&req, &req) == -1 && errno == EINTR) {
        /* retry */
    }
}

/* The firmware also calls task_delay(ticks). On vaios ticks are usec; with
 * our SYSTICK_PERIOD=1000 us this maps to ms. */
void task_delay(uint32_t ticks) {
    v_delay(ticks);
}

/* Drift-free periodic delay shim (cf. vaios task_delay_until). v_get_ticks() is
 * ms on the host, so periods are in ms. Sleeps until the absolute deadline
 * *last_wake + period; returns false without sleeping on overrun. */
bool task_delay_until(uint32_t *last_wake, uint32_t period) {
    if (last_wake == NULL || period == 0)
        return false;
    uint32_t wake = *last_wake + period;
    *last_wake = wake;
    int32_t remaining = (int32_t)(wake - v_get_ticks());
    if (remaining <= 0)
        return false;
    v_delay((uint32_t)remaining);
    return true;
}

/* ---- task creation: 1 task <-> 1 pthread ------------------------------ */
typedef struct {
    void (*entry)(void *);
    void *arg;
} task_trampoline_arg_t;

static void *task_trampoline(void *p) {
    task_trampoline_arg_t *t = (task_trampoline_arg_t *)p;
    void (*entry)(void *) = t->entry;
    void *arg = t->arg;
    free(t);
    entry(arg);
    return NULL;
}

static uint32_t next_task_id = 1;

uint32_t task_create(void (*entry)(void *), void *arg, uint32_t stack_size,
                     uint32_t priority) {
    (void)priority;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (stack_size < 65536) stack_size = 65536; /* libc + pthread overhead */
    pthread_attr_setstacksize(&attr, stack_size);

    task_trampoline_arg_t *t = malloc(sizeof(*t));
    t->entry = entry;
    t->arg = arg;

    pthread_t th;
    if (pthread_create(&th, &attr, task_trampoline, t) != 0) {
        fprintf(stderr, "host_vaios: pthread_create failed\n");
        free(t);
        return 0;
    }
    pthread_detach(th);
    pthread_attr_destroy(&attr);
    return next_task_id++;
}

uint32_t task_create_named(void (*entry)(void *), void *arg,
                           uint32_t stack_size, uint32_t priority,
                           const char *name) {
    (void)name; /* host shim has no TCB to store the name in */
    return task_create(entry, arg, stack_size, priority);
}

/* The host shim has no TCB registry, so naming is a no-op and lookups return
 * "". Task names are a target-side diagnostic; SITL doesn't model them. */
void task_set_name(uint32_t task_id, const char *name) {
    (void)task_id;
    (void)name;
}

const char *task_get_name(const struct Task_Control_Block *task) {
    (void)task;
    return "";
}

const char *task_get_name_by_id(uint32_t task_id) {
    (void)task_id;
    return "";
}

void task_exit(void) {
    pthread_exit(NULL);
}

void task_exit_request(uint32_t task_id) { (void)task_id; }

void task_yield(void) { sched_yield(); }

/* ---- panic ------------------------------------------------------------ */
volatile uint8_t is_panicking = 0;

void v_panic(const char *file, int line, const char *fmt, ...) {
    is_panicking = 1;
    fprintf(stderr, "PANIC %s:%d ", file, line);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    abort();
}

/* ---- mem* shims so the kernel callers link --------------------------- */
void *v_memset(void *s, int c, unsigned int n) { return memset(s, c, n); }
void *v_memcpy(void *dest, const void *src, unsigned int n) {
    return memcpy(dest, src, n);
}
uint32_t v_strlen(const char *s) { return (uint32_t)strlen(s); }
int v_strcmp(const char *a, const char *b) { return strcmp(a, b); }
int v_strncmp(const char *a, const char *b, int n) { return strncmp(a, b, (size_t)n); }
float v_atof(const char *s) { return (float)strtod(s, NULL); }

/* ---- v_malloc / v_free: just libc ------------------------------------ */
void *v_malloc(size_t size) { return malloc(size); }
void v_free(void *ptr) { free(ptr); }
uint32_t v_get_heap_size(void) { return 0; }
uint32_t v_get_heap_allocation_count(void) { return 0; }
uint32_t v_get_heap_allocation_size(void) { return 0; }
void v_heap_memory_init(void) {}

/* ---- print / log: route to stderr ------------------------------------ */
void print(const char *str) { fputs(str, stderr); }
void print_fmt(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}
int print_fmt_buf(char *out, uint32_t out_size, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(out, out_size, fmt, ap);
    va_end(ap); return n;
}
int vaprint_fmt_buf(char *out, size_t out_size, const char *fmt, va_list ap) {
    return vsnprintf(out, out_size, fmt, ap);
}
void v_log(Log_Type type, const char *msg, ...) {
    (void)type;
    va_list ap; va_start(ap, msg); vfprintf(stderr, msg, ap); fputc('\n', stderr);
    va_end(ap);
}
void v_log_flush(void) { fflush(stderr); }
void direct_dma_print(const uint8_t *bytes, uint32_t len) {
    fwrite(bytes, 1, len, stderr);
}
void dma_tx_complete_callback(void) {}

/* ---- semaphores -------------------------------------------------------- */
typedef struct {
    sem_t sem;
    uint32_t max_count;
} host_sem_t;

SemaphoreHandle_t v_semaphore_create_binary(void) {
    host_sem_t *s = malloc(sizeof(*s));
    sem_init(&s->sem, 0, 0);
    s->max_count = 1;
    return s;
}

SemaphoreHandle_t v_semaphore_create_binary_static(StaticSemaphore_t *buf) {
    (void)buf;
    return v_semaphore_create_binary();
}

SemaphoreHandle_t v_semaphore_create_counting(uint32_t max_count,
                                              uint32_t initial_count) {
    host_sem_t *s = malloc(sizeof(*s));
    sem_init(&s->sem, 0, (unsigned int)initial_count);
    s->max_count = max_count;
    return s;
}

SemaphoreHandle_t v_semaphore_create_counting_static(uint32_t max_count,
                                                     uint32_t initial_count,
                                                     StaticSemaphore_t *buf) {
    (void)buf;
    return v_semaphore_create_counting(max_count, initial_count);
}

int v_semaphore_take(SemaphoreHandle_t sem, uint32_t ticks_to_wait) {
    host_sem_t *s = (host_sem_t *)sem;
    if (ticks_to_wait == 0xFFFFFFFFu) {
        while (sem_wait(&s->sem) == -1 && errno == EINTR) { }
        return VA_PASS;
    }
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += ticks_to_wait / 1000;
    deadline.tv_nsec += (long)(ticks_to_wait % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    while (sem_timedwait(&s->sem, &deadline) == -1) {
        if (errno == EINTR) continue;
        return VA_FAIL;
    }
    return VA_PASS;
}

int v_semaphore_give(SemaphoreHandle_t sem) {
    host_sem_t *s = (host_sem_t *)sem;
    return sem_post(&s->sem) == 0 ? VA_PASS : VA_FAIL;
}

int v_semaphore_give_from_isr(SemaphoreHandle_t sem,
                              int *higher_priority_task_woken) {
    if (higher_priority_task_woken) *higher_priority_task_woken = 0;
    return v_semaphore_give(sem);
}

/* ---- mutexes ----------------------------------------------------------- */
typedef struct {
    pthread_mutex_t m;
} host_mutex_t;

MutexHandle_t v_mutex_create(void) {
    host_mutex_t *m = malloc(sizeof(*m));
    pthread_mutex_init(&m->m, NULL);
    return m;
}

MutexHandle_t v_mutex_create_static(StaticSemaphore_t *buf) {
    (void)buf;
    return v_mutex_create();
}

MutexHandle_t v_mutex_create_recursive(void) {
    host_mutex_t *m = malloc(sizeof(*m));
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&m->m, &attr);
    pthread_mutexattr_destroy(&attr);
    return m;
}

MutexHandle_t v_mutex_create_recursive_static(StaticSemaphore_t *buf) {
    (void)buf;
    return v_mutex_create_recursive();
}

int v_mutex_lock(MutexHandle_t h, uint32_t ticks_to_wait) {
    host_mutex_t *m = (host_mutex_t *)h;
    if (ticks_to_wait == 0xFFFFFFFFu) {
        return pthread_mutex_lock(&m->m) == 0 ? VA_PASS : VA_FAIL;
    }
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += ticks_to_wait / 1000;
    deadline.tv_nsec += (long)(ticks_to_wait % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    return pthread_mutex_timedlock(&m->m, &deadline) == 0 ? VA_PASS : VA_FAIL;
}

int v_mutex_unlock(MutexHandle_t h) {
    host_mutex_t *m = (host_mutex_t *)h;
    return pthread_mutex_unlock(&m->m) == 0 ? VA_PASS : VA_FAIL;
}

int v_mutex_lock_recursive(MutexHandle_t h, uint32_t t) {
    return v_mutex_lock(h, t);
}
int v_mutex_unlock_recursive(MutexHandle_t h) { return v_mutex_unlock(h); }

/* ---- vaios init/start: no-ops on host -------------------------------- */
void v_init(vaios_init_config_t *cfg)        { (void)cfg; }
void v_system_init(vaios_init_config_t *cfg) { (void)cfg; }
void v_start(void) { }
void v_stop(void)  { }
void scheduler_init(void)  { }
void scheduler_start(void) { }
