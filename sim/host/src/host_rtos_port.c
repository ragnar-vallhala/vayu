/*
 * host_rtos_port.c -- ucontext host port for the REAL vaios scheduler (Phase 4
 * of docs/plans/sitl-lockstep-sim.md).
 *
 * Instead of stubbing the vaios scheduler and running each task as a free pthread
 * (the legacy host_vaios.c path — nondeterministic, never exercises the real
 * scheduler), this port runs the ACTUAL kernel/task.c scheduler on the host with
 * cooperative ucontext context switches. One OS thread, one task runnable at a
 * time, chosen by vaios's own O(1) priority policy → deterministic AND maximally
 * faithful (the flight scheduler itself is under test).
 *
 * Policy lives in the kernel (set_next_task, ready_bitmap, delayed_list); this
 * file is only the MECHANISM the cortex-m4 port.c provides on hardware:
 *   init_task_stack / scheduler_start / task_yield / load_next_task_from_isr,
 *   the critical-section + cpu-relax port hooks, and the v_port_hw_* bring-up.
 *
 * Stepper integration (the key idea): vaios runs the IDLE task whenever nothing
 * else is ready. On hardware idle spins (v_port_cpu_relax → WFI). Here idle's
 * relax instead SWAPS BACK TO THE STEPPER — so "scheduler reached idle" is an
 * explicit, race-free quiescence signal: the pipeline has fully settled for this
 * tick. The stepper (host_rtos_run_tick) then advances the virtual clock, wakes
 * delayed tasks, injects the next sensor sample, and resumes the scheduler.
 *
 * Only compiled into the opt-in `vayu_sitl_rtos` target; the legacy pthread SITL
 * (host_vaios.c) is untouched.
 */
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ucontext.h>

#include "host_clock.h"
#include "memory.h"   /* HEAP_SIZE */
#include "task.h"

/* ---- per-task execution context -------------------------------------- */
/* One ucontext per task, indexed by task_id (SITL uses a handful of fixed
 * tasks + idle, ids well under this cap). The ucontext runs on the kernel-
 * allocated task stack (TCB.mem_block), so the kernel's accounting still owns
 * the memory; we only borrow it as the ucontext stack. */
#define HOST_PORT_MAX_TASKS 32
static ucontext_t task_ctx[HOST_PORT_MAX_TASKS];
static ucontext_t stepper_ctx;          /* the driver/stepper context */
static int        in_scheduler = 0;     /* 1 while the scheduler is running */

extern TCB *current_task;               /* kernel-owned */
extern void set_next_task(void);        /* kernel policy: pick highest-ready */
extern void task_exit(void);            /* TASK_EXIT trampoline target */
extern int  wake_up_delayed_tasks_isr(void); /* kernel: move due delayed→ready */

/* ---- the sim clock for the RTOS path --------------------------------- *
 * The kernel's delay/timeout logic counts SysTick ticks (systick_count, 1 tick =
 * SYSTICK_PERIOD = 1 ms). On hardware a 1 kHz SysTick ISR bumps it; here the
 * stepper bumps it one tick per IMU sample (the firmware's loop rate), so sim
 * time is exactly the sample count — fully deterministic. v_get_ticks reads it;
 * scheduler_running gates vaios.c's v_delay (busy-wait before start, task_delay
 * after). Both are kernel-extern'd; we own the definitions on host. */
volatile uint32_t systick_count = 0;
extern uint8_t scheduler_running;         /* defined in kernel/task.c */
volatile uint32_t critical_nesting = 0;   /* port.h critical-section nesting */
uint32_t v_get_ticks(void) { return systick_count; }

/* Kernel heap backing store: memory.c uses &_heap_start as the base of a
 * HEAP_SIZE region (on target this is a linker symbol in SRAM). */
uint32_t _heap_start[HEAP_SIZE / sizeof(uint32_t)] __attribute__((aligned(8)));

/* perf.c cycle-counter port hooks — no DWT on host; report a coarse count off
 * the sim clock (perf numbers are diagnostic only under lockstep). */
uint32_t v_port_hw_cycle_counter_read(void) { return systick_count * 1000u; }
void v_port_hw_cycle_counter_init(void) {}

static ucontext_t *ctx_of(const TCB *t) {
  return &task_ctx[t->task_id % HOST_PORT_MAX_TASKS];
}

/* makecontext can only pass int args, so split the 64-bit TCB* into two. */
static void task_trampoline(unsigned int hi, unsigned int lo) {
  TCB *t = (TCB *)(((uintptr_t)hi << 32) | (uintptr_t)lo);
  t->entry(t->arg);
  task_exit();                          /* task body returned -> kernel cleanup */
}

/* ---- port: stack/context init ---------------------------------------- */
/* Build the task's initial ucontext on its own stack so a later swapcontext
 * starts it at entry(arg). Keep TCB.sp pointed in-range (the kernel's stack
 * watermark guard reads it; on host it's vestigial — real overflow is caught by
 * the OS guard page / ucontext stack bounds). */
void init_task_stack(TCB *task) {
  ucontext_t *uc = ctx_of(task);
  getcontext(uc);
  uc->uc_stack.ss_sp = task->mem_block;
  uc->uc_stack.ss_size = task->stack_size;
  uc->uc_link = &stepper_ctx;           /* if a task ever returns, fall to stepper */
  const uintptr_t p = (uintptr_t)task;
  makecontext(uc, (void (*)(void))task_trampoline, 2,
              (unsigned int)(p >> 32), (unsigned int)(p & 0xffffffffu));
  /* in-range sp so the (host-vestigial) watermark check can't false-fire */
  task->sp = task->mem_block + (task->stack_size / sizeof(uint32_t)) - 4;
}

/* ---- port: context switch -------------------------------------------- */
/* Yield: let the kernel pick the next task, then swap to it. Called by a task
 * that has just parked itself (delay/semaphore) so set_next_task picks another
 * (or idle). No-op if the pick is unchanged. */
void task_yield(void) {
  TCB *from = current_task;
  set_next_task();
  TCB *to = current_task;
  if (to != from)
    swapcontext(ctx_of(from), ctx_of(to));
}

/* ISR-path reschedule (e.g. after a tick wakes a higher-priority task). Same as
 * task_yield here — the host has no separate ISR stack. */
void load_next_task_from_isr(void) { task_yield(); }

/* Start the scheduler: pick the first task and jump into it. Returns to the
 * stepper when the system goes idle (idle's cpu_relax swaps back). */
void scheduler_start(void) {
  scheduler_running = 1;                /* gates v_delay onto cooperative task_delay */
  set_next_task();
  in_scheduler = 1;
  swapcontext(&stepper_ctx, ctx_of(current_task));
  in_scheduler = 0;
}

/* ---- stepper: advance the sim clock one tick ------------------------- *
 * Bump SysTick `ms` times, waking any delayed/timed-out tasks each tick, then
 * let the scheduler drain (run_until_idle). Called by the single-threaded
 * stepper once per IMU sample (ms = the sample period). */
void host_rtos_tick(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    systick_count++;
    wake_up_delayed_tasks_isr();        /* delayed→ready; reschedule on next run */
  }
}

/* ---- stepper entry: run the scheduler until it goes idle (quiescent) ---
 * Resume the current task (or idle) and let the cooperative scheduler run until
 * the idle task's cpu_relax swaps back here. On return the tick is fully settled
 * (PWM written). Call after advancing the clock + injecting a sensor sample. */
void host_rtos_run_until_idle(void) {
  if (!current_task) return;            /* not started yet */
  in_scheduler = 1;
  swapcontext(&stepper_ctx, ctx_of(current_task));
  in_scheduler = 0;
}

/* ---- port: idle behaviour = reschedule, else yield to the stepper ----
 * vaios's idle task loops calling hal_cpu_idle() (its WFI hook). On hardware a
 * SysTick ISR preempts idle to run a woken task; cooperatively there is no
 * preemption, so hal_cpu_idle must do the reschedule itself: pick the highest-
 * ready task and switch to it; if nothing but idle is ready the tick is
 * quiescent → swap back to the stepper. This makes "scheduler reached idle with
 * nothing ready" the race-free settle signal. (Overrides host_navhal's no-op.) */
extern TCB *idle_task;
void hal_cpu_idle(void) {
  set_next_task();                      /* may select a just-woken task */
  if (current_task != idle_task)
    swapcontext(ctx_of(idle_task), ctx_of(current_task));
  else
    swapcontext(ctx_of(idle_task), &stepper_ctx);  /* quiescent → to stepper */
}

/* Pre-scheduler busy-wait relax (vaios.c v_delay before scheduler_running).
 * The boot path doesn't hit this on host; keep it a cheap no-op. */
void v_port_cpu_relax(void) {}

/* ---- port: critical sections ----------------------------------------- */
/* Cooperative single-threaded scheduler: tasks only switch at explicit yields,
 * so a nesting counter is enough to satisfy the kernel's enter/exit balance.
 * (Cross-thread sensor injection is funnelled through the stepper, not raw
 * concurrent ISRs, so no hardware interrupt masking is needed here.) */
static uint32_t crit_nesting = 0;
void v_port_disable_interrupts(void) { crit_nesting++; }
void v_port_enable_interrupts(void)  { if (crit_nesting) crit_nesting--; }
uint32_t v_port_get_psp(void) { return 0; }
void v_port_trigger_pendsv(void) { /* host switches synchronously in task_yield */ }
void v_port_halt(void) { abort(); }

/* ---- port: hardware bring-up (v_port_hw_*) --------------------------- */
/* The kernel's v_init() calls these during boot. On host the clock/FPU/systick
 * are modelled by the virtual clock + the stepper tick, the console is stderr,
 * and there is no SD peripheral here (the host VFS is disk-backed elsewhere). */
void v_port_hw_clock_init(int internal_clock_setup) { (void)internal_clock_setup; }
void v_port_hw_fpu_enable(void) {}
void v_port_hw_systick_init(uint32_t period_us) { (void)period_us; }
void v_port_hw_sched_irq_init(void) {}
void v_port_hw_console_init(uint32_t baud, void (*dma_cb)(void)) {
  (void)baud; (void)dma_cb;
}
int  v_port_hw_sdio_init(void) { return 0; }
int  v_port_hw_sdio_card_init(void) { return 0; }
