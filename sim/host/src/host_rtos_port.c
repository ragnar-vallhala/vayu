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
/*
 * host_rtos_port.c -- ucontext host port for the REAL vaios scheduler (Phase 4
 * of firmware/docs/plans/sitl-lockstep-sim.md).
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
#include <pthread.h>
#include <ucontext.h>

#include "host_clock.h"
#include "memory.h" /* HEAP_SIZE */
#include "task.h"

/* ---- per-task execution context -------------------------------------- */
/* One ucontext per task, indexed by task_id (SITL uses a handful of fixed
 * tasks + idle, ids well under this cap). The ucontext runs on the kernel-
 * allocated task stack (TCB.mem_block), so the kernel's accounting still owns
 * the memory; we only borrow it as the ucontext stack. */
#define HOST_PORT_MAX_TASKS 32
static ucontext_t task_ctx[HOST_PORT_MAX_TASKS];
static ucontext_t stepper_ctx; /* the driver/stepper context */
static pthread_t sched_thread; /* set by scheduler_start */
static int in_scheduler = 0;   /* 1 while the scheduler is running */

extern TCB *current_task;        /* kernel-owned */
extern void set_next_task(void); /* kernel policy: pick highest-ready */
extern void task_exit(void);     /* TASK_EXIT trampoline target */
extern int wake_up_delayed_tasks_isr(void); /* kernel: move due delayed→ready */

/* ---- the sim clock for the RTOS path --------------------------------- *
 * The kernel's delay/timeout logic counts SysTick ticks (systick_count, 1 tick =
 * TICK_PERIOD_US = 1000). On hardware a 1 kHz SysTick ISR bumps it; here the
 * stepper bumps it one tick per IMU sample (the firmware's loop rate), so sim
 * time is exactly the sample count — fully deterministic. v_get_ticks reads it;
 * scheduler_running gates vaios.c's v_delay (busy-wait before start, task_delay
 * after). Both are kernel-extern'd; we own the definitions on host. */
volatile uint32_t systick_count = 0;
extern uint8_t scheduler_running;       /* defined in kernel/task.c */
volatile uint32_t critical_nesting = 0; /* port.h critical-section nesting */
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
  task_exit(); /* task body returned -> kernel cleanup */
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
  uc->uc_link = &stepper_ctx; /* if a task ever returns, fall to stepper */
  const uintptr_t p = (uintptr_t)task;
  makecontext(uc, (void (*)(void))task_trampoline, 2, (unsigned int)(p >> 32),
              (unsigned int)(p & 0xffffffffu));
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
  sched_thread = pthread_self(); /* the only thread allowed to swapcontext */
  scheduler_running = 1;         /* gates v_delay onto cooperative task_delay */
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
    wake_up_delayed_tasks_isr(); /* delayed→ready; reschedule on next run */
  }
}

/* ---- stepper entry: run the scheduler until it goes idle (quiescent) ---
 * Resume the current task (or idle) and let the cooperative scheduler run until
 * the idle task's cpu_relax swaps back here. On return the tick is fully settled
 * (PWM written). Call after advancing the clock + injecting a sensor sample. */
void host_rtos_run_until_idle(void) {
  if (!current_task)
    return; /* not started yet */
  in_scheduler = 1;
  swapcontext(&stepper_ctx, ctx_of(current_task));
  in_scheduler = 0;
}

/* ---- port: idle behaviour = reschedule, else yield to the stepper ----
 * vaios's idle task loops calling v_port_hw_cpu_idle() (its WFI hook; it was
 * hal_cpu_idle before 0.2.0 moved the facade out of NavHAL's namespace). On
 * hardware a SysTick ISR preempts idle to run a woken task; cooperatively there
 * is no preemption, so this must do the reschedule itself: pick the highest-
 * ready task and switch to it; if nothing but idle is ready the tick is
 * quiescent → swap back to the stepper. This makes "scheduler reached idle with
 * nothing ready" the race-free settle signal.
 *
 * It is therefore NOT a no-op, unlike every other port on host: a no-op here
 * leaves the idle task spinning inside the kernel's while(1) and run_step()
 * never returns. */
extern TCB *idle_task;
void v_port_hw_cpu_idle(void) {
  set_next_task(); /* may select a just-woken task */
  if (current_task != idle_task)
    swapcontext(ctx_of(idle_task), ctx_of(current_task));
  else
    swapcontext(ctx_of(idle_task), &stepper_ctx); /* quiescent → to stepper */
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
void v_port_enable_interrupts(void) {
  if (crit_nesting)
    crit_nesting--;
}
uint32_t v_port_get_psp(void) { return 0; }
/* Pend the context switch. vaios 0.2.0 ended the block-and-switch path here
 * (ipc.c marks the task BLOCKED, leaves the critical section, then pends PendSV)
 * where it used to call task_yield() itself -- which is why this was a documented
 * no-op. A no-op now means semaphore_take_common marks the task blocked and then
 * RETURNS INTO IT: the task spins in its own while(1) and no yield ever happens.
 * Cooperatively there is no PendSV to pend, so the switch has to happen here.
 *
 * Only from the scheduler thread, and only once tasks are running: a give from
 * the UART RX thread also lands here, and swapcontext there would hijack that
 * thread into a task's stack. Those wakes are picked up by the next
 * set_next_task() on the scheduler thread instead, which is the same deferral
 * the hardware gets from an ISR that pends PendSV. */
void v_port_trigger_pendsv(void) {
  if (!in_scheduler || current_task == NULL)
    return; /* pre-scheduler, or the stepper's own context */
  if (!pthread_equal(pthread_self(), sched_thread))
    return; /* another thread's give; the scheduler will see it */
  task_yield();
}
void v_port_halt(void) { abort(); }

/* ---- port: hardware bring-up (v_port_hw_*) --------------------------- */
/* The kernel's v_init() calls these during boot. On host the clock/FPU/systick
 * are modelled by the virtual clock + the stepper tick, the console is stderr,
 * and there is no SD peripheral here (the host VFS is disk-backed elsewhere). */
void v_port_hw_clock_init(int internal_clock_setup) {
  (void)internal_clock_setup;
}
void v_port_hw_fpu_enable(void) {}
void v_port_hw_systick_init(uint32_t period_us) { (void)period_us; }
void v_port_hw_sched_irq_init(void) {}
void v_port_hw_console_init(uint32_t baud, void (*dma_cb)(void)) {
  (void)baud;
  (void)dma_cb;
}
int v_port_hw_sdio_init(void) { return 0; }
int v_port_hw_sdio_card_init(void) { return 0; }

/* ---- Port surface vaios 0.2.0 added ---------------------------------------
 *
 * The kernel reaches hardware and the memory map only through these, so a port
 * that does not define them fails at link. Each one answers for the host the
 * way the ARM port answers for the F401, so SITL and the FC agree wherever the
 * answer is a fact about the kernel rather than about silicon.
 */

/* On ARM this is the SRAM window [0x20000000, _estack). The host has no known
 * map, so the only corruption this can still catch is a NULL -- which the ARM
 * port rejects too, since 0 is outside its window. */
int v_port_ptr_is_ram(const void *p) { return p != NULL; }

/* Memory outside the task's own block that a task may reach. On ARM that is
 * flash, read-only, granted by an MPU region. SITL runs no unprivileged tasks
 * (nothing flips CONTROL.nPRIV), so syscall validation never consults this; it
 * answers "no such region" rather than inventing a permission, which fails
 * closed if that ever stops being true. */
int v_port_user_region(uintptr_t a, int write, uintptr_t *end) {
  (void)a;
  (void)write;
  (void)end;
  return 0;
}

/* The kernel owns the task stack here: the ucontext runs on TCB.mem_block, which
 * the kernel allocates and frees. Same as the ARM port, and unlike vaios's own
 * host RUN port, which mmaps its stacks and must unmap them. */
void v_port_free_task_stack(TCB *task) { (void)task; }

/* No NVIC, and the single-threaded stepper never runs kernel code from a
 * handler: always thread mode, so no priority and VECTACTIVE 0. */
uint32_t v_port_hw_active_irq_priority(uint32_t *vectactive_out) {
  if (vectactive_out != NULL) {
    *vectactive_out = 0u;
  }
  return 0u;
}

void v_port_hw_debug_init(void) {} /* no debug block to keep clocked */
void v_port_mpu_init(void) {}      /* no MPU */

/* host_vfs.c backs the card with host files, so the slot is never empty --
 * reporting 0 here would make the VFS bring-up report "no card" and skip. */
int v_port_hw_sdio_card_present(void) { return 1; }

/* Peripheral-bus teardown on task exit. kernel/periph_bus.c is not linked into
 * SITL and nothing registers an endpoint, so there is nothing to tear down. */
void v_pbus_task_teardown(struct Task_Control_Block *t) { (void)t; }
