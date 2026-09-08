# Deferred issue #2 — two thirds of the serial-channel .bss is never used

## The issue

`_serial_handlers` is **12,336 B of `.bss` — 30% of the firmware's entire static
footprint**, and the largest single object in the image by a factor of three.
Found 2026-09-09 by `tools/dev/alloc_budget.py`, not by anyone reading the code.

```c
#define CHANNEL_TX_BUF_SIZE 2048u                       /* channel.c:13   */
typedef struct {
  ...
  byte buffers[2][CHANNEL_TX_BUF_SIZE];   /* ping-pong, 4096 B           */
  ...
} serial_channel_handle_t;                /* 4112 B with the fields      */

static serial_channel_handle_t _serial_handlers[MAX_SERIAL_HANDLERS] = {0};
#define MAX_SERIAL_HANDLERS 3                           /* variables.h:65 */
```

**Only one slot is ever occupied.** `get_handler()` is called exactly once in
the whole tree — `main.c:79`, for `HAL_UART_6` (telemetry). RC on UART2 does not
go through the channel layer at all; `rc_task.c:114` drives `hal_uart_init` +
`hal_uart_init_dma_rx` directly. So:

```
slot 0   UART6 telemetry     4112 B   used
slot 1   --                  4112 B   never touched
slot 2   --                  4112 B   never touched
                             ------
                             8224 B   permanently idle
```

That is **20% of the static footprint** doing nothing, on a part where the whole
MSP + slack region is 14.5 KB.

## The sizing rationale is also stale

`CHANNEL_TX_BUF_SIZE` is justified in a comment as covering the worst single-shot
burst between flushes:

> the 1 Hz perf report dumps global + up to 24 task + 16 fifo frames
> back-to-back (~1336 B worst case)

Two of those three inputs have since changed:

- The perf report runs at **5000 ms**, not 1 Hz (`perf_telemetry.c`, changed
  2026-09-08 to buy link budget for the vertical estimator).
- The live counts are **17 tasks and 9 fifos**, not 24 and 16 (`PerfGlobal`
  read off the FC: `total_tasks 17, total_fifos 9`).

So the real worst burst is 27 frames, not 41 — roughly 880 B against a 2048 B
buffer. `channel_tx_overflow_count()` reads **0** on hardware, so nothing is
being stressed.

## Why it is deferred, not fixed

Both changes are one-line, and together they would return **up to ~10 KB** of
`.bss`:

- `MAX_SERIAL_HANDLERS` 3 → 1  frees 8,224 B
- `CHANNEL_TX_BUF_SIZE` 2048 → 1024  frees a further 2,048 B

But neither is free of judgement, and neither is urgent:

- **The spare slots may be deliberate headroom.** A second telemetry link, a
  GPS, or an ESC-telemetry UART would each want one, and `get_handler()` returns
  `ERROR` when it runs out rather than degrading. Dropping to 1 makes the next
  serial device a two-line change instead of a zero-line one. **2** is probably
  the honest number, not 1.
- **The buffer is a burst absorber, and the burst it absorbs is about to
  change.** Re-enabling ControlTrace, restoring the perf report to 1 Hz, or
  raising the IMU ODR (the P0 in
  `../log-analysis/20260909-001748-imu-never-configured/`) all push more frames
  per flush. Shrinking it now and changing the telemetry mix next week is how
  you get an overflow that is hard to attribute.

Right order: fix the ODR, settle the telemetry mix that follows from it, then
re-measure the worst burst against `channel_tx_overflow_count()` and size the
buffer against the number rather than the comment. The unused slots can go
whenever someone wants the 8 KB.

## Checking it

```sh
tools/dev/alloc_budget.py --elf firmware/build-notch/main     # top static objects
```
