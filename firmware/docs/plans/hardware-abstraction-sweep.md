# Hardware-abstraction sweep

Getting silicon out of every layer that is not a driver, one section at a time,
so that vaios' MPU work has a boundary to enforce.

Under an MPU an unprivileged task cannot touch a register at all. Everything
that reaches `navhal.h` has to sit on the privileged side or behind a syscall,
so "which code names hardware" stops being a matter of taste and becomes the
thing that decides what can run unprivileged. That is the reason for doing this
now rather than continuously.

## The rule

No translation unit outside `driver/` may name a pin, a peripheral instance, a
timer, a register or a NavHAL function, nor include a header that does so on its
behalf. Those layers take SI quantities and `dt`; a hardware change must not
recompile them.

Time is the one spill: it is an OS concept, not silicon. Logic gets cycle stamps
from `sys/clock.h`, never from the DWT directly.

`driver/` is exempt. Silicon is what a driver is for.

## Measured baseline

`tools/dev/check_layering.sh` counts lines naming silicon, per section, and
fails a section that goes **above** its allowance. The allowance may only fall.
As of the sweep's start:

| Section | Count | Directories |
| --- | ---: | --- |
| `core` | 0 | `control/ est/ maths/` |
| `hub` | 0 | `hub/` |
| `dsp` | 0 | `dsp/ calib/` |
| `storage` | 3 | `storage/` |
| `actuator` | 5 | `actuator/` |
| `internal` | 43 | `sys/ logger/` |
| `comm` | 44 | `comm/` |

`core`, `hub` and `dsp` are done and gated at zero. The other 95 are the work.

## What the counts actually are

Breaking each section down by what it names, because the shape decides the fix:

| Section | Dominated by | Fix |
| --- | --- | --- |
| `storage` | 3 × `hal_cycle_counter_get` | the clock seam, nothing else |
| `actuator` | `TIM1`, `GPIO_PA08..11`, one `driver/` include | board wiring belongs in a board header or in `driver/esc.c` |
| `internal` | 48 × GPIO (`HAL_GPIO`, `hal_gpio_write`, `hal_gpio_set_mode`) — LEDs and buzzer; 7 × `navhal.h`; 6 × timer | an LED/indicator driver, then a timer driver |
| `comm` | 26 × UART (`HAL_UART` + the calls); 5 × GPIO (the blue LED); 5 × `navhal.h`; 3 × `driver/` | a UART driver — `channel.c` **is** the driver today |

Two thirds of `comm` is one peripheral. Most of `internal` is LEDs.

## Order, and why it is not the obvious one

**1. `sys/clock.h` gains `vayu_clock_now()`.** Not a section — a prerequisite.
`hal_cycle_counter_get` appears in four sections; the seam already exists
(`vayu_clock_hz`, `vayu_dt_from_cycles`) but has no "give me a stamp" call, so
everyone bypasses it. One function closes `storage` entirely and shrinks three
others before they are started.

**2. `internal`.** Largest after the clock, and an LED driver is
self-contained: no callers outside `sys/` and `comm/`, no timing risk.

**3. `comm`.** The biggest single job. UART has no driver at all — `channel.c`
does init, DMA, interrupt attach and `hal_interrupt_disable_global`. That last
one is the MPU-relevant item: masking interrupts globally is exactly what an
unprivileged task must never do.

**4. `actuator`.** Small, but it is the motor path, so it is verified on
hardware rather than by the gate alone.

**5. `storage`, `dsp`, `hub`, `core`.** Verification passes. If any of these
turns into real work, something regressed and that is worth knowing.

`sensors` is absent on purpose: `driver/` is where its silicon belongs, and the
`i2c_manager` → bus-manager move is gated on vaios' `periph_bus`, not on this.

## Acceptance, per section

A section is done when **all** of these hold:

1. `check_layering.sh <section>` reports `0`.
2. The allowance in `check_layering.sh` is lowered to `0` **in the same commit**
   as the work. The gate never lowers it itself.
3. `firmware` and the SITL both build clean, and `ctest` is green.
4. The SITL determinism fingerprint is unchanged — noting it is bimodal and
   flakes at roughly 30% (see the todo ledger), so a single mismatch is not
   evidence of a behaviour change. Re-run before concluding anything.
5. Anything on the flight path is flashed and bench-checked before the section
   is called done. `actuator` and `comm` both qualify.

## Verifying

```
tools/dev/check_layering.sh            # every section
tools/dev/check_layering.sh comm       # while working in one
```

Under its allowance, the gate passes and prints the number to lower it to. Over,
it fails and names the lines. CI runs it bare, which covers every section.
