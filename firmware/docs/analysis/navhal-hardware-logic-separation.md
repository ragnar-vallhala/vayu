# Hardware/logic decoupling — the guide

**What this is.** The working guide for getting silicon out of vayu's logic
layers: the rule, the gate that enforces it, what is left, and the order to do
it in. It replaces the June 2026 audit of the same name, and folds in
`docs/plans/hardware-abstraction-sweep.md`, which was a second document about
this one problem.

Fault lines keep their original **F-numbers** — commits and notes reference
them — but every path, count and state below was re-measured on 2026-09-28
against `main` at `71c23a2`. Where the audit's evidence no longer matches the
tree, the guide says what closed it rather than preserving the old line
numbers.

---

## 1. The rule

> A translation unit under `control/`, `est/`, `maths/`, `hub/`, `dsp/` or
> `calib/` may not name a pin, a bus, a peripheral instance, a timer, a
> register, or a NavHAL function — and may not include a driver header.
> Those layers take SI quantities and `dt`. A hardware change must not
> recompile them, and they must stay host-testable with no HAL present.

`storage/`, `actuator/`, `sys/`, `logger/` and `comm/` are held to the same
rule; they are simply not there yet, and carry a budget (§2).

`driver/` is exempt. Silicon is what a driver is *for*.

### 1.1 Controlled spill — what may cross, and how

Time is an OS concept and may spill. Pins, buses, chips, timers and registers
are silicon and may not.

| Symbol class | Allowed in the logic layers? |
|---|---|
| vaios portable service (`v_malloc`, `v_get_ticks`, semaphores) | **Yes** — via a vayu-owned `sys/*.h` re-export |
| vaios *port* layer | No |
| NavHAL capability API (`hal_*`, `HAL_*`) | No — only `src/driver/**`, `src/port/**` and `board/` |
| Board facts (`GPIO_PB12`, `HAL_I2C_1`, `TIM1`, AF numbers, register addresses) | No — only the selected `board/<name>/vayu_board.h` |
| Sensor data | **Yes**, as an SI sample from `hub/` — never as a driver header |
| A cycle stamp or the CPU rate | **Yes**, from `sys/clock.h` — never raw DWT |

The four questions the gate's failure message asks, in order:

- need a cycle stamp or the CPU rate? → `sys/clock.h`
- need sensor data? → take an SI sample from `hub/`, not a driver header
- need a pin, bus, timer or `hal_` call? → it belongs in a driver
- need a board constant? → `#include "vayu_board.h"`, not the consumer

---

## 2. How it is enforced

`tools/dev/check_layering.sh`, in CI and runnable alone
(`check_layering.sh comm` for one section).

Each section carries the count it is **allowed**. The gate fails when a section
goes *above* it, and prints the new number when a section comes in *under*.
**The allowance may only go down.** That is what makes a staged sweep
survivable: a section cleaned this week cannot quietly regress while the next
one is being worked on.

**The sweep is finished.** Every section below is at its **floor**, so these
numbers are no longer a backlog — they are what each section legitimately
names because that is its job. A floor driven to zero does not remove the
hardware dependency, it hides it behind one more indirection and makes the
gate report clean while the coupling is still there. A number going *up* is a
new dependency; a number going *down* is progress only if a dependency
actually went away.

Ledger, 2026-09-29:

| Section | Floor | Directories |
|---|---:|---|
| core | 0 | `control/ est/ maths/` |
| hub | 0 | `hub/` |
| sensor | 0 | `sensor/` |
| dsp | 0 | `dsp/ calib/` |
| storage | 0 | `storage/` |
| actuator | 1 | `actuator/` |
| internal | 7 | `sys/ logger/` |
| comm | 4 | `comm/` |

12 lines total, in 9 files. Where they live:

| Section | File | Lines | What it is |
|---|---|---:|---|
| actuator | `actuator/motor.c` | 1 | `driver/esc.h` — **its floor**; the pins, timer and AF are board macros |
| internal | `sys/clock.c` | 3 | DWT — **legitimate**, this is the seam |
| internal | `sys/boot.c` | 2 | clock verification — legitimate |
| internal | `sys/heartbeat.c` | 1 | `driver/indicator.h` — it is the annunciator's policy |
| internal | `sys/sys_utils.c` | 1 | `driver/crc.h` |
| comm | `comm/channel.c` | 1 | `driver/uart.h` — **its floor** |
| comm | `comm/channel.h` | 1 | `driver/uart.h`, for `vuart_t` |
| comm | `comm/rc_task.c` | 1 | `driver/uart.h` |
| comm | `comm/serializer.c` | 1 | `driver/uart.h` |

None of those is debt. `sys/clock.c` and `sys/boot.c` are *where* the CPU rate
and the cycle counter are allowed to be read — that seam is the reason
`control/`, `est/` and `maths/` never have to name them. `heartbeat.c` and
`sys_utils.c` name a driver because they genuinely drive one, the same way
`motor.c` does. The floor for `internal` is **7**, not 0; `logger/` is already
at 0 and contributes none of it. A section reaching its floor is the end of
the work, not a failure to finish it.

### 2.1 Why the gate is textual, not an include graph

Because an include-graph rule would fail every file and therefore gate nothing:
`variables.h` pulls `navhal.h` into 31 translation units (F1). Textual is what
is enforceable *today*, and it is what stopped the growth — between the June
audit and September the raw-DWT call sites went 4 → 7 and `SYS_CLOCK_FREQ`
gained consumers, because nothing was watching.

Driver *headers* are in the pattern for the same reason: `driver/bmx160.h`
includes `navhal.h`, so a TU that names a driver gets the whole register map
without ever typing `hal_`. An umbrella header once made this invisible — the
check reported clean while `control/` transitively included `navhal.h` through
`driver/driver.h`. **Naming a driver is itself the violation.**

---

## 3. What the gate cannot see

The ledger understates the real coupling in two ways. Both matter for the
order of work.

### 3.1 The invisible include (F1) — closed 2026-09-28

This section used to say that `est/attitude_task.c` includes no driver header,
scores `core` at 0, and calls `bmx160_get_board_trim()` anyway — the
declaration arriving through `variables.h`.

That is fixed. `variables.h` now includes **nothing at all**, and the board
trim comes from `hub/` like every other SI quantity. The header still has 31
includers and still mixes four concerns, so F1 is not fully closed (§4.1), but
it no longer carries silicon.

What the compiler's own dependency files say, before and after:

| Translation unit | NavHAL headers before | after |
|---|---:|---:|
| `control/angle_controller.c` | 77 | 2 |
| `control/angle_rate_controller.c` | 77 | 2 |
| `est/attitude_task.c` | 2 | 2 |
| `est/vertical_task.c` | 2 | 2 |
| every other TU in `control/ est/ dsp/ hub/` | 0 | 0 |

`driver/bmx160.h` no longer reaches any of them.

The 2 that remain are `common/hal_types.h` and `common/navhal_compiler.h` —
types and compiler attributes, no function declarations, arriving through
vaios. That is the controlled spill of §1.1 working as intended, not debt.

The 77 were the whole HAL, and they came through an umbrella: `comm/comm.h`
includes `comm/channel.h`, which includes `navhal.h`. Two control files wanted
`rc_buffer.h` and got the entire peripheral API. They now include what they
use. **An umbrella header is how this damage always arrives** — it was
`driver/driver.h` the first time, `variables.h` the second, `comm/comm.h` the
third.

### 3.2 Driver calls from outside `driver/`

12 sites, 4 files. A driver called from a logic or service layer is coupling
whether or not the gate's pattern happens to catch the line.

| Caller | Calls | Verdict |
|---|---|---|
| `main.c` | `bmx160_init`, `bme280_init`, `vl53l0x_init`, `init_i2c_manager` | composition root — correct, stays |
| `actuator/motor.c` | `esc_init` ×4, `esc_arm`, `esc_set_throttle` ×4 | a thin ESC shim; its board facts are now macros, the calls are its job |
| `comm/telemetry_task.c` | `bme280_read_all` | **fixed 2026-09-28** — goes through the barometer model |

`est/attitude_task.c` was the fifth and is gone (§3.1).

## 4. Fault-line catalogue

### 4.1 Open

#### F1 🟠 — `variables.h` is still a god-header (no longer a silicon one)

**The silicon weld is gone** (2026-09-28). `variables.h` includes nothing; the
measured effect is in §3.1. What closed it:

- board facts (LEDs, buzzer, I2C bus/pins/DR, IMU address, ESC timer and pins)
  moved to `board/<name>/vayu_board.h`, selected by `VAYU_BOARD`
- IMU tuning (ODRs, ranges, bandwidths) moved to `include/driver/bmx160.h`,
  `#ifndef`-guarded — it is tuning, not a board fact
- `extern channel_t g_telemetry_channel` moved to `comm/channel.h`, where
  `channel_t` is declared, which removed the third silicon include
- the board trim moved from a driver getter to `hub_get_board_trim()`
- `I2C_MODE` and `BMX_MAG_ODR` were dead and were deleted

**What remains:** 31 includers, and four unrelated concerns in one header —
control tunables, loop rates, storage filenames and buffer sizes. Changing a
PID default still recompiles the comm layer. That is a build-time cost and a
readability one, not a portability one, which is why it drops from 🔴🔴 to 🟠.

**Fix:** split by concern — `control/tuning.h`, `storage/paths.h`, and leave
the rates. Not urgent; nothing depends on it now.

#### F2b 🟠 — `i2c_config` is triple-aliased

- `firmware/src/main.c:214` — the definition, external linkage
- `firmware/src/driver/i2c_manager.c:27` — a **different** object, `static`,
  same name
- `firmware/src/driver/bmx160.c:99` — `extern`, binds to main.c's

The IMU driver reaches past the manager to the raw global and re-inits the bus
with it during recovery (`bmx160.c:1051`). One name, two objects, three
owners: a linkage footgun that reads as a single variable.

**Fix:** rename the static to `s_cfg`; give `i2c_manager` an accessor for the
recovery path so `bmx160.c` stops needing the `extern`. This is the last live
piece of F2.

### 4.2 Closed

| # | Was | What closed it |
|---|---|---|
| F2 | I2C bus owned by the IMU driver | `bmx160.c` no longer calls `hal_i2c_*` at all; it goes through `i2c_manager`. Residue is F2b |
| F3 | IMU driver read the barometer | zero `bme280`/`baro` references remain in `bmx160.c` |
| F4 | Telemetry byte-banged the UART | `channel.c` DMAs UART6 |
| F6 | No hardware motor-kill | closed by decision — `esc_disarm` is a `@noreq` primitive; disarm is ACT-FAIL-001's zero-PWM-in-one-iteration. 0 call sites outside the driver is correct, not a gap |
| F9 | Raw DWT in 4 modules | `sys/clock.h` is the seam. `clock.c`/`boot.c` read DWT legitimately; `storage/imu_hs_log.c` ×3 is the last consumer that should be repointed |
| F10 | `SYS_CLOCK_FREQ` duplicated vs the PLL | every consumer needing a real interval divides the rate measured at boot (`vayu_clock_hz()`). The macro survives only as the value `boot.c` checks against, and as the pre-boot fallback |
| F11 | Logic consumed `bmx160_all_reading_t` | the hub migration; 0 references remain in `control/` or `est/` |
| F13 | ESC band duplicated in the SITL host | one band in `actuator.h`, both sides derive |
| F12 | IRQ wiring split, no registry | `sys/irq_registry.h`. Every vayu claim — the two UART RX vectors, both TX-DMA vectors, the HF timer, the RC idle callback — records an owner first, and a second owner is logged rather than discovered later by the peripheral it evicted. Vector numbers moved to the board. **Gap:** vaios claims vectors too, behind its own build flags, and mirroring another repo's config macros here is the duplication F13 was about — so a vayu-vs-vaios collision still needs reasoning about by hand |
| F7 | TIM1 re-inited per channel, no owner | `esc_group_init(timer, freq)` is the only caller of `hal_pwm_init`, so PSC/ARR are written once. `esc_init` sets up its channel with `hal_pwm_set_duty_cycle`, which touches only that channel's CCR. Arm and disarm became channel scoped: `esc_disarm` used to stop the shared timer, which would have cut PWM to all four motors |
| F8 | AF pinmux hardcoded in the ESC driver | `BOARD_ESC_AF`; `esc.c` names no timer instance at all now |
| F5 | two owners of the BLUE LED | `driver/indicator.c` owns the four pins and is the only writer. `heartbeat.c` renders flight state; the router calls `heartbeat_note_link_activity()` instead of driving a pin on its own timer, and the activity blink is an explicit override with a defined precedence |

---

## 5. Order of work

Not the obvious order. Cheapest-with-widest-blast-radius first, because each
step makes the next step's measurement honest.

### Step 0 — F1, the god-header ✅ done 2026-09-28

The silicon path is closed and the board layer exists. See §4.1 for what
landed and what is left. `core`, `hub` and `dsp` now read 0 with the
transitive path genuinely gone, which the dependency files confirm (§3.1) —
before this, that 0 was conditional.

### Step 1 — finish F9: the clock seam ✅ done 2026-09-28

`storage/imu_hs_log.c`'s three `hal_cycle_counter_get()` calls now go through
`vayu_clock_cycles()`. `storage` ratcheted 3 → 0.

These three were the only thing still proving F1 mattered: once `variables.h`
stopped including `navhal.h`, they became implicit declarations and the build
said so. A layering violation that the compiler cannot see is the dangerous
kind — removing the umbrella is what made it visible.

### Step 1b — `actuator` 5 → 1, partial F7/F8 ✅ done 2026-09-28

`motor.c` no longer names `TIM1` or a pin; it uses the board macros. The
remaining 1 is `#include "driver/esc.h"`, which is honest — `motor.c` is the
ESC shim. F7 (nobody owns the timer) and F8 (AF hardcoded in `esc.c`) are
untouched.

### Step 2 — `internal` 43 → 7, and F5 ✅ done 2026-09-28

Four moves, and three of them were the same realisation: code in `sys/` that
drives a peripheral is a driver wearing the wrong label.

- **`driver/indicator.c`** now owns the three LEDs and the buzzer, and is the
  only writer. That is F5 (§4.2). `heartbeat.c` kept the policy — what each
  flight state looks like — and lost all 21 of its silicon lines.
- **`driver/timer_callbacks.c`** (was `sys/timer_callbacks.c`) owns the
  high-frequency timer and its ISR. It was always a driver; it moved, and
  `TIM5` became `BOARD_HF_TIMER`. That took 9 lines out of the section by
  putting them where the rule already permits them.
- **`driver/crc.c`** holds the CRC32 peripheral. `sys_utils.c` keeps the mutex
  that makes the shared accumulator safe, and its two near-identical entry
  points collapsed into one body with different patience.
- `logger/log_text.c` joined the clock seam.

`comm` fell 44 → 39 on the way, because the router stopped driving a pin and
stopped reading DWT.

`sys/sys.h` was an umbrella header with **zero** includers — deleted. That is
how the 77-header leak into `control/` got in (§3.1), and a dead one is just a
loaded gun for the next person.

### Step 3 — F7 + F8 ✅ done 2026-09-28

Not a ratchet: `actuator` stays at **1** and that is its floor — the one hit is
`motor.c` including `driver/esc.h`, which is what `motor.c` is for. An earlier
draft of this plan said 1 → 0, which was the same mistake as "drain `internal`
to zero": a section that has reached its floor is finished, and driving the
number below it would mean hiding a dependency rather than removing one.

What actually changed is ownership:

- **`esc_group_init(timer, freq)`** is now the only caller of `hal_pwm_init`,
  and therefore the only thing that writes the timer's prescaler and
  auto-reload. It also starts the timer, once. `esc_init` no longer takes a
  timer at all; it sets its channel up with `hal_pwm_set_duty_cycle`, which
  reads the group's ARR and writes only that channel's CCR, mode, preload and
  output enable — exactly what `hal_pwm_init` did per channel, minus the
  timer-wide re-init a single channel had no business doing.
- **Arm and disarm are channel scoped.** They used to be `hal_pwm_start` and
  `hal_pwm_stop`, and `hal_pwm_stop` stops the timer — so disarming one motor
  would have cut the PWM to all four. Nothing called `esc_disarm` (F6), which
  is the only reason that never fired.
- **`BOARD_ESC_AF`** replaces the `if (timer == TIM1 || timer == TIM2)` ladder
  whose comment said "usually". `esc.c` names no timer instance now.

The host's PWM stub had to become honest before any of this was testable: it
modelled `hal_pwm_stop` as channel-local, so the shared-timer hazard was
invisible and a mutant that reintroduced it passed. It now tracks the timer's
CEN separately from each channel's output enable, as the silicon does.

### Step 4 — `comm` 39 → 4, and F12 ✅ done 2026-09-28

**F12 is fixed.** `sys/irq_registry.h` is a flat table of vector → owner, and
every claim vayu makes goes through it first: both UART RX vectors, both
TX-DMA vectors, the HF timer, the RC idle callback. A second owner is a log
line at init instead of a peripheral that stops working months later. The
vector numbers moved to the board, where they belong.

That matters because the collision it is named for was expensive: the
telemetry UART's default TX DMA stream is shared with SDIO on this part, SDIO
re-grabbed the completion IRQ on every card write, and telemetry died after
the first save — saving a calibration was enough. The board now picks the
alternate stream, and the registry would have said so at boot.

Also done: the UART instances and their vectors became board macros, `rc_task`
joined the clock seam (which finishes F9's repointing outside `driver/`), and
a dead `driver/bmx160.h` include came out of `telemetry_task.c`.

**The UART driver landed too.** `driver/uart.h` owns the peripheral and both
its interrupt vectors; `comm/` asks for a link and gets one. Every vector claim
goes through the registry, so F12 covers the transport as well.

`comm`'s floor is **4**: one `#include "driver/uart.h"` each in `channel.c`,
`channel.h`, `rc_task.c` and `serializer.c`. Driving the UART driver is what
the transport is for — the same floor `motor.c` has against `driver/esc.h`.

Two things fell out that were not the point but were worth having. The vector
ternary (`uart == TELEMETRY ? USART6_IRQn : USART2_IRQn`) appeared twice and
is now `_rx_vector()` once, which is why `get_handler` shrank by 52 bytes. And
`hal_interrupt_disable_global()` guarding the handler list was the HAL standing
in for a scheduler primitive — it is `v_enter_critical_from_isr()` now, which
has exactly the save-and-restore-a-local semantics that call had.

**Both driver includes are gone, and the answer was a third thing.**

`telemetry_task.c` → `bme280_read_all` looked like a choice between routing
through `hub/` and leaving it alone. The hub's `baro_sample_t` carries
pressure and temperature; the BARO telemetry message also sends humidity and a
derived altitude. Widening the SI sample to hold them would put a
chip-specific field in the type the control core reads — F11 again — and
routing through the hub as it stood would have silently dropped fields from a
wire message.

The split that works is by PURPOSE, not by field: measurements the flight code
needs go through the hub, and a reporting-only readout is asked for through
the device model (`sensor/baro.h`). A barometer with no humidity sensor
reports 0 **and says so**, which is a different statement from "0% relative
humidity".
- `comm_processor.c` → `bmx160_calib_request_cancel`: **fixed 2026-09-28.**
  It goes through the IMU model now (`sensor/imu.h`), so the command layer
  asks "an IMU" to calibrate itself instead of naming one. It used to allocate
  the calibration task's arguments, create that task with a stack depth it had
  measured itself, track the handle, and cancel through a chip-specific
  function — the command layer knew the stack depth of a routine inside the
  IMU driver. All of that moved to the driver, where it is a property of that
  driver's routine rather than of "an IMU". The calibration engine still lives
  in `bmx160.c`, but nothing outside the driver can tell.

## 6. Doing a step

1. `./tools/dev/check_layering.sh <section>` — see the current number.
2. Make the change.
3. Re-run. The gate prints `ratchet: lower it to N`.
4. **Lower the allowance in `check_layering.sh` in the same commit as the
   work.** A step that does not ratchet has not landed.
5. Build both trees — the firmware and the host tests. The host build is the
   real proof: a layer that still needs a HAL will not link without one.

---

## 7. Not in scope

- **A full BSP.** A minimal board layer landed with step 0 —
  `firmware/board/<name>/vayu_board.h` plus a `VAYU_BOARD` selection, modelled
  on NavHAL's own `src/board/<name>/board.h` so a board can graduate into
  NavHAL by `git mv` plus a Kconfig entry. That is deliberately as far as it
  goes: a macro list and a selector, no init hooks, no device tables, no
  per-board driver registration. See `firmware/board/README.md`. The roadmap
  (navixdev → navixsm_f401re → f441 → h767) crosses an MCU family, which is
  what the layer is sized for; anything more is bought when a board needs it.
- **Porting `driver/` to anything.** Drivers name silicon. That is the job.
- **Driving `comm` to 0.** See step 4.
- **Replacing the textual gate with an include-graph one.** Possible only
  after F1, and worth doing only if textual starts missing things.
