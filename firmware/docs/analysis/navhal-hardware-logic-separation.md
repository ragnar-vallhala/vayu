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
| Board facts (`GPIO_PB12`, `HAL_I2C_1`, `TIM1`, AF numbers, register addresses) | No — only `board_*.h` |
| Sensor data | **Yes**, as an SI sample from `hub/` — never as a driver header |
| A cycle stamp or the CPU rate | **Yes**, from `sys/clock.h` — never raw DWT |

The four questions the gate's failure message asks, in order:

- need a cycle stamp or the CPU rate? → `sys/clock.h`
- need sensor data? → take an SI sample from `hub/`, not a driver header
- need a pin, bus, timer or `hal_` call? → it belongs in a driver
- need a board constant? → `board_*.h`, not the consumer

---

## 2. How it is enforced

`tools/dev/check_layering.sh`, in CI and runnable alone
(`check_layering.sh comm` for one section).

Each section carries the count it is **allowed**. The gate fails when a section
goes *above* it, and prints the new number when a section comes in *under*.
**The allowance may only go down.** That is what makes a staged sweep
survivable: a section cleaned this week cannot quietly regress while the next
one is being worked on.

Ledger, 2026-09-28:

| Section | Allowed | Directories |
|---|---:|---|
| core | 0 | `control/ est/ maths/` |
| hub | 0 | `hub/` |
| dsp | 0 | `dsp/ calib/` |
| storage | 3 | `storage/` |
| actuator | 5 | `actuator/` |
| internal | 43 | `sys/ logger/` |
| comm | 44 | `comm/` |

95 lines total, in 16 files. Where they live:

| Section | File | Lines | What it is |
|---|---|---:|---|
| storage | `storage/imu_hs_log.c` | 3 | `hal_cycle_counter_get()` — F9 residue |
| actuator | `actuator/motor.c` | 5 | `TIM1` ×4, pin macros ×4, `driver/esc.h` (F7, F8) |
| internal | `sys/heartbeat.c` | 21 | GPIO, the LED (F5) |
| internal | `sys/timer_callbacks.c` | 7 | timers, IRQ wiring (F12) |
| internal | `sys/sys_utils.c` | 6 | GPIO |
| internal | `sys/clock.c` | 3 | DWT — **legitimate**, this is the seam |
| internal | `sys/boot.c` | 2 | clock verification — legitimate |
| internal | `logger/log_text.c` | 2 | UART |
| internal | `sys/timer_callbacks.h` | 2 | timer types |
| comm | `comm/channel.c` | 26 | UART/DMA/IRQ — the whole transport (F12) |
| comm | `comm/rc_task.c` | 7 | UART |
| comm | `comm/navlink_router.c` | 5 | the LED again (F5) |
| comm | `comm/telemetry_task.c` | 2 | `bme280_read_all` |
| comm | `comm/serializer.c` | 2 | — |
| comm | `comm/comm_processor.c` | 1 | `bmx160_calib_request_cancel` |
| comm | `comm/channel.h` | 1 | — |

Two of those are the seam working correctly, not debt: `sys/clock.c` and
`sys/boot.c` are *where* DWT is allowed to be read. The floor for `internal` is
5, not 0.

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

### 3.1 The invisible include (F1)

`firmware/src/est/attitude_task.c` includes no driver header, and the gate
scores `core` at **0**. It also calls `bmx160_get_board_trim()` at line 210.
The declaration arrives through `variables.h`, which includes `navhal.h` and
`driver/bmx160.h` at lines 24–25.

10 of the 31 files that include `variables.h` are in sections the gate rates
zero:

```
control/angle_controller.c      control/pid_config.c
control/angle_rate_controller.c est/vertical_task.c
est/attitude_task.c             est/sensor_fusion.c
dsp/gyro_notch.c                include/control/control_buffer.h
include/control/angle_controller.h
include/control/angle_rate_controller.h
```

Every "0" in the ledger is conditional on F1 until F1 is split.

### 3.2 Driver calls from outside `driver/`

16 sites, 5 files. A driver called from a logic or service layer is coupling
whether or not the gate's pattern happens to catch the line.

| Caller | Calls |
|---|---|
| `main.c` | `bmx160_init`, `bme280_init`, `vl53l0x_init`, `init_i2c_manager` |
| `actuator/motor.c` | `esc_init` ×4, `esc_arm`, `esc_set_throttle` ×4 |
| `comm/telemetry_task.c` | `bme280_read_all` |
| `comm/comm_processor.c` | `bmx160_calib_request_cancel` |
| `est/attitude_task.c` | `bmx160_get_board_trim` ← the F1 case above |

`main.c` is composition root: wiring drivers there is correct and stays.
`motor.c` is a thin ESC shim; it is the *board facts* in it (F7, F8) that are
the problem, not the calls. The three that are genuinely misplaced are the
last three rows.

---

## 4. Fault-line catalogue

### 4.1 Open

#### F1 🔴🔴 — `variables.h` is a god-header that welds the tree to silicon

`firmware/include/variables.h:24-25` includes `navhal.h` and
`driver/bmx160.h`. 31 files include it, 10 of them in zero-rated sections
(§3.1). It also carries pin macros (`_BLUE_LED_PIN`), `SYS_CLOCK_FREQ`,
filenames and buffer sizes — four unrelated concerns in one header that
everything takes.

**Worst in the catalogue**, and the cheapest to fix: it is two lines in one
file. Until it is split, every section count understates real coupling and the
include-graph gate cannot be written.

**Fix:** drop both includes; let the ~15 files that actually need a HAL type
include it themselves. Split what remains by concern — `board_pins.h`,
`sys/clock.h` (already exists), storage filenames where they are used.

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

#### F5 🔴 — Two owners fight over the BLUE status LED

- `firmware/src/sys/heartbeat.c:34,50,65,74` — sets the mode and toggles it
- `firmware/src/comm/navlink_router.c:63,75,81` — drives it high/low/blinking
  for link state

Both via `_BLUE_LED_PIN` out of `variables.h`. Whichever ran last wins, so the
LED means nothing reliable. This is an **active bug**, not just structure.

**Fix:** one owner. A `status_led_set(state)` with a priority order, or give
the router its own indicator.

#### F7 🟠 — TIM1 is re-initialised per channel with no single owner

`firmware/src/actuator/motor.c:40-43` calls `esc_init(..., TIM1, n, pin)` four
times; `esc_init` configures the timer each time. Four callers, one timer, no
owner — the last call's prescaler wins, and nothing says so.

**Fix:** a PWM group owner that configures TIM1 once and hands out channels.

#### F8 🟠 — STM32 AF pinmux is hardcoded inside the "generic" ESC driver

`firmware/src/driver/esc.c:39-47` picks `HAL_GPIO_AF1` vs `AF2` from the timer
instance, with a comment that says "usually". That is an STM32F4 truth living
in a driver that claims to be portable, and "usually" is doing load-bearing
work.

**Fix:** AF is a board fact — pass it in, or resolve it in `board_*.h`.

#### F12 🟠 — IRQ wiring is split with no ownership registry

`firmware/src/comm/channel.c:127-156` attaches USART and DMA callbacks by hand;
`firmware/src/sys/timer_callbacks.c` wires timers separately. Nothing lists who
owns which vector.

The cost is already documented in the code: `channel.c:71` notes that the SD
block-write path "re-grabs the Stream6 completion IRQ on every SD block write",
which the telemetry path has to work around. Two subsystems claiming one vector
with no registry to notice.

**Fix:** a single table of vector → owner, checked at init.

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

---

## 5. Order of work

Not the obvious order. Cheapest-with-widest-blast-radius first, because each
step makes the next step's measurement honest.

### Step 0 — F1, the god-header

Two lines in one file, 31 dependents. Do this before anything else: it is the
only change that makes every other count *true*, and it is a prerequisite for
ever replacing the textual gate with an include-graph one.

**Accept when:** `variables.h` includes neither `navhal.h` nor a driver header;
`core`, `hub` and `dsp` still read 0 with the transitive path gone; the
`bmx160_get_board_trim` call in `attitude_task.c` is either routed through
`hub/` or made an explicit include that the gate counts.

### Step 1 — finish F9: `vayu_clock_now()`

Repoint the remaining `hal_cycle_counter_get()` consumers at `sys/clock.h`.
3 lines in `storage/imu_hs_log.c`, plus the driver sites (which the rule
allows to stay).

**Accept when:** `storage` ratchets 3 → 0.

### Step 2 — `internal` (43 → 5)

`heartbeat.c` (21) is the bulk and carries F5. Resolving the LED owner
collapses the heartbeat and router cases together.

**Accept when:** `internal` is at 5 — `clock.c` (3) and `boot.c` (2), the
legitimate seam. Lower the allowance to 5 in the same commit.

### Step 3 — `actuator` (5 → 0), F7 + F8

`motor.c` stops naming `TIM1` and pins; a PWM group owner takes them.
AF selection leaves `esc.c`.

### Step 4 — `comm` (44 → ?), F12

Largest and least urgent: `channel.c` is transport, and transport touching
UART/DMA is closer to being a driver than a violation. The valuable part here
is the IRQ ownership registry, not the count. Decide `comm`'s floor before
starting — it is probably not 0.

The three misplaced driver calls (§3.2) fall out of steps 2–4: `bme280_read_all`
and `bmx160_calib_request_cancel` should take their data from `hub/` or go
through a service, not reach for the device.

---

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

- **A BSP.** Tempting, and it does dissolve F1/F7/F8 — but it is a large
  mechanism bought up front for problems that a header split and a PWM owner
  close individually. Revisit if a second board appears.
- **Porting `driver/` to anything.** Drivers name silicon. That is the job.
- **Driving `comm` to 0.** See step 4.
- **Replacing the textual gate with an include-graph one.** Possible only
  after F1, and worth doing only if textual starts missing things.
