# NavHAL cross-coupling in vayu — hardware/logic separation audit

> **Archived 2026-09-10.** Written 2026-06-19 on the (now deleted) branch it
> names, and kept for the reasoning, not as a current map: every path predates the
> monorepo restructure — `src/` is now `firmware/src/`, `software/` is `navigator/`,
> `tools/sim_host/` is `sim/host/`.

> Branch `analysis/navhal-coupling`, cut from `main@1e58df2`. Pure analysis — no
> firmware behaviour is changed here. Goal: find the real fault lines where vayu's
> logic is welded to silicon, and decide the separation mechanism (BSP vs.
> alternatives). This is a **call-level** audit (who calls/owns what), not an
> include-graph audit — every finding below is backed by `file:line` evidence.
>
> Target hardware: the **navixsmf401 v0.0.4** flight-controller board — a custom
> PCB carrying the same STM32F401 with a fixed, board-defined pin map. The build's
> `BOARD nucleo_f401re` (`CMakeLists.txt`) is just the matching MCU/pin target and
> is fine as-is. Because the board layout is fixed, the BSP below is a single
> concrete board descriptor, not a multi-board abstraction — the value is one
> source of truth for the fixed pins/timers/DMA, not runtime portability.

## 0. The question

> "Keep a proper hardware/logic separation so a future hardware change (MCU /
> pin / sensor) doesn't create havoc. Vaios' non-port *logical* things may spill
> through in a controlled manner, but the hardware-dependent ones must not."

Two axes are currently tangled and must be separated:

- **Hardware-dependent** (must be quarantined behind one seam): pin maps,
  peripheral instance IDs (`HAL_I2C_1`, `HAL_UART_6`, `TIM1`), DMA stream/channel
  IDs, register addresses, AF numbers, MCU clock facts, the DWT.
- **Logical** (may spill, in a controlled way): vaios' OS services that are
  portable by construction — ticks (`v_get_ticks`), tasks/scheduling
  (`task_delay_until`), IPC, memory. Free of silicon, so a *controlled*
  dependency is acceptable.

## 1. Headline verdict

The **dependency layering is structurally sound** (vayu → vaios → NavHAL; NavHAL
is already a capability HAL you own; vaios already has a clean `portable/cortex-m4`
port that does *not* leak into logic). The control/estimation **math** is genuinely
pure — `pid.c`, `ekf.c`, `lpf.c`, and the Mahony/complementary filter bodies make
zero hardware calls and take `dt`+samples as parameters.

But **everything between the math and the silicon is coupled by convention, not by
a seam.** There is:

- **No board seam.** No `board.h` / `bsp_init()`. `main()` is the de-facto BSP, and
  pin/timer/DMA/register facts are *additionally* scattered across `variables.h`
  and a dozen driver/task bodies — and **duplicated again in the SITL host HAL**.
- **No single owner for shared peripherals.** The I2C bus is really owned by the
  *IMU driver*, not `i2c_manager`. A UART and a status LED each have **two**
  owners that actively fight.
- **No timebase seam.** The DWT cycle counter is read raw in 4 modules, and the
  clock frequency exists as two independent constants that can silently disagree.

The result is exactly the "havoc on hardware change" you want to prevent: an MCU,
pin, UART, timer, DMA-stream, or sensor change forces coordinated edits across 3–5
unrelated translation units that **the compiler cannot keep consistent** — and
three of these tangles are *active bugs today*, not just future risk.

## 2. The dependency stack (context)

```
vayu  (control, est, sensor-drivers, comm, sys glue)
  └─ extern/vaios            RTOS: kernel + IPC + VFS + timing (v_get_ticks/v_delay = SysTick)
       ├─ portable/cortex-m4 vaios PORT layer   ← clean; does NOT leak into vayu logic
       └─ extern/NavHAL       capability HAL: hal_{gpio,uart,i2c,spi,pwm,timer,dwt,dma,clock}.h
```

NavHAL is authored under NAVRobotec (yours) — the abstraction seam is yours to
move, not a vendor SDK to wrap blindly. That matters for §6.

## 3. Fault-line catalogue (worst first, with evidence)

Severity = blast radius on a hardware change × whether it's wrong *today*.
**🔴 ACTIVE** = a real defect in the current build. **🟠 LATENT** = correct today
only by coincidence; breaks on the next change.

### F1 🔴🔴 — `variables.h` is a god-header that welds the whole tree to silicon
`include/variables.h:8-9` opens with `#include "navhal.h"` + `#include
"sensor/bmx160.h"`, then interleaves three unrelated concerns in one file that is
included **26×**, including by every control/est TU:

- pure tuning constants: PID gains, `SF_COMPLEMENTARY_ALPHA`, loop rates, the
  `vayu_dt_from_cycles()` inline;
- board pin map: `_BLUE_LED_PIN GPIO_PB12`, `_BUZZER_PIN GPIO_PA05`,
  `I2C_PIN_1 GPIO_PB08` (`variables.h:33-47`);
- raw silicon: `SYS_CLOCK_FREQ 84000000` (`:12`), `I2C_BUS HAL_I2C_1`,
  `I2C_DR_REG_ADDR (uint32_t)(0x40005400 + 0x10)` — **a literal STM32 I2C1 data
  register address** (`:47`), and BMX160 driver enums (`:51-62`).

So `pid.c` and `ekf.c` transitively `#include` the full HAL register map and the
IMU driver header **just to read a gain**. A sensor or MCU change recompiles the
control math. This single file is the largest cause of "hardware change → havoc":
the blast radius of a pin or peripheral edit is the entire firmware, by
construction. *(Confirmed: control/est call-level is clean; the coupling is
entirely this transitive include backbone.)*

### F2 🔴🔴 — The I2C bus is owned by the IMU driver, not `i2c_manager`
`i2c_manager` is structurally a thin single-transaction executor
(`i2c_manager.c:14-16` — one `static i2c_async_t _current_trans`, one shared
`_rx_data[]` buffer; `:195` hard-codes `hal_i2c_read_regs_dma(HAL_I2C_1, …)`). The
*actual* bus owner — the "who's next on the bus" state machine, decimation,
recovery, and DMA-completion fan-out — lives in `bmx160.c`:

- bus arbitration state machine `imu_op_t {FAST,MAG,TEMP,BARO}` + `_next_op`
  is private to the IMU driver (`bmx160.c:72-85`);
- `bmx160_initiate_read` (`bmx160.c:859-940`) is the **only** thing that pumps the
  bus at runtime;
- the IMU driver even **re-initialises the whole bus** on recovery —
  `init_i2c_manager(&i2c_config)` + `i2c_manager_unstick()` (`bmx160.c:906-936`).

**Havoc:** swap or remove the IMU and the entire bus dies — nothing else pumps it.
There is no bus owner that survives changing the IMU. *(See F2b for the related
`i2c_config` aliasing footgun.)*

### F3 🔴🔴 — Cross-driver coupling: the IMU driver reads the barometer
The baro's entire runtime read path is hard-wired inside the IMU driver:
`bmx160.c:3` `#include "sensor/bme280.h"`; `bmx160.c:896-902` issues
`i2c_manager_read_async(BME280_I2C_ADDR, BME280_REG_DATA, BME280_DATA_LEN, …)`;
the IMU's TEMP ISR decides *when* the baro is sampled
(`bmx160.c:979-994`, gated by `BARO_READ_DECIM 10` — the baro's sample rate is a
constant inside the IMU driver); the IMU ISR hands raw bytes to the baro via
`bme280_ingest_raw()` (`bmx160.c:1006-1011`).

The single-owner-DMA invariant (from memory: "the baro must ride the IMU's loop")
*is* upheld at runtime — but it's upheld by **welding the baro into the IMU's
source file** instead of living in the bus owner. Swap the baro (BME280 → DPS310:
different addr/reg/len/cadence) and you edit **bmx160.c**, not just a baro driver.

### F4 🔴 ACTIVE — Telemetry TX byte-bangs the UART because channel.c is hardcoded to the wrong peripheral
Telemetry is created on `HAL_UART_6` (`main.c:72`), but every fast-path branch in
`channel.c` is hardcoded to `HAL_UART_2` / `DMA1_Stream6`:
`_dma_complete_callback` only clears `busy` for `== HAL_UART_2` (`channel.c:35`);
the DMA-IRQ attach is `if (uart == HAL_UART_2)` (`:110`); and `flush_channel`'s DMA
write is `if (uart == HAL_UART_2) hal_uart_write_dma(...)` else **"Other UARTs
(currently blocking)"** byte-bang `hal_uart_write_char` loop (`:196-208`).

**Active defect:** the real telemetry path (UART6) takes the `else` branch and
**busy-waits the flush_task byte-by-byte at 230400 baud** (~22 ms for a 512 B
flush). The entire ping-pong/`busy`/DMA-complete machinery is dead code for the
peripheral it's supposed to serve. The telemetry transport identity is also
duplicated and must be kept in sync by hand across `main.c:72`,
`serializer.c:40`, and `channel.c`.

### F5 🔴 ACTIVE — Two owners fight over the BLUE status LED
`heartbeat.c` owns the LEDs/buzzer as a state indicator (sets pin modes `:32`,
toggles/writes `_BLUE_LED_PIN` at `:46,53,78-80,111,146`, with a cached
`_blue_led_state` at `:11`). But `navlink_router.c` — pure protocol routing —
*also* drives the same pin directly for a "command received" blink:
`hal_gpio_write(_BLUE_LED_PIN, …)` at `navlink_router.c:41,52,58`.

**Active defect:** two subsystems write PB12 with **separate, unsynchronised shadow
state**. The router writes behind heartbeat's back, desyncing heartbeat's cached
`_blue_led_state`, so its next toggle produces the wrong level; the state-transition
clear at `heartbeat.c:111` can stomp the router's blink mid-window. Protocol code
also can't be host-tested without a GPIO HAL.

### F6 🔴 — No true hardware motor-kill; disarm is a logic-side zeroing only
`esc_disarm()` (which calls `hal_pwm_stop`, `esc.c:51-54`) is **defined but never
called** (grep: declaration + definition only, zero call sites). Disarm is done
purely in logic by writing `motor_outputs.m* = 0` (`motor.c:52-57`), which
`esc_set_throttle` maps to the **minimum pulse (~1 ms), not a stopped signal**. The
only thing between a fault and spinning props is a `system_state_get()` check
*inside the same task that drives the ESCs* — there is no independent hardware
disable path even though the HAL supports one.

### F7 🟠 — One hardware timer (TIM1) is re-initialised per-channel with no single owner
The motor map is hand-unrolled inline in the actuator module:
`motor.c:23-26` `esc_init(&motors[i], TIM1, ch, GPIO_PA08…PA11)`. Each `esc_init`
calls `hal_pwm_init` against the **same** TIM1 (`esc.c:43`), so the timer base
(PSC/ARR) is redundantly driven four times, and each channel keeps its **own**
`esc->frequency` copy (`actuator.h:30`). If two channels ever hold different
frequencies the last init silently re-periods channels already configured, and
their duty math (`esc.c:69`) then emits wrong pulse widths. No "one owner sets the
timer base, channels attach" structure.

### F8 🟠 — STM32 pinmux AF truth is hardcoded inside the "generic" ESC driver
`esc.c:31-37` branches `if (timer == TIM1 || TIM2) AF1 else if (TIM3/4/5) AF2`.
AF routing is a per-**pin** property on STM32, not per-timer; the comment even
hedges "usually". A timer outside TIM1–TIM5 (TIM9/10/11 exist on F401) sets the pin
to AF mode (`:28`) with **no AF selected** → dead output, no error. Silicon pinmux
belongs in a board table keyed by pin, not in the ESC abstraction.

### F9 🟠 — The DWT cycle counter is read raw in 4 modules with no timebase seam
`hal_cycle_counter_get()` is called directly in `bmx160.c:1141`, `bme280.c:196`,
and `attitude_task.c:122,135` (init at `main.c:142`). There is no
`vayu_timebase_now()` wrapper. The 32-bit-cycles-at-SYS_CLOCK_FREQ timestamp
*semantics* are an undocumented contract shared across `bmx160.h`, `bme280.h`,
`est.h` structs and `vayu_dt_from_cycles()` — coupled by convention. On a part
without DWT, or a different HAL, all four sites break independently. *(Note:
`attitude_task.c:122,135` is also a `hal_*` reach inside the estimation layer — a
direct logic→silicon call, `#if ATTITUDE_CYCLE_PROBE`, on by default.)*

### F10 🟠 — `SYS_CLOCK_FREQ` exists twice and can silently disagree
`vayu_dt_from_cycles()` divides DWT deltas by the hardcoded `SYS_CLOCK_FREQ = 84e6`
(`variables.h:12,24`) — and this dt feeds the **entire attitude/EKF/PID chain**
(`attitude_task.c:96`). But the real clock is set by an independent literal PLL
config in `main.c:42-61` with no compile-time link to the macro. Retune the PLL (or
hit the HSI fallback with a different M) and every gyro integration, derivative,
and integral is scaled wrong with **no error** — `boot.c:25` only flags the
mismatch and goes to FAILSAFE, it doesn't fix the math. Tellingly,
`attitude_task.c:76` already does it right with the runtime
`hal_cycle_counter_cycles_per_us()` — two notions of "cycles per second" coexist 20
lines apart.

### F11 🟠 — The logic core consumes the driver's chip-named sample type
The rate loop and attitude task pull `bmx160_all_reading_t` —
a type *named after and defined by* the BMX160 driver, which itself
`#include "navhal.h"` (`bmx160.h:4`) — e.g. `angle_rate_controller.c:177`,
`attitude_task.c:61`. Mitigation: it's a union and consumers only read the
`.converted` arm, whose fields are SI units, so the *data* is clean. But the
*type*, the `mag_fusion` layout, and its transitive `navhal.h` are the driver's, so
an IMU swap recompiles (and may force edits to) the control/est seam.

### F12 🟠 — IRQ wiring is split across subsystems with no ownership registry
USART1/6/2 + `DMA1_Stream6` + the global interrupt mask are owned by
`comm/channel.c` (`:85-88,114-115,248-251`); TIM5 (+ the HF tick + IMU pacing ISR)
by `sys/timer_callbacks.c` via `main.c:112,118`. Nothing answers "who owns IRQn
X", and nothing prevents two modules claiming the same vector — F4 is exactly a
latent version of that (channel.c would re-point `DMA1_Stream6_IRQn` away from the
iBus RX path that `rc_task.c` independently owns on USART2).

### F2b 🟠 — `i2c_config` is triple-aliased (a linkage footgun)
`hal_i2c_config_t i2c_config` is a global in `main.c:127`, a separate file-`static`
of the same name in `i2c_manager.c:11`, and an `extern` in `bmx160.c:83` (resolving
to main's global). They hold equal values *today*; edit one (e.g. tune
`clock_speed`) and the IMU recovery path may re-init the bus from a stale copy. A
driver reaching for a global bus-config object to re-init the bus is itself the
ownership inversion of F2.

### F13 — Hardware constants duplicated across the firmware/sim boundary
The ESC pulse band (`DEFAULT_MIN/MAX_PULSE_MS = 1.0/2.0`, `DEFAULT_PWM_FREQ 400`,
`esc.c:12-14`) is **re-derived by hand** in the SITL host HAL
(`host_navhal.c:144-146`, `esc_idle=0.4f`, `esc_full=0.8f`). Change the band in
`esc.c` and SITL silently diverges from hardware. This is the proof that "scattered
hardware facts" already costs you correctness across the sim seam, not just
portability.

## 4. The patterns underneath the catalogue

Every finding is an instance of one of four missing structures:

1. **No board seam (BSP).** F1, F7, F8, F10, F13 — board facts live in
   `variables.h` + driver bodies + task bodies + the sim, never one place. The
   compiler can't keep them consistent; the sim already drifts.
2. **No single peripheral owner.** F2, F3, F4, F5, F12, F2b — the I2C bus, a UART,
   a status LED, and the IRQ table each have a *de facto* owner that is the wrong
   module, or two owners that collide.
3. **No timebase seam.** F9, F10 — raw DWT reads + a duplicated clock constant.
4. **No capability port between logic and drivers.** F11 — logic speaks the
   driver's chip struct instead of an SI sample contract.

The control *algorithms* are clean; the **plumbing around them** is the problem.

## 5. The target seam

Three rules, each turning a pattern in §4 into something checkable:

1. **No `GPIO_*`, `HAL_*`, `TIM*`, `navhal.h`, AF number, or register literal
   appears above the driver/adapter layer.** Logic, control, est, protocol never
   name silicon.
2. **Every board fact is stated once.** "blue LED = PB12", "IMU = `HAL_I2C_1`@0x68",
   "sysclk = 84 MHz", "telemetry = UART6", "motor 1 = TIM1/CH1/PA08, AF1", "I2C1 RX
   = DMA1/S0/C1" — one board descriptor, referenced by symbolic role everywhere
   (including the sim, so F13 can't recur).
3. **Each shared peripheral has exactly one owner, and the allowed vaios spill goes
   through one vayu header.** The bus owner is `i2c_manager`, not the IMU. The LED
   has one arbiter, not heartbeat + router. Logic gets time from `sys/clock.h`, not
   raw `vaios.h`/DWT.

## 6. Mechanism: BSP, and what beats it

### Option A — a classic BSP (board support package)
One `board/<board>/` module owning the pin map, peripheral-instance assignment,
clock tree, DMA/IRQ assignment, and `board_init()`, exposing symbolic roles.

**Fit:** necessary — it directly dissolves F1/F7/F8/F10/F13 (scattered board
facts). A `board_navixsmf401.h` replacing the hardware half of `variables.h`
collapses a pin change to one file and gives the sim a single source to compile
against.

**Limit:** a BSP is a *data* fix. It does not by itself stop logic `#include`-ing
NavHAL, and re-wrapping every `hal_*` in a `board_*` is the **two-HALs problem**
(NavHAL is *already* a HAL — don't wrap a HAL in a HAL). A BSP also says nothing
about F2/F3/F4/F5 (ownership) — those are about *who calls*, not *where constants
live*. So: necessary, not sufficient.

### Option B — Ports & Adapters (hexagonal) — the better primary mechanism
Invert the dependency: logic declares the **narrow** interface it needs in
vayu-owned headers using plain C / SI types; thin adapters implement them against
NavHAL. Logic then compiles with NavHAL *off the include path*.

```
include/port/clock.h     uint32_t now_ticks(void);  float dt_seconds(uint32_t,uint32_t);
include/port/imu.h        bool imu_read(imu_sample_t *out);   // SI units, no chip struct  → kills F11
include/port/motors.h     void motors_write(const float duty[]);  // normalized, no TIM/pin → F7/F8
include/port/status_io.h  void status_led(led_id_t, bool);    // roles, one arbiter         → kills F5
include/port/i2c_bus.h    bus owner API; baro is a client, not a guest in the IMU  → F2/F3
                          ─────────────────────────────────────────────────────────────
src/port/stm32/*_adapter.c  // the ONLY firmware files (besides board/) that include navhal.h
```

Why it beats a BSP *here*: it enforces rule §5.1 mechanically (silicon headers
aren't reachable from logic — checkable with `grep`), and it sizes interfaces by
the *consumer's* need (4 floats, SI sample) so a sensor or MCU swap is one new
adapter, not a tree-wide recompile. It is also the seam your **SITL host already
needs** — `host_navhal.c` is *already* an alternate adapter; today it re-derives
constants (F13) because there's no shared port contract to implement.

### Option C — thin capability facade (the pragmatic overlap)
For the GPIO/LED/buzzer/UART-select concerns, expose only the *roles* vayu uses
(`status_led(STATUS_ARMED,true)`), not all of NavHAL. This is the minimal form of A∩B
and is the right weight for F5/F8.

### Option D — compile-time adapter selection (what vaios already does)
Pick `src/port/${VENDOR}/` at configure time off the existing `BOARD`/`VENDOR`
CMake vars — zero runtime indirection, the linker proves exactly one adapter is
bound. Right cost model for a single-airframe FC; don't reach for runtime vtables.

### Option E — build-enforced layering (the guardrail)
A CI/clang-tidy rule: no TU under `src/{control,est,maths}` may include `navhal.h`,
`hal_*.h`, or match `GPIO_|HAL_I2C|HAL_UART|TIM[0-9]`. You already gate warnings
per-module in `CMakeLists.txt`, so the rollout mechanism exists. This turns the
cleanup into an invariant instead of a one-off.

### Recommendation — layer them, smallest mechanism per concern
- **Thin BSP** (`include/board/board_navixsmf401.h`) for *board facts* — the one
  place pins/timers/DMA/IRQ/clock live (A+C). Fixes F1/F7/F8/F10/F13.
- **Hexagonal ports** for the *data path and shared peripherals* — `port/imu.h`,
  `port/motors.h`, `port/clock.h`, `port/status_io.h`, `port/i2c_bus.h`, with
  `src/port/stm32/` adapters as the only NavHAL includers (B). Fixes
  F2/F3/F5/F11 and makes the SITL host a real adapter (kills F13's duplication).
- **Compile-time selection** (D) keyed off `BOARD`/`VENDOR`.
- **One vaios-spill header** `sys/clock.h` so logic never includes `vaios.h` for
  time, and a single timebase wrapper for the DWT (fixes F9; pairs with F10).
- **CI layering guard** (E) so it can't regress.

## 7. Migration plan — fix the active bugs first, then the structure

Ordered by (active-defect first) then (highest blast-radius), each step
independently shippable:

1. **Active bug F4** — make `flush_channel` peripheral-agnostic (drive
   `s_handle->uart` via the HAL's DMA path) so telemetry stops byte-banging. Pure
   transport fix, no architecture needed.
2. **Active bug F5** — give the BLUE LED one owner: route the router's blink
   request *through* heartbeat (a `status_request()` call), delete the direct
   `hal_gpio_write` from `navlink_router.c`.
3. **Active gap F6** — wire `esc_disarm()`/`hal_pwm_stop` into the disarm path so
   there is a real hardware kill.
4. **Free wins** — delete the dead `navhal.h` includes in
   `angle_rate_controller.c:10` / `sensor_fusion.c:3`; collapse the `i2c_config`
   aliasing (F2b) to one definition.
5. **Split `variables.h`** (F1) into `tuning.h` (pure, no NavHAL) +
   `board_navixsmf401.h` (hardware); repoint the 26 includers. Most only want
   tuning and immediately stop pulling silicon. Add the CI guard (E) scoped to
   `control/est/maths`.
6. **Timebase seam** (F9/F10): `port/clock.h` + `sys/clock.h`; one DWT wrapper that
   derives its rate at runtime (follow `attitude_task.c:76`); move logic off
   `vaios.h`-for-time.
7. **`port/imu.h` + bmx160 adapter** (F11): hand the control/est seam an SI
   `imu_sample_t`; `est/` compiles NavHAL-free.
8. **Make `i2c_manager` the real bus owner** (F2/F3): move the FAST/MAG/TEMP/BARO
   scheduling + DMA fan-out out of `bmx160.c` into the manager; the baro becomes a
   registered bus client, not a guest hard-wired into the IMU's source.
9. **`port/motors.h` + board motor table** (F7/F8): one timer owner sets the base,
   channels attach; AF/pin/channel map moves to `board_*.h`.
10. **Widen the CI guard to the whole logic tree** once it's clean.

## 8. The "controlled spill" rule, written down

| Symbol class | Example | Allowed in `control`/`est`/`maths`? |
|---|---|---|
| vaios portable service | `v_get_ticks`, `task_delay_until`, IPC, `memory.h` | **Yes**, via a vayu-owned `sys/*.h` re-export, not raw `vaios.h` |
| vaios port | `portable/cortex-m4`, `port.h` | **No** (already clean — keep it) |
| NavHAL capability API | `hal_i2c_*`, `hal_gpio_*`, `hal_cycle_counter_get` | **No** — only in `src/port/**` adapters + `board/` |
| Board facts | `GPIO_PB12`, `HAL_I2C_1`, `TIM1`, register addrs, AF | **No** — only in `board_*.h` |

The line is: *time and scheduling are OS concepts and may spill; pins, buses,
chips, timers, and registers are silicon and may not.* Exactly the policy you
stated — now checkable in CI.

## 9. Bottom line

- The architecture (vayu → vaios → NavHAL, NavHAL already a HAL, vaios cleanly
  ported) is sound, and the **control/estimation math is genuinely portable**. You
  do **not** need a heavyweight framework.
- The damage is concentrated in the **plumbing**: no board seam, no single
  peripheral owners, no timebase seam. A **thin BSP** (board data) + **hexagonal
  ports** (driver data-path & shared peripherals) + **compile-time adapter
  selection** + **CI-enforced layering** closes all 13 fault lines and turns the
  SITL host from a copy-paste twin into a real second adapter.
- **Three of the fault lines are bugs today** — telemetry byte-banging (F4), the
  BLUE-LED owner collision (F5), and the absent hardware motor-kill (F6) — and they
  are fixable independently, before any structural work. Do those first.

### Severity index
| # | Fault line | Evidence | Class |
|---|---|---|---|
| F1 | `variables.h` god-header welds tree to silicon | variables.h:8-12,33-62 (26 includers) | 🔴🔴 structural |
| F2 | I2C bus owned by IMU driver, not i2c_manager | bmx160.c:72-85,859-940 | 🔴🔴 structural |
| F3 | IMU driver reads the barometer (cross-driver) | bmx160.c:3,896-902,1006-1011 | 🔴🔴 structural |
| F4 | Telemetry TX byte-bangs (channel hardcoded UART2) | main.c:72 vs channel.c:196-208 | 🔴 ACTIVE bug |
| F5 | Two owners fight over BLUE LED | heartbeat.c vs navlink_router.c:41,52,58 | 🔴 ACTIVE bug |
| F6 | No hardware motor-kill; esc_disarm dead | esc.c:51 (no callers), motor.c:52-57 | 🔴 ACTIVE gap |
| F7 | TIM1 re-inited per-channel, no single owner | motor.c:23-26, esc.c:43 | 🟠 latent |
| F8 | STM32 AF pinmux hardcoded in ESC driver | esc.c:31-37 | 🟠 latent |
| F9 | Raw DWT read in 4 modules, no timebase seam | bmx160.c:1141, bme280.c:196, attitude_task.c:122,135 | 🟠 structural |
| F10 | SYS_CLOCK_FREQ duplicated vs PLL config | variables.h:12,24 vs main.c:42-61 | 🟠 latent |
| F11 | Logic consumes chip-named `bmx160_all_reading_t` | angle_rate_controller.c:177, attitude_task.c:61 | 🟠 structural |
| F12 | IRQ wiring split, no ownership registry | channel.c:85-115 vs timer_callbacks.c | 🟠 structural |
| F2b | `i2c_config` triple-aliased | main.c:127, i2c_manager.c:11, bmx160.c:83 | 🟠 latent |
| F13 | Hardware constants duplicated in SITL host | esc.c:12-14 vs host_navhal.c:144-146 | 🟠 correctness |
