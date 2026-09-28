# Boards

One directory per flight-controller board. Each ships a `vayu_board.h`
defining the same macro names, so nothing outside this directory names a pin,
a bus or a timer.

```
firmware/board/
  boards.cmake          selection + the NavHAL-board consistency check
  navixdev/             F401 development FC  (current)
    vayu_board.h
```

Roadmap: `navixdev` → `navixsm_f401re` → `f441` → `h767`. The last is a
different Cortex-M core, so the port type headers and the register addresses
below both change — which is the point of having this layer at all.

## Selecting one

```sh
cmake -S firmware -B build -DVAYU_BOARD=navixdev
```

Consumers write `#include "vayu_board.h"` and never name a board. The selected
directory is the only one on the include path.

**`VAYU_BOARD` and `navhal.config` must agree.** They are two independent
selections: NavHAL's `CONFIG_BOARD` drives the linker script and the MCU
family, vayu's drives the pins. A mismatch builds F4 pin numbers against an H7
linker script and says nothing, so each board declares the NavHAL board it
belongs on and `boards.cmake` fails the configure if the two disagree.

## The contract

Every `vayu_board.h` defines all of these. A board that cannot (no buzzer, a
different bus topology) still defines the name — omitting one moves the
`#ifdef` into the consumers, which is what this layer exists to prevent.

| Macro | What |
|---|---|
| `VAYU_BOARD_NAME` | identity string |
| `VAYU_BOARD_NAVHAL_BOARD` | the NavHAL board this must be built against |
| `BOARD_LED_BLUE` `BOARD_LED_GREEN` `BOARD_LED_RED` | status LEDs |
| `BOARD_BUZZER` | buzzer pin |
| `BOARD_I2C_BUS` `BOARD_I2C_SCL` `BOARD_I2C_SDA` | the sensor bus |
| `BOARD_I2C_DR_ADDR` | that bus's `DR` address, for DMA |
| `BOARD_IMU_I2C_ADDR` | IMU 7-bit address |
| `BOARD_ESC_TIMER` `BOARD_ESC_M1_PIN`..`M4_PIN` | motor outputs, mixer order |

Include only the narrow port type headers (`utils/gpio_types.h`,
`utils/i2c_types.h`, `utils/timer_types.h`), never `navhal.h`. Naming a pin
must not cost the caller sixteen subsystem headers. This is the include set
NavHAL's own board files use.

Sensor *tuning* is not a board fact and does not belong here — ODRs, ranges
and bandwidths live with their driver (`include/driver/bmx160.h`).

## Adding a board

1. `cp -r navixdev <name>` and edit every macro in the table.
2. Point `VAYU_BOARD_NAVHAL_BOARD` at the NavHAL board it runs on, and set the
   matching `CONFIG_BOARD` / `CONFIG_BOARD_*` / `CONFIG_FAMILY_*` lines in
   `firmware/navhal.config`.
3. Build with `-DVAYU_BOARD=<name>`.

A new MCU family also needs its NavHAL side (port, family, linker script) to
exist first — that is NavHAL's Kconfig, not this directory.

## Why `vayu_board.h` and not `board.h`

NavHAL's selected board directory is already on our include path and already
ships a `board.h`. Two files of that name means the build picks one by `-I`
order, and the loser is a board compiled with another board's pins.

This layout mirrors NavHAL's own `src/board/<name>/board.h` deliberately. When
a board graduates into NavHAL it is a `git mv`, a `Kconfig` + `Kconfig.choice`
pair, and the rename to `board.h`; vayu's copy and this selection machinery are
then deleted.
