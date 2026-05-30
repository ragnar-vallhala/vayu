# Phase 4 — per-module warning rollout

Tracks the CONV-06 / R1.2 compiler-flag widening. Each module is brought
to structural conformance (R2.1 umbrella header, R2.6 folder placement,
R10.3 magic-number sweep) and then its translation units are flipped to
the full R1.2 warning set with `-Werror`.

## R1.2 warning set

```
-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wcast-align
-Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes -Wundef
-Wfloat-equal -Werror
```

Applied per-TU in the **target build** (`CMakeLists.txt`, via
`VAYU_R12_CLEAN_SOURCES`). The rest of the tree stays at the baseline
`-Wall` until its module's turn. Verified on `arm-none-eabi-gcc`.

## Status

Order (per alignment-plan): SNS → EST → CTRL → ACT → COMM → LOG → SYS.
Started with **ACT** to prove the per-module mechanism on the smallest,
already-clean module before the heavier ones.

| Module | TUs | Umbrella header | Folder move (R2.6) | Warning level | Status |
|--------|-----|-----------------|--------------------|---------------|--------|
| ACT    | `actuator/esc.c`, `actuator/motor.c` | `include/actuator/actuator.h` (merged esc.h+motor.h) | none (already a module) | **full R1.2 + `-Werror`** | ✅ done |
| SNS    | `sensor/{bmx160,imu_buffer,i2c_manager}.c` | `include/sensor/sensor.h` (facade — driver header kept) | ✅ `drivers/` folded into `src/sensor/` | **full R1.2 + `-Werror`** | ✅ done |
| EST    | `est/sensor_fusion.c`, `est/lpf.c` | `include/est/est.h` (merged sensor_fusion.h+lpf.h) | ✅ `maths/{sensor_fusion,lpf}.c` → `src/est/` | **full R1.2 + `-Werror`** | ✅ done |
| CTRL   | `control/{angle_controller,angle_rate_controller,pid_config,pid,control_buffer}.c` | `include/control/control.h` | ✅ `maths/{pid,control_buffer}.c` → `src/control/` | **full R1.2 + `-Werror`** | ✅ done |
| COMM   | `comm/*.c` | `include/comm/comm.h` | none | `-Wall` | ⬜ pending (~20 warns) |
| LOG    | `logger/logger.c` (+`utils/utils.c` → `logger/`) | `include/logger/logger.h` (exists) | `utils/utils.c` → `src/logger/` | `-Wall` | ⬜ pending (~6 warns) |
| SYS    | `sys/*.c` | `include/sys/sys.h` | `utils/{math_utils,timer_callbacks,types}.h` → `include/sys/` | `-Wall` | ⬜ pending (~13 warns) |

Warning counts are the R1.2 load measured on the current tree (pre-fix);
they are the work each module's PR clears before its `-Werror` flip.

## Mechanism

`CMakeLists.txt` defines `VAYU_R12_WARN_FLAGS` (the set above) and
`VAYU_R12_CLEAN_SOURCES` (the conformant TUs). Bringing a module in =
fix its warnings, then add its TUs to `VAYU_R12_CLEAN_SOURCES`. A
negative-control check (inject a warning → build must fail) confirms the
flags bite.
